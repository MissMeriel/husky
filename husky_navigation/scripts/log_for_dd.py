#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import LaserScan
import tf
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from move_base_msgs.msg import MoveBaseActionGoal
import math

spawn_pose = [0.0, 0.0, 0.0]
goal = None #[0.0, 0.0, 0.0]
posefile = None
epsilon = 0.05
current_pose = None
epsilon_goal = 0.1

def pose_callback(data):
   global current_pose
   current_pose = data

def action_goal_callback(data):
   global goal, posefile
   goal = data.goal.target_pose.pose
   rospy.loginfo(rospy.get_caller_id() + "GOAL INITIALIZED AS: "+str(goal))
   goal = parse_goal_message(goal)
   posefile.write("GOAL {} \n".format(goal))

# can also parse tf msg
def parse_goal_message(data):
   msglist = [data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
   #rospy.loginfo(rospy.get_caller_id() + " goal message list:" + str(msglist))
   return msglist

def parse_message(trans, rot):
   trans.extend(rot)
   return str(trans)

def distance(p1, p2):
   return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))

def check_goal_proximity(trans, rot):
   global epsilon_goal, goal
   if goal == None:
      return False
   if distance(goal, trans[0:3]) < epsilon_goal:
      return True
   return False

def main():
   global posefile, current_pose
   ### anonymous adds number
   rospy.init_node('pose_logger', anonymous=False)
   listener = tf.TransformListener()
   ### vvvvv this breaks move_base action server vvvvv
   #rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback) 
   rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, action_goal_callback)

   ### open pose file
   dirname = os.path.dirname(__file__)
   filename = os.path.join(dirname, 'poses.log')
   
   posefile = open(filename, 'w')
   print("\n\nDirname is "+str(dirname)+"\nPublishing pose logs to "+str(posefile)+"\n\n")
   ### write a pose every second
   rate = rospy.Rate(3)
   trans = None
   rot = None
   while not rospy.is_shutdown():
      try:
         (trans,rot) = listener.lookupTransform('odom', '/base_link', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         continue
      msg_string = parse_message(trans, rot)
      posefile.write(msg_string+"\n")
      near_goal = check_goal_proximity(trans, rot)
      if (near_goal):
         posefile.write("GOAL REACHED\n")
      rate.sleep()
   posefile.close()
   

if __name__ == '__main__':
   main()