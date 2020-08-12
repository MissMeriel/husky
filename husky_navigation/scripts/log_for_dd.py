#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import LaserScan
import tf
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from move_base_msgs.msg import MoveBaseActionGoal


spawn_pose = [0.0, 0.0, 0.0]
goal = [0.0, 0.0, 0.0]
posefile = None
epsilon = 0.05
current_pose = None

def pose_callback(data):
   global current_pose
   current_pose = data
   
def goal_callback(data):
   global goal
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
   #print("THIS IS THE GOAL: "+str(goal))
   goal = data
   #print("THIS IS THE INITIALIZED GOAL: "+str(goal))

def action_goal_callback(data):
   global goal, posefile
   goal = data.goal.target_pose.pose
   rospy.loginfo(rospy.get_caller_id() + " THIS IS THE INITIALIZED GOAL: "+str(goal))
   msg_string = parse_goal_message(goal)
   posefile.write("GOAL {} \n".format(msg_string))

# can also parse tf msg
def parse_goal_message(data):
   msglist = [data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
   rospy.loginfo(rospy.get_caller_id() + " goal message list:" + str(msglist))
   return str(msglist)

def parse_message(trans, rot):
   trans.extend(rot)
   return str(trans)

def main():
   global posefile, current_pose
   # anonymous adds number
   rospy.init_node('pose_logger', anonymous=False)
   listener = tf.TransformListener()
   #rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
   rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, action_goal_callback)

   # open pose file
   dirname = os.path.dirname(__file__)
   filename = os.path.join(dirname, 'poses.log')
   posefile = open(filename, 'w')
   # write a pose every second
   rate = rospy.Rate(1)
   trans = None
   rot = None
   while not rospy.is_shutdown():
      #rospy.loginfo("INSIDE ROSPY LOOP")
      try:
         (trans,rot) = listener.lookupTransform('odom', '/base_link', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         continue
      #trans = [1, 2, 3]
      #rot = [4, 5, 6]
      msg_string = parse_message(trans, rot)
      posefile.write(msg_string+"\n")
      rate.sleep()
   posefile.close()
   

if __name__ == '__main__':
   main()