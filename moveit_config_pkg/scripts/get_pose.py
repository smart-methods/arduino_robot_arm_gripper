#!/usr/bin/env python

import rospy
import tf
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose

if __name__=="__main__":
	
	rospy.init_node("simple_marker")
	#int_marker = InteractiveMarker()
	#menu_handler = MenuHandler()
	listener=tf.TransformListener()
	display_traj_pub=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
	#marker_pub= rospy.Publisher('/via_points',MarkerArray,queue_size=20)

	# moveit start

	#moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "arm"
	group = moveit_commander.MoveGroupCommander(group_name)
	

	listener=tf.TransformListener()
	listener.waitForTransform('/arm3','/base',rospy.Time(), rospy.Duration(1.0))
	print listener.frameExists('base')
	print listener.frameExists('arm3')
 	(trans,rot)=listener.lookupTransform('base','arm3',rospy.Time())
	print trans,rot

	rospy.spin()
