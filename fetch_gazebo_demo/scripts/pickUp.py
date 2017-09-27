#!/usr/bin/env python
#ADD LICENCE

# Author: Michael Ferguson
# Author: Di Sun

import copy
import actionlib
import time
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == "__main__":
    # Create a node
    rospy.init_node("pickUp")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    def callback(msg):
        global grasps
        grasps = msg.grasps


    move_group = MoveGroupInterface("arm","map")
    simplePose = PoseStamped()
    simplePose.pose.position.x =0

    simplePose.pose.position.y =0 
    simplePose.pose.position.z =0  

    simplePose.pose.orientation.x = 0.46
    simplePose.pose.orientation.z = -0.5
    simplePose.pose.orientation.y = 0.51
    simplePose.pose.orientation.w = 0.52

    simplePose.header.frame_id = 'gripper_link'
    #simplePose.header.stamp = rospy.Time.now()
    move_group.moveToPose(simplePose,'gripper_link')

    


    find_topic = 'detect_grasps/clustered_grasps'


#    sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)
#    grasps = []


    # Wait for grasps to arrive.
#    rate = rospy.Rate(1)

#    while not rospy.is_shutdown():    
#        if len(grasps) > 0:
           # rospy.loginfo('Received %d grasps.', len(grasps))
           # break
     
