#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion, euler_matrix
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray, String
import time


class WaypointManager:
    def __init__(self,master, mode = "hardware"):
        # creating subscribers
        self.controller_state_sub = rospy.Subscriber('motion_controller_state', String, self.controller_state_callback)
        self.pose_sub = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        


        self.state = "Disabled"
        self.current_pose = PoseStamped()
        self.current_roll,self.current_pitch,self.current_yaw = None, None, None
        self.sub3 = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)



        
        self.sub4 = rospy.Subscriber('wapoint_index_reset', Int8, self.waypoint_index_callback)
        self.sub5 = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)
        self.sub6 = rospy.Subscriber('hold_pose_waypoint', PoseStamped, self.hold_pose_waypoint_callback)
        self.sub7 = rospy.Subscriber('controller_gains', Float32MultiArray, self.gui_info_callback)
        # self.sub8 = rospy.Subscriber('sitl_current_velocity', TwistStamped, self.velocity_callback)
        self.sub8 = rospy.Subscriber('/dvl/twist', TwistStamped, self.velocity_callback)
       
        
    def controller_state_callback(self, msg:String):
        self.state = msg.data            
    
    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
       
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation
        self.current_roll,self.current_pitch,self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])




