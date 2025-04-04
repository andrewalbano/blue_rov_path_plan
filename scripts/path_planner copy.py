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
    def __init__(self):
        # creating subscribers
        self.controller_state_sub = rospy.Subscriber('motion_controller_state', String, self.controller_state_callback)
        self.pose_sub = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        self.reset_waypoints_sub= rospy.Subscriber('reset_waypoints',Bool, self.reset_waypoints_callback)
        # self.waypoint_list_sub = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)
        self.add_waypoint_sub = rospy.Subscriber('add_waypoint', PoseStamped, self.add_waypoint_callback)
        self.hold_pose_sub = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)

        # self.int_poses_sub = rospy.Subscriber('int_poses', PoseArray, self.int_poses_sub)

        # publishers
        self.waypoint_pub = rospy.Publisher('current_waypoint', PoseStamped, queue_size=1)

        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)

        # motion controller state
        self.state = "Disabled"

        # Determine what to publish. either holding the current pose or the next waypoint
        self.hold_pose = True
        self.hold_pose_sent = False # i only wnat to send the variable once not repeately so flag if sent

        self.hold_pose_waypoint = PoseStamped()
        self.current_waypoint = PoseStamped()
        self.current_waypoint.header.frame_id ="NED"
        self.waypoint_yaw = None
        self.current_waypoint_index = None  

        self.current_int_waypoint = PoseStamped()
        self.current_int_waypoint_index = None  

        




        # Current Pose 
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id ="NED"
        self.current_roll,self.current_pitch,self.current_yaw = None, None, None

        # list of the target waypoints
        self.target_waypoints = PoseArray()
        self.target_waypoints.header.frame_id = "NED"
        self.num_waypoints = None
        self.target_waypoint_sent =False

        # self.sub4 = rospy.Subscriber('wapoint_index_reset', Int8, self.waypoint_index_callback)
        # self.sub5 = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)
        # self.sub6 = rospy.Subscriber('hold_pose_waypoint', PoseStamped, self.hold_pose_waypoint_callback)
    


    def controller_state_callback(self, msg:String):
        self.state = msg.data
        if self.state == "waypoint":
            self.hold_pose_waypoint.header.frame_id = self.current_pose.header.frame_id
            self.hold_pose_waypoint.pose.position = self.current_pose.pose.position
            self.hold_pose_waypoint.pose.orientation = self.current_pose.pose.orientation
            self.hold_pose_sent = False 
    
    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
       
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation
        self.current_roll,self.current_pitch,self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # def waypoint_list_callback(self, msg:PoseArray):
    #     # checks if it is a list and get how many waypoint
    #     if isinstance(msg.poses, list):
    #         self.num_waypoints = len(msg.poses)
    #         rospy.loginfo("Received " + str(self.num_waypoints) + " waypoints")
    #     else:
    #         self.num_waypoints = 1
    #         rospy.loginfo("Received 1 waypoint")


    #     # assigns waypoints
    #     self.target_waypoints = msg

    # def int_poses_callback(self, msg:PoseArray):
    #     self.target_waypoints.poses.append(msg.pose)
    #     self.num_waypoints = len(self.target_waypoints.poses)

    def add_waypoint_callback(self, msg:PoseStamped):
        self.target_waypoints.poses.append(msg.pose)
        self.num_waypoints = len(self.target_waypoints.poses)

    def hold_pose_callback(self,msg:Bool):
        self.hold_pose = msg.data
        if self.hold_pose:
            self.hold_pose_waypoint.header.frame_id = self.current_pose.header.frame_id
            self.hold_pose_waypoint.pose.position = self.current_pose.pose.position
            self.hold_pose_waypoint.pose.orientation = self.current_pose.pose.orientation
            self.hold_pose_sent = False

            
        else:
            self.hold_pose_sent = False
            self.target_waypoint_sent =False

    def reset_waypoints_callback(self, msg:Bool):
        if msg:
            self.target_waypoints.poses.clear()
            # self.target_waypoints.poses=[]
            self.num_waypoints = None
            self.target_waypoint_sent = False

       

    def waypoint_reached(self, current, target):
        _,_,target_yaw = euler_from_quaternion([target.pose.orientation.x,target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w])
        _,_,current_yaw = euler_from_quaternion([current.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])


        # calculate the yaw error to final orientation
        yaw_error = target_yaw - current_yaw
         # Normalize yaw error to [-pi, pi] range
        if yaw_error> np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        # calculate position errors in the NED FRAME
        error_x = target.pose.position.x - current.pose.position.x
        error_y = target.pose.position.y - current.pose.position.y
        error_z = target.pose.position.z - current.pose.position.z
        distance = np.linalg.norm((error_x, error_y, error_z))

        if distance < 0.3:
            position_reached = True
        else: 
            position_reached = False

        if yaw_error < 20*np.pi/180:
            orientation_reached = True
        else: 
            orientation_reached = False

        if position_reached and orientation_reached:
            return True
        else:   
            return False

    def get_current_waypoint(self):
    

        # if not self.num_waypoints == None:
            # rospy.loginfo_once("testing this loop")
            # if this is the first time setting it 
        if self.current_waypoint_index == None:
            self.current_waypoint_index = 0
            # rospy.loginfo_once("testing this loop")
        if self.target_waypoints.poses:        
            self.current_waypoint.header.frame_id = self.target_waypoints.header.frame_id
            self.current_waypoint.pose = self.target_waypoints.poses[self.current_waypoint_index]

            _,_,self.waypoint_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])
            # rospy.loginfo_once(self.current_waypoint.pose)
        else:
            rospy.logerr("No waypoints in path planner")
    

def main():
    # Initialize the ROS node
    rospy.init_node('waypoint_manager')
    
    # initialize the waypoint manager
    wp = WaypointManager()

    while not rospy.is_shutdown():

        
        if wp.state == "waypoint":
            rospy.loginfo_once("started wp mode")
            
            # activate hold pose mode, only send the waypoint once
            if wp.hold_pose and not wp.hold_pose_sent:
                wp.waypoint_pub.publish(wp.hold_pose_waypoint)
                wp.hold_pose_sent = True
                wp.target_waypoint_sent = False # reset the flag for the waypoint follower loop
                rospy.loginfo("Sent hold position")
            elif not wp.hold_pose:
                # rospy.loginfo_once("testing this loop")
                wp.get_current_waypoint()
                
                try:
                    # for publisbhing the first waypoint
                    if not wp.target_waypoint_sent:
                        wp.waypoint_pub.publish(wp.current_waypoint)
                        wp.target_waypoint_sent = True
              

                    
                    if wp.waypoint_reached(wp.current_pose,wp.current_waypoint):
                        
                        if wp.current_waypoint_index +1 < wp.num_waypoints:
                            wp.current_waypoint_index +=1
                            wp.get_current_waypoint()
                           
                            wp.waypoint_pub.publish(wp.current_waypoint)
                            
                            rospy.loginfo_once("sent new target waypoint")
                        elif wp.current_waypoint_index +1 == wp.num_waypoints:
                            rospy.logwarn_throttle(5,"Reached last waypoint")

                        else:
                            rospy.loginfo_throttle(5,"Heading to waypoint")
                except: 
                    rospy.logerr("Error with sending waypoint")
            else:
                rospy.loginfo_throttle(5,"Holding Pose")
   
        wp.rate.sleep()





if __name__ == "__main__":

    main()