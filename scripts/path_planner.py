#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion, euler_matrix, quaternion_from_euler
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray, String,Int8MultiArray
import time
import dubins


class WaypointManager:
    def __init__(self):
        # Current Pose 
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id ="NED"
        self.current_roll,self.current_pitch,self.current_yaw = None, None, None

        # creating subscribers
        self.controller_state_sub = rospy.Subscriber('motion_controller_state', String, self.controller_state_callback)
        self.pose_sub = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        self.reset_waypoints_sub= rospy.Subscriber('reset_waypoints',Bool, self.reset_waypoints_callback)
        # self.waypoint_list_sub = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)
        self.add_waypoint_sub = rospy.Subscriber('add_waypoint', PoseStamped, self.add_waypoint_callback)
        self.hold_pose_sub = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)

        # self.int_poses_sub = rospy.Subscriber('int_poses', PoseArray, self.int_pose_callback)
        self.orientation_style_sub= rospy.Subscriber("path_orientation_style", Int8MultiArray,self.orientation_style_callback)
        self.parameters_sub = rospy.Subscriber("path_planner_parameters", Float32MultiArray,self.parameters_callback)

        # publishers
        self.goal_waypoint_pub = rospy.Publisher('current_goal_waypoint', PoseStamped, queue_size=1)
        self.lookahead_waypoint_pub = rospy.Publisher('lookahead_waypoint', PoseStamped, queue_size=1)


        # run frequency
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)

        # motion controller state
        self.state = "Disabled"

        # Determine what waypoint type to publish. either holding the current pose or the next waypoint
        # i only wnat to send the variable once not repeately so flag if sent, consider making this a service message
        self.hold_pose = True
        self.hold_pose_sent = False 

        #  The waypoint to hold position at
        self.hold_pose_waypoint = PoseStamped()

        # the current waypoint 
        self.current_waypoint = PoseStamped()
        self.current_waypoint.header.frame_id ="NED"
        self.waypoint_yaw = None
        self.current_waypoint_index = None  

       # previous waypoint
        self.prev_waypoint = PoseStamped()
        self.prev_waypoint.header.frame_id ="NED"

        # next waypoint
        self.next_waypoint = PoseStamped()
        self.next_waypoint.header.frame_id ="NED"


        # the waypoint used to guide the robot, it is a point on the desired path with orientation specified
        self.lookahead_waypoint = PoseStamped()
        self.lookahead_waypoint.header.frame_id = "NED"

        # list of the target waypoints
        self.target_waypoints = PoseArray()
        self.target_waypoints.header.frame_id = "NED"
        self.num_waypoints = None
        self.target_waypoint_sent =False

        #  the default orientation style when needed is RTR
        self.path_orientation_style = 4

        self.orientation_style = Int8MultiArray()

        # need to make these adjustable in the user interface
        self.position_threshold = 0.2   
        self.yaw_threshold = 10*np.pi/180
        self.lookahead_distance = 0.5
        self.lookahead_past_waypoint = 0


        self.alg = 2

    
    def controller_state_callback(self, msg:String):
        # activate t
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


    # def int_pose_callback(self, msg:PoseArray):
    #     self.waypoints.poses= msg.poses
    #     # self.num_waypoints = len(self.target_waypoints.poses)

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
            self.orientation_style.data = []
            # self.target_waypoints.poses=[]
            self.num_waypoints = None
            self.current_waypoint_index = None
            self.target_waypoint_sent = False

    def orientation_style_callback(self, msg:Int8MultiArray):
        self.orientation_style = msg

    def parameters_callback(self,msg:Float32MultiArray):
        
        if msg.data[0] == 1:
            self.position_threshold = msg.data[1] 
        elif msg.data[0] == 2:
            self.yaw_threshold = msg.data[1]*np.pi/180
        elif msg.data[0] == 3:
            self.lookahead_distance = msg.data[1] 

        elif msg.data[0] == 4:
            self.lookahead_past_waypoint = msg.data[1] 

        elif msg.data[0] == 5:
            self.alg = msg.data[1] 
                   
    def waypoint_reached(self, current:PoseStamped, target:PoseStamped):
        _,_,target_yaw = euler_from_quaternion([target.pose.orientation.x,target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w])
        _,_,current_yaw = euler_from_quaternion([current.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])


        # calculate the yaw error to final orientation
        yaw_error = target_yaw - current_yaw
        #  Normalize yaw error to [-pi, pi] range
        if yaw_error> np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        # calculate position errors in the NED FRAME
        error_x = target.pose.position.x - current.pose.position.x
        error_y = target.pose.position.y - current.pose.position.y
        error_z = target.pose.position.z - current.pose.position.z
        distance = np.linalg.norm((error_x, error_y, error_z))

        if distance < self.position_threshold:
            position_reached = True
        else: 
            position_reached = False

        if abs(yaw_error) < self.yaw_threshold:
            orientation_reached = True
        else: 
            orientation_reached = False

        if position_reached and orientation_reached:
            # rospy.loginfo(f"yaw error = { yaw_error*180/np.pi}")
            return True
        else:   
            return False

    def position_reached(self, current:PoseStamped, target:PoseStamped):
       
        # calculate position errors in the NED FRAME
        error_x = target.pose.position.x - current.pose.position.x
        error_y = target.pose.position.y - current.pose.position.y
        error_z = target.pose.position.z - current.pose.position.z
        distance = np.linalg.norm((error_x, error_y, error_z))

        if distance < self.position_threshold:
            return True
        else: 
            return False
             
    def get_current_waypoint(self):
    

        # if not self.num_waypoints == None:
            # rospy.loginfo_once("testing this loop")
            # if this is the first time setting it 
        try:
            
            if self.current_waypoint_index == None:
                self.current_waypoint_index = 0
                # rospy.loginfo_once("testing this loop")
            if self.target_waypoints.poses:        
                self.current_waypoint.header.frame_id = self.target_waypoints.header.frame_id
                self.current_waypoint.pose = self.target_waypoints.poses[self.current_waypoint_index]

                _,_,self.waypoint_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])
                # rospy.loginfo_once(self.current_waypoint.pose)

                try:
                    #  work around to publist as one list 
                    self.path_orientation_style = self.orientation_style.data[self.current_waypoint_index*2]
                    self.alg = self.orientation_style.data[self.current_waypoint_index*2+1]
                    # self.path_orientation_style = self.orientation_style.data[self.current_waypoint_index,0]
                    # self.alg = self.orientation_style.data[self.current_waypoint_index,1]
                except: 
                    self.current_waypoint.header.frame_id = self.current_pose.header.frame_id
                    self.current_waypoint.pose = self.current_pose.pose
                
                    # self.path_orientation_style = 4
                    rospy.logwarn(f"could not get path orientation style from the array for index {self.current_waypoint_index}, using current pose as waypoint")
                

                    # rospy.logwarn(f"could not get path orientation style from the array for index {self.current_waypoint_index}")
                
            else:
                rospy.logerr("No waypoints in path planner")

        except:
            self.current_waypoint.pose = self.current_pose.pose
            rospy.logwarn(f"could not get waypoint for index {self.current_waypoint_index}, using current pose as waypoint")           
            
    def carrot_chasing(self):
        # let d  = crosstrack error 
        self.get_current_waypoint()
        
       
        r1 = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        r2 = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        ru = np.linalg.norm(np.array(r1,r2))
        theta = np.arctan2(self.lookahead_waypoint.pose.position.y- self.current_waypoint.pose.position.y,self.lookahead_waypoint.pose.position.x- self.current_waypoint.pose.position.x) 
        theta_u = np.arctan2(self.current_pose.pose.position.y-self.current_waypoint.pose.position.y, self.current_pose.pose.position.x-self.current_waypoint.pose.position.x)
        beta = theta-theta_u
        R = np.sqrt((ru**2)-(ru*np.sin(beta)**2))
        x_t_prime = (R + self.delta) * np.cos(theta)
        y_t_prime = (R + self.delta) * np.sin(theta)
        carrot_point = (x_t_prime, y_t_prime,0)
        # self.heading = np.arctan2(y_t_prime-self.current_pose.pose.position.y,x_t_prime-self.current_pose.pose.position.x)
        # u = self.Kp_yaw()

    def get_prev_waypoint(self):
        # if self.current_waypoint_index == 0:
        #     self.prev_waypoint = None
        # elif self.current_waypoint_index > 0:
        #     self.prev_waypoint = self.target_waypoints.poses[self.current_waypoint_index-1]
        self.prev_waypoint = PoseStamped()
        if self.current_waypoint_index > 0:
        
            self.prev_waypoint.header.frame_id = self.target_waypoints.header.frame_id
            self.prev_waypoint.pose = self.target_waypoints.poses[self.current_waypoint_index-1]
        else:
            self.prev_waypoint.header.frame_id = self.hold_pose_waypoint.header.frame_id
            self.prev_waypoint.pose = self.hold_pose_waypoint.pose
            
    def get_next_waypoint(self):
        self.next_waypoint = PoseStamped()

        if self.num_waypoints > 1 and self.current_waypoint_index < self.num_waypoints-1:
            self.next_waypoint.header.frame_id = self.target_waypoints.header.frame_id
            self.next_waypoint.pose = self.target_waypoints.poses[self.current_waypoint_index+1]
        else:
            self.next_waypoint.header.frame_id = self.current_waypoint.header.frame_id
            self.next_waypoint.pose = self.current_waypoint.pose

    def get_lookahead_waypoint(self):
        # add various method of obtaining lookahead point here, start with position, once that works move to orientation styles
        # self.alg = 1

        # method 1: find the point on the line between previous waypoint and current waypoint that is closest to current pose and set the look ahead point as the point d distance away 
        if not self.prev_waypoint == None: # possibly add: and not self.next_waypoint == None
            if self.alg == 1 and not self.next_waypoint ==None:
                self.lookahead_waypoint = self.lookahead_method1()
                # self.lookahead_waypoint = self.lookahead_method2(self.current_waypoint, self.prev_waypoint)
            elif self.alg == 2 and not self.next_waypoint ==None: #and not self.next_waypoint==self.current_waypoint:
                self.lookahead_waypoint =self.lookahead_method2(self.current_waypoint, self.prev_waypoint)
                
            else:
                rospy.logwarn("could not use the requested alg, defaulting to alg 2")
                self.lookahead_waypoint =self.lookahead_method2(self.current_waypoint, self.prev_waypoint)
            
        else: 
            self.lookahead_waypoint.pose = self.current_waypoint.pose

    def lookahead_method1(self):
        # setting look ahead position
        lookahead_pose = PoseStamped()
        lookahead_pose.header.frame_id = "NED"

        if self.prev_waypoint == self.current_waypoint:
            rospy.loginfo("prev waypoint = current waypoint check")
            # pass
        else:
            p1  = np.array([self.prev_waypoint.pose.position.x,self.prev_waypoint.pose.position.y, self.prev_waypoint.pose.position.z])
            p2  = np.array([self.current_waypoint.pose.position.x,self.current_waypoint.pose.position.y, self.current_waypoint.pose.position.z])
            p =  np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])
            

            # Calculate the direction vector of the line (d)
            d = p2 - p1
            
            # Calculate the norm of the direction vector
            d_norm = np.linalg.norm(d)

            if not self.path_orientation_style == 2:
            
                # Calculate the normalized direction vector
                if not d_norm == 0:
                    d_normalized = d / d_norm
                    # Calculate the vector from A to P (v)
                    v = p - p1
                    
                    # Calculate the projection of v onto d
                    d_dot_d = np.dot(d, d) # Dot product of d with itself
                    v_dot_d = np.dot(v, d) # Dot product of v with d
                    proj_v_onto_d = (v_dot_d / d_dot_d) * d
                    closest_position = p1 + proj_v_onto_d

                else:
                    # Calculate the closest point C
                    d_normalized = 0
                    closest_position = p1
                    rospy.logerr("got here")
                    
                
                # # Move distance units from C along the direction vector
                # if self.lookahead_distance > d_norm:
                #     self.lookahead_distance = d_norm
                    
                lookahead_position = closest_position + self.lookahead_distance * d_normalized


                # # # setting look ahead position
                # lookahead_pose = PoseStamped()
                # lookahead_pose.header.frame_id = "NED"
                lookahead_pose.pose.position.x = lookahead_position[0]
                lookahead_pose.pose.position.y = lookahead_position[1]
                lookahead_pose.pose.position.z = lookahead_position[2]


                if self.path_orientation_style == 1:            
                    lookahead_pose.pose.orientation = self.prev_waypoint.pose.orientation
                        
                # smooth orientation
                elif self.path_orientation_style == 3: 

                    # if self.current_orientation_style
                    # get start and target yaw
                    roll1, pitch1, yaw1 = euler_from_quaternion([self.prev_waypoint.pose.orientation.x, self.prev_waypoint.pose.orientation.y, self.prev_waypoint.pose.orientation.z, self.prev_waypoint.pose.orientation.w])
                    roll2, pitch2, yaw2 = euler_from_quaternion([self.current_waypoint.pose.orientation.x, self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])
                    # normalize the yaw
                    if yaw1 > np.pi:
                        yaw1 -= 2 * np.pi
                    elif yaw1 < -np.pi:
                        yaw1 += 2 * np.pi

                    if yaw2 > np.pi:
                        yaw2 -= 2 * np.pi
                    elif yaw2 < -np.pi:
                        yaw2 += 2 * np.pi

                    delta_yaw = yaw2 - yaw1
                    if delta_yaw > np.pi:
                        delta_yaw -= 2 * np.pi
                    elif delta_yaw < -np.pi:
                        delta_yaw += 2 * np.pi
                    
                    if not d_norm == 0:
                        distance_ratio = np.linalg.norm(lookahead_position-p1)/d_norm
                    else:
                        #  this is the case where it is the last waypoint and lookahead is equal to the last waypoint
                        distance_ratio = 1

                    # prevent overshooting th eration and going into the negatives
                    if distance_ratio <= 0:
                        distance_ratio =1
                    
                    lookahead_yaw = yaw1 + delta_yaw * distance_ratio

                    if lookahead_yaw > np.pi:
                        lookahead_yaw -= 2 * np.pi
                    elif lookahead_yaw < -np.pi:
                        lookahead_yaw += 2 * np.pi

                    # get quaternion
                    q = quaternion_from_euler(roll1,pitch1,lookahead_yaw)

                    lookahead_pose.pose.orientation.x = q[0]
                    lookahead_pose.pose.orientation.y = q[1]
                    lookahead_pose.pose.orientation.z = q[2]
                    lookahead_pose.pose.orientation.w = q[3]

                    # # determine at what distance along path to make the point  =to the goal point
                    # if d_norm - np.linalg.norm(lookahead_position-p1) < 0:  
                    #     lookahead_pose = self.current_waypoint

                elif self.path_orientation_style == 4:
                    
                    # get start and target yaw
                    roll1, pitch1, yaw1 = euler_from_quaternion([self.prev_waypoint.pose.orientation.x, self.prev_waypoint.pose.orientation.y, self.prev_waypoint.pose.orientation.z, self.prev_waypoint.pose.orientation.w])
                    # roll2, pitch2, yaw2 = euler_from_quaternion([self.current_waypoint.pose.orientation.x, self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])
                    # normalize 
                
                    yaw = np.arctan2(d[1],d[0])

                    if yaw > np.pi:
                        yaw -= 2 * np.pi
                    elif yaw < -np.pi:
                        yaw += 2 * np.pi

                    q = quaternion_from_euler(roll1,pitch1,yaw)

                    lookahead_pose.pose.orientation.x = q[0]
                    lookahead_pose.pose.orientation.y = q[1]
                    lookahead_pose.pose.orientation.z = q[2]
                    lookahead_pose.pose.orientation.w = q[3]

                       
                    # calculate the yaw error to final orientation
                    yaw_error = yaw - self.current_yaw
                    # Normalize yaw error to [-pi, pi] range
                    if yaw_error> np.pi:
                        yaw_error -= 2 * np.pi
                    elif yaw_error < -np.pi:
                        yaw_error += 2 * np.pi


                    # overwrite the look ahead position
                    if yaw_error > self.yaw_threshold:
                        lookahead_pose.pose.position = self.prev_waypoint.pose.position
                        lookahead_position = p1
                    

            
            elif self.path_orientation_style == 2: 
                lookahead_pose.pose.orientation = self.current_waypoint.pose.orientation
                
                # _,_,target_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])
                
                # current_yaw = euler_from_quaternion([self.c.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])


                # calculate the yaw error to final orientation
                yaw_error = self.waypoint_yaw - self.current_yaw
                # Normalize yaw error to [-pi, pi] range
                if yaw_error> np.pi:
                    yaw_error -= 2 * np.pi
                elif yaw_error < -np.pi:
                    yaw_error += 2 * np.pi

                if yaw_error > self.yaw_threshold:
                    lookahead_pose.pose.position = self.prev_waypoint.pose.position
                    lookahead_position = p1
                else:
            
                    # Calculate the normalized direction vector
                    if not d_norm == 0:
                        d_normalized = d / d_norm
                        # Calculate the vector from A to P (v)
                        v = p - p1
                        
                        # Calculate the projection of v onto d
                        d_dot_d = np.dot(d, d) # Dot product of d with itself
                        v_dot_d = np.dot(v, d) # Dot product of v with d
                        proj_v_onto_d = (v_dot_d / d_dot_d) * d
                        closest_position = p1 + proj_v_onto_d

                    else:
                        # Calculate the closest point C
                        d_normalized = 0
                        closest_position = p1
                        rospy.logerr("got here 2")


                    #  this has room for error TO DO

                    # # Move distance units from C along the direction vector
                    # if self.lookahead_distance > d_norm:
                    #     self.lookahead_distance = d_norm
                        
                    lookahead_position = closest_position + self.lookahead_distance * d_normalized


                    # # # setting look ahead position
            
                    lookahead_pose.pose.position.x = lookahead_position[0]
                    lookahead_pose.pose.position.y = lookahead_position[1]
                    lookahead_pose.pose.position.z = lookahead_position[2]


            # # determine at what distance along path to make the point  =to the goal point
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0:  
            #     rospy.loginfo("got to last if statement")
            #     lookahead_pose = self.current_waypoint

            #  tune THIS PARAMETER make it editable 
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0-self.position_threshold: 
            # TO DO
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0-self.lookahead_distance/2:   
            if d_norm - np.linalg.norm(lookahead_position-p1) <= -self.lookahead_past_waypoint:
                # rospy.loginfo("got to last if statement")
                # lookahead_pose = self.current_waypoint
                rospy.loginfo("running lookahead 3")

                lookahead_pose = self.lookahead_method3(self.next_waypoint, self.current_waypoint)
    
        return lookahead_pose

    # same as method 1 just doesnt use th eobject specirfied 
    def lookahead_method2(self, current:PoseStamped, prev:PoseStamped):
        # setting look ahead position
        lookahead_pose = PoseStamped()
        lookahead_pose.header.frame_id = "NED"

        if prev == current:
            rospy.loginfo("prev waypoint = current waypoint check")
            # pass
        # elif current == next:
        #     rospy.loginfo("next waypoint = current waypoint check")
        #     return self.lookahead_method1()

        else:
            p1 = np.array([prev.pose.position.x,prev.pose.position.y, prev.pose.position.z])
            p2  = np.array([current.pose.position.x,current.pose.position.y, current.pose.position.z])
            # p2  = np.array([self.next_waypoint.pose.position.x,self.next_waypoint.pose.position.y, self.next_waypoint.pose.position.z])
            p =  np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])
            

            # Calculate the direction vector of the line (d)
            d = p2 - p1
            
            # Calculate the norm of the direction vector
            d_norm = np.linalg.norm(d)

            if not self.path_orientation_style == 2:
            
                # Calculate the normalized direction vector
                if not d_norm == 0:
                    d_normalized = d / d_norm
                    # Calculate the vector from A to P (v)
                    v = p - p1
                    
                    # Calculate the projection of v onto d
                    d_dot_d = np.dot(d, d) # Dot product of d with itself
                    v_dot_d = np.dot(v, d) # Dot product of v with d
                    proj_v_onto_d = (v_dot_d / d_dot_d) * d
                    closest_position = p1 + proj_v_onto_d

                else:
                    # Calculate the closest point C
                    d_normalized = 0
                    closest_position = p1
                    rospy.logerr("got here")
                    
                
                # # Move distance units from C along the direction vector
                # if self.lookahead_distance > d_norm:
                #     self.lookahead_distance = d_norm
                    
                lookahead_position = closest_position + self.lookahead_distance * d_normalized


                # # # setting look ahead position
                # lookahead_pose = PoseStamped()
                # lookahead_pose.header.frame_id = "NED"
                lookahead_pose.pose.position.x = lookahead_position[0]
                lookahead_pose.pose.position.y = lookahead_position[1]
                lookahead_pose.pose.position.z = lookahead_position[2]


                if self.path_orientation_style == 1:            
                    lookahead_pose.pose.orientation = prev.pose.orientation
                        
                # smooth orientation
                elif self.path_orientation_style == 3: 

                    # if self.current_orientation_style
                    # get start and target yaw
                    roll1, pitch1, yaw1 = euler_from_quaternion([prev.pose.orientation.x, prev.pose.orientation.y, prev.pose.orientation.z, prev.pose.orientation.w])
                    roll2, pitch2, yaw2 = euler_from_quaternion([current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
                    # normalize the yaw
                    if yaw1 > np.pi:
                        yaw1 -= 2 * np.pi
                    elif yaw1 < -np.pi:
                        yaw1 += 2 * np.pi

                    if yaw2 > np.pi:
                        yaw2 -= 2 * np.pi
                    elif yaw2 < -np.pi:
                        yaw2 += 2 * np.pi

                    delta_yaw = yaw2 - yaw1
                    if delta_yaw > np.pi:
                        delta_yaw -= 2 * np.pi
                    elif delta_yaw < -np.pi:
                        delta_yaw += 2 * np.pi
                    
                    if not d_norm == 0:
                        distance_ratio = np.linalg.norm(lookahead_position-p1)/d_norm
                    else:
                        #  this is the case where it is the last waypoint and lookahead is equal to the last waypoint
                        distance_ratio = 1

                    # prevent overshooting th eration and going into the negatives
                    if distance_ratio <= 0:
                        distance_ratio =1
                    
                    lookahead_yaw = yaw1 + delta_yaw * distance_ratio

                    if lookahead_yaw > np.pi:
                        lookahead_yaw -= 2 * np.pi
                    elif lookahead_yaw < -np.pi:
                        lookahead_yaw += 2 * np.pi

                    # get quaternion
                    q = quaternion_from_euler(roll1,pitch1,lookahead_yaw)

                    lookahead_pose.pose.orientation.x = q[0]
                    lookahead_pose.pose.orientation.y = q[1]
                    lookahead_pose.pose.orientation.z = q[2]
                    lookahead_pose.pose.orientation.w = q[3]

                    # # determine at what distance along path to make the point  =to the goal point
                    # if d_norm - np.linalg.norm(lookahead_position-p1) < 0:  
                    #     lookahead_pose = current

                elif self.path_orientation_style == 4:
                    
                    # get start and target yaw
                    roll1, pitch1, yaw1 = euler_from_quaternion([prev.pose.orientation.x, prev.pose.orientation.y, prev.pose.orientation.z, prev.pose.orientation.w])
                    # roll2, pitch2, yaw2 = euler_from_quaternion([current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
                    # normalize 
                
                    yaw = np.arctan2(d[1],d[0])

                    if yaw > np.pi:
                        yaw -= 2 * np.pi
                    elif yaw < -np.pi:
                        yaw += 2 * np.pi

                    q = quaternion_from_euler(roll1,pitch1,yaw)

                    lookahead_pose.pose.orientation.x = q[0]
                    lookahead_pose.pose.orientation.y = q[1]
                    lookahead_pose.pose.orientation.z = q[2]
                    lookahead_pose.pose.orientation.w = q[3]

                       
                    # calculate the yaw error to final orientation
                    yaw_error = yaw - self.current_yaw
                    # Normalize yaw error to [-pi, pi] range
                    if yaw_error> np.pi:
                        yaw_error -= 2 * np.pi
                    elif yaw_error < -np.pi:
                        yaw_error += 2 * np.pi


                    # overwrite the look ahead position
                    if yaw_error > self.yaw_threshold:
                        lookahead_pose.pose.position = prev.pose.position
                        lookahead_position = p1
                    

            
            elif self.path_orientation_style == 2: 
                lookahead_pose.pose.orientation = current.pose.orientation
                
                # _,_,target_yaw = euler_from_quaternion([current.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
                
                # current_yaw = euler_from_quaternion([self.c.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])


                # calculate the yaw error to final orientation
                yaw_error = self.waypoint_yaw - self.current_yaw
                # Normalize yaw error to [-pi, pi] range
                if yaw_error> np.pi:
                    yaw_error -= 2 * np.pi
                elif yaw_error < -np.pi:
                    yaw_error += 2 * np.pi

                if yaw_error > self.yaw_threshold:
                    lookahead_pose.pose.position = prev.pose.position
                    lookahead_position = p1
                else:
            
                    # Calculate the normalized direction vector
                    if not d_norm == 0:
                        d_normalized = d / d_norm
                        # Calculate the vector from A to P (v)
                        v = p - p1
                        
                        # Calculate the projection of v onto d
                        d_dot_d = np.dot(d, d) # Dot product of d with itself
                        v_dot_d = np.dot(v, d) # Dot product of v with d
                        proj_v_onto_d = (v_dot_d / d_dot_d) * d
                        closest_position = p1 + proj_v_onto_d

                    else:
                        # Calculate the closest point C
                        d_normalized = 0
                        closest_position = p1
                        rospy.logerr("got here 2")


                    #  this has room for error TO DO

                    # # Move distance units from C along the direction vector
                    # if self.lookahead_distance > d_norm:
                    #     self.lookahead_distance = d_norm
                        
                    lookahead_position = closest_position + self.lookahead_distance * d_normalized


                    # # # setting look ahead position
            
                    lookahead_pose.pose.position.x = lookahead_position[0]
                    lookahead_pose.pose.position.y = lookahead_position[1]
                    lookahead_pose.pose.position.z = lookahead_position[2]


            # # determine at what distance along path to make the point  =to the goal point
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0:  
            #     rospy.loginfo("got to last if statement")
            #     lookahead_pose = current

            #  tune THIS PARAMETER make it editable 
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0-self.position_threshold: 
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0-self.lookahead_distance/2:   
            if d_norm - np.linalg.norm(lookahead_position-p1) <= -self.lookahead_past_waypoint:
                # rospy.loginfo("got to last if statement")
                lookahead_pose = current

    
        return lookahead_pose

       # same as method 1 just doesnt use th eobject specirfied 
    
    def lookahead_method3(self, current:PoseStamped, prev:PoseStamped):
        # setting look ahead position
        lookahead_pose = PoseStamped()
        lookahead_pose.header.frame_id = "NED"

        if prev == current:
            rospy.loginfo("prev waypoint = current waypoint check")
            # pass
        # elif current == next:
        #     rospy.loginfo("next waypoint = current waypoint check")
        #     return self.lookahead_method1()

        else:
            p1 = np.array([prev.pose.position.x,prev.pose.position.y, prev.pose.position.z])
            p2  = np.array([current.pose.position.x,current.pose.position.y, current.pose.position.z])
            # p2  = np.array([self.next_waypoint.pose.position.x,self.next_waypoint.pose.position.y, self.next_waypoint.pose.position.z])
            p =  np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])
            

            # Calculate the direction vector of the line (d)
            d = p2 - p1
            
            # Calculate the norm of the direction vector
            d_norm = np.linalg.norm(d)

            if not self.path_orientation_style == 2:
            
                # Calculate the normalized direction vector
                if not d_norm == 0:
                    d_normalized = d / d_norm
                    # Calculate the vector from A to P (v)
                    v = p - p1
                    
                    # Calculate the projection of v onto d
                    d_dot_d = np.dot(d, d) # Dot product of d with itself
                    v_dot_d = np.dot(v, d) # Dot product of v with d
                    proj_v_onto_d = (v_dot_d / d_dot_d) * d
                    closest_position = p1 + proj_v_onto_d

                else:
                    # Calculate the closest point C
                    d_normalized = 0
                    closest_position = p1
                    rospy.logerr("got here")
                    
                
                # # Move distance units from C along the direction vector
                # if self.lookahead_distance > d_norm:
                #     self.lookahead_distance = d_norm
                    
                lookahead_position = closest_position + self.lookahead_distance * d_normalized


                # # # setting look ahead position
                # lookahead_pose = PoseStamped()
                # lookahead_pose.header.frame_id = "NED"
                lookahead_pose.pose.position.x = lookahead_position[0]
                lookahead_pose.pose.position.y = lookahead_position[1]
                lookahead_pose.pose.position.z = lookahead_position[2]


                if self.path_orientation_style == 1:            
                    lookahead_pose.pose.orientation = prev.pose.orientation
                        
                # smooth orientation
                elif self.path_orientation_style == 3: 

                    # if self.current_orientation_style
                    # get start and target yaw
                    roll1, pitch1, yaw1 = euler_from_quaternion([prev.pose.orientation.x, prev.pose.orientation.y, prev.pose.orientation.z, prev.pose.orientation.w])
                    roll2, pitch2, yaw2 = euler_from_quaternion([current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
                    # normalize the yaw
                    if yaw1 > np.pi:
                        yaw1 -= 2 * np.pi
                    elif yaw1 < -np.pi:
                        yaw1 += 2 * np.pi

                    if yaw2 > np.pi:
                        yaw2 -= 2 * np.pi
                    elif yaw2 < -np.pi:
                        yaw2 += 2 * np.pi

                    delta_yaw = yaw2 - yaw1
                    if delta_yaw > np.pi:
                        delta_yaw -= 2 * np.pi
                    elif delta_yaw < -np.pi:
                        delta_yaw += 2 * np.pi
                    
                    if not d_norm == 0:
                        distance_ratio = np.linalg.norm(lookahead_position-p1)/d_norm
                    else:
                        #  this is the case where it is the last waypoint and lookahead is equal to the last waypoint
                        distance_ratio = 1

                    # prevent overshooting th eration and going into the negatives
                    if distance_ratio <= 0:
                        distance_ratio =1
                    
                    lookahead_yaw = yaw1 + delta_yaw * distance_ratio

                    if lookahead_yaw > np.pi:
                        lookahead_yaw -= 2 * np.pi
                    elif lookahead_yaw < -np.pi:
                        lookahead_yaw += 2 * np.pi

                    # get quaternion
                    q = quaternion_from_euler(roll1,pitch1,lookahead_yaw)

                    lookahead_pose.pose.orientation.x = q[0]
                    lookahead_pose.pose.orientation.y = q[1]
                    lookahead_pose.pose.orientation.z = q[2]
                    lookahead_pose.pose.orientation.w = q[3]

                    # # determine at what distance along path to make the point  =to the goal point
                    # if d_norm - np.linalg.norm(lookahead_position-p1) < 0:  
                    #     lookahead_pose = current

                elif self.path_orientation_style == 4:
                    
                    # get start and target yaw
                    roll1, pitch1, yaw1 = euler_from_quaternion([prev.pose.orientation.x, prev.pose.orientation.y, prev.pose.orientation.z, prev.pose.orientation.w])
                    # roll2, pitch2, yaw2 = euler_from_quaternion([current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
                    # normalize 
                
                    yaw = np.arctan2(d[1],d[0])

                    if yaw > np.pi:
                        yaw -= 2 * np.pi
                    elif yaw < -np.pi:
                        yaw += 2 * np.pi

                    q = quaternion_from_euler(roll1,pitch1,yaw)

                    lookahead_pose.pose.orientation.x = q[0]
                    lookahead_pose.pose.orientation.y = q[1]
                    lookahead_pose.pose.orientation.z = q[2]
                    lookahead_pose.pose.orientation.w = q[3]

                       
                    # calculate the yaw error to final orientation
                    yaw_error = yaw - self.current_yaw

                    # Normalize yaw error to [-pi, pi] range
                    if yaw_error> np.pi:
                        yaw_error -= 2 * np.pi
                    elif yaw_error < -np.pi:
                        yaw_error += 2 * np.pi


                    # overwrite the look ahead position
                    if yaw_error > self.yaw_threshold:
                        lookahead_pose.pose.position = prev.pose.position
                        lookahead_position = p1
                    

            
            elif self.path_orientation_style == 2: 
                lookahead_pose.pose.orientation = current.pose.orientation
                
                # _,_,target_yaw = euler_from_quaternion([current.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
                
                # current_yaw = euler_from_quaternion([self.c.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])


                # calculate the yaw error to final orientation
                yaw_error = self.waypoint_yaw - self.current_yaw
                # Normalize yaw error to [-pi, pi] range
                if yaw_error> np.pi:
                    yaw_error -= 2 * np.pi
                elif yaw_error < -np.pi:
                    yaw_error += 2 * np.pi

                if yaw_error > self.yaw_threshold:
                    lookahead_pose.pose.position = prev.pose.position
                    lookahead_position = p1
                else:
            
                    # Calculate the normalized direction vector
                    if not d_norm == 0:
                        d_normalized = d / d_norm
                        # Calculate the vector from A to P (v)
                        v = p - p1
                        
                        # Calculate the projection of v onto d
                        d_dot_d = np.dot(d, d) # Dot product of d with itself
                        v_dot_d = np.dot(v, d) # Dot product of v with d
                        proj_v_onto_d = (v_dot_d / d_dot_d) * d
                        closest_position = p1 + proj_v_onto_d

                    else:
                        # Calculate the closest point C
                        d_normalized = 0
                        closest_position = p1
                        rospy.logerr("got here 2")


                    #  this has room for error TO DO

                    # # Move distance units from C along the direction vector
                    # if self.lookahead_distance > d_norm:
                    #     self.lookahead_distance = d_norm
                        
                    lookahead_position = closest_position + self.lookahead_distance * d_normalized


                    # # # setting look ahead position
            
                    lookahead_pose.pose.position.x = lookahead_position[0]
                    lookahead_pose.pose.position.y = lookahead_position[1]
                    lookahead_pose.pose.position.z = lookahead_position[2]


            # # determine at what distance along path to make the point  =to the goal point
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0:  
            #     rospy.loginfo("got to last if statement")
            #     lookahead_pose = current

            #  tune THIS PARAMETER make it editable 
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0-self.position_threshold: 
            # if d_norm - np.linalg.norm(lookahead_position-p1) <= 0-self.lookahead_distance/2:   
            if d_norm - np.linalg.norm(lookahead_position-p1) <= -self.lookahead_past_waypoint:
                # rospy.loginfo("got to last if statement")
                rospy.loginfo("got here 5")
                lookahead_pose = current
                

        return lookahead_pose




        
# working pure pursuit to target waypoint
def main():
    # Initialize the ROS node
    rospy.init_node('waypoint_manager')
    
    # initialize the waypoint manager
    wp = WaypointManager()

    while not rospy.is_shutdown():

        # need to look into adjusting to the target hold pose once i receive it sometimes im overshooting it 
        if wp.state == "waypoint":
            rospy.loginfo_once("started wp mode")
            
            # activate hold pose mode, only send the waypoint once
            if wp.hold_pose and not wp.hold_pose_sent:
                wp.goal_waypoint_pub.publish(wp.hold_pose_waypoint)
                wp.hold_pose_sent = True
                wp.target_waypoint_sent = False # reset the flag for the waypoint follower loop
                rospy.loginfo("Sent hold position")
            elif not wp.hold_pose:
                # rospy.loginfo_once("testing this loop")
                wp.get_current_waypoint()
                
                try:
                    # for publisbhing the first waypoint
                    if not wp.target_waypoint_sent:
                        wp.goal_waypoint_pub.publish(wp.current_waypoint)
                        wp.target_waypoint_sent = True
              

                    
                    if wp.waypoint_reached(wp.current_pose,wp.current_waypoint):
                        
                        if wp.current_waypoint_index +1 < wp.num_waypoints:
                            wp.current_waypoint_index +=1
                            wp.get_current_waypoint()
                           
                            wp.goal_waypoint_pub.publish(wp.current_waypoint)
                            
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


def main2():
    # Initialize the ROS node
    rospy.init_node('waypoint_manager')
    
    # initialize the waypoint manager
    wp = WaypointManager()

    while not rospy.is_shutdown():

        # need to look into adjusting to the target hold pose once i receive it sometimes im overshooting it 
        if wp.state == "waypoint":
            rospy.loginfo_once("started wp mode")
            
            # activate hold pose mode, only send the waypoint once
            if wp.hold_pose and not wp.hold_pose_sent:
                wp.goal_waypoint_pub.publish(wp.hold_pose_waypoint)
                wp.hold_pose_sent = True
                wp.target_waypoint_sent = False # reset the flag for the waypoint follower loop
                rospy.loginfo("Sent hold position")
            elif not wp.hold_pose:
                # rospy.loginfo_once("testing this loop")
                wp.get_current_waypoint()
                
                # try:
                    # for publisbhing the first waypoint
                if not wp.target_waypoint_sent:
                    try:
                        wp.get_current_waypoint()
                        wp.get_prev_waypoint()
                        wp.get_next_waypoint()
                        wp.get_lookahead_waypoint()
                        wp.lookahead_waypoint_pub.publish(wp.lookahead_waypoint)

                        wp.goal_waypoint_pub.publish(wp.current_waypoint)
                    except:
                        rospy.logwarn("Could not get lookahead waypoint, setting the curent goal waypoint as the lookahead")
                        wp.lookahead_waypoint_pub.publish(wp.current_waypoint)
                        wp.goal_waypoint_pub.publish(wp.current_waypoint)
            
                    wp.target_waypoint_sent = True                      
            

                # try:
                if wp.waypoint_reached(wp.current_pose,wp.current_waypoint):
                    # rospy.loginfo("Waypoint Reached, heading to next waypoint")
                    
                    if wp.current_waypoint_index +1 < wp.num_waypoints:
                        wp.current_waypoint_index +=1
                        wp.get_current_waypoint()
                        wp.get_prev_waypoint()
                        wp.get_next_waypoint()
                        wp.get_lookahead_waypoint()
                        wp.lookahead_waypoint_pub.publish(wp.lookahead_waypoint)
                        
                        wp.goal_waypoint_pub.publish(wp.current_waypoint)
                        
                        # rospy.loginfo_once("Sent new target waypoint")
                        # rospy.loginfo(f"The previous waypoint was \n {wp.prev_waypoint}")
                        # rospy.loginfo(f"The next waypoint is \n {wp.next_waypoint}")


                    elif wp.current_waypoint_index +1 == wp.num_waypoints:
                        rospy.logwarn_throttle(5,"Reached last waypoint")

                    else:
                        rospy.loginfo_throttle(5,"Heading to waypoint")
                else:
                    wp.get_current_waypoint()
                    wp.get_prev_waypoint()
                    wp.get_next_waypoint()
                    wp.get_lookahead_waypoint()
                    wp.lookahead_waypoint_pub.publish(wp.lookahead_waypoint)
                    # rospy.loginfo_throttle(5,"Heading to the target waypoint")
                    wp.goal_waypoint_pub.publish(wp.current_waypoint)
                    # rospy.loginfo_throttle(5,"chasing look ahead waypoint")
            # except: 
            #     rospy.logerr("Error with sending waypoint, could not get look ahead, sending current goal waypoint")
            #     # rospy.logwarn("Could not get lookahead waypoint, setting the curent goal waypoint as the lookahead")
            #     wp.lookahead_waypoint_pub.publish(wp.current_waypoint)
            #     wp.goal_waypoint_pub.publish(wp.current_waypoint)
            else:
                rospy.loginfo_throttle(5,"Holding Pose")
   
        wp.rate.sleep()



if __name__ == "__main__":

    # main()
    main2()