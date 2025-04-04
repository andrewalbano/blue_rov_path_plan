#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion, euler_matrix,quaternion_from_euler
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray, String, Int8MultiArray
from nav_msgs.msg import Path
import time


class IntermediateWaypointGeneration:
    def __init__(self):
        
        # publishers
        # self.waypoint_pub = rospy.Publisher('current_waypoint', PoseStamped, queue_size=1)

        self.create_path_flag = False

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
        self.current_pose = PoseStamped()


        self.path_generation_details = []
        
        # Current Pose 
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id ="NED"
        self.current_roll,self.current_pitch,self.current_yaw = None, None, None

        # list of the target waypoints
        self.target_waypoints = PoseArray()
        self.target_waypoints.header.frame_id = "NED"
        self.num_waypoints = None
        self.target_waypoint_sent =False


        self.path_options = {1: "Straight Line",
                             2: "Sine Wave",
                             3: "Arc"
                             }
        
        self.orientation_options = {1: "Static",
                                    2: "Facing Next",
                                    3: "Orbit"
                                    }
        self.orientation_type = 1
        self.path_type = 1


        self.int_waypoint_list = PoseArray()
        self.int_waypoint_list .header.frame_id = "NED"
        self.desired_path = Path()
        self.desired_path.header.frame_id ="NED"
        
        # creating subscribers
        self.controller_state_sub = rospy.Subscriber('motion_controller_state', String, self.controller_state_callback)
        self.pose_sub = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        self.reset_waypoints_sub= rospy.Subscriber('reset_waypoints',Bool, self.reset_waypoints_callback)
        # self.waypoint_list_sub = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)
        self.add_waypoint_sub = rospy.Subscriber('add_waypoint', PoseStamped, self.add_waypoint_callback)
        self.hold_pose_sub = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)
        self.path_generation_details_sub = rospy.Subscriber('path_generation_type', Int8MultiArray, self.path_generation_details_callback)
        
        self.path_pub = rospy.Publisher('generated_path', Path, queue_size=1)
        self.poses_pub = rospy.Publisher('int_poses', PoseArray, queue_size=1)

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

    def path_generation_details_callback(self, msg:Int8MultiArray):
        self.path_generation_detail = []
        self.path_generation_detail = msg.data
        self.path_type = self.path_generation_detail[0]
        self.orientation_type = self.path_generation_detail[1]

        self.create_path_flag = True


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


    def straight_line_path(self,start_pose:PoseStamped, target_pose:PoseStamped, spacing = 0.2):
       
        vector = (target_pose.pose.position.x - start_pose.pose.position.x, target_pose.pose.position.y - start_pose.pose.position.y, target_pose.pose.position.z - start_pose.pose.position.z) 
        # vector = (2,0,0)
        magnitude = np.linalg.norm(vector)    

        unit_vector = vector/magnitude
        # rospy.loginfo(f"unit vector is: {unit_vector[0]}, {unit_vector[1]}, {unit_vector[2]}")

        # get number of points including both endpoints 
        num_points = int(magnitude / spacing) + 2
        # rospy.loginfo(f"number of points: {num_points}")


        # initialize waypoint, each waypoint gets start orientation until the last waypoint
        waypoint = Pose()
        waypoint.orientation = start_pose.pose.orientation

        waypoint_list = PoseArray()
        waypoint_list.header.frame_id = "NED"

        # get each point get pose array
        for i in range(num_points):
            # rospy.loginfo(f"index: {i}")
            waypoint = Pose()
            waypoint.orientation = start_pose.pose.orientation


            vec = spacing*unit_vector*i

            # rospy.loginfo(f"vec :{vec}")
        
        
            if i == num_points-1:
                waypoint.position.x = target_pose.pose.position.x
                waypoint.position.y = target_pose.pose.position.y
                waypoint.position.z = target_pose.pose.position.z
            else:

                waypoint.position.x = start_pose.pose.position.x + vec[0]
                waypoint.position.y = start_pose.pose.position.y + vec[1]
                waypoint.position.z = start_pose.pose.position.z + vec[2]
                waypoint.orientation = start_pose.pose.orientation


            waypoint_list.poses.append(waypoint)
            

            # rospy.loginfo(waypoint)
            # rospy.loginfo(f"last waypoint added = \n {waypoint_list.poses[-1]}")
            # rospy.loginfo(f"waypoint list")
            # for i in range(len(waypoint_list.poses)):

            #     rospy.loginfo(waypoint_list.poses[i])

            if waypoint_list.poses[-1].position == target_pose.pose.position:
                break
    
        # get last waypoint and ensure it ends directly on the tareget waypoint
        if not waypoint_list.poses[-1].position == target_pose.pose.position:
            waypoint.position = target_pose.pose.position
            waypoint_list.poses.append(waypoint)

        # # if orientation change is required ... should consider incrementing the rotation aswell
        # if not waypoint_list.poses[-1].orientation == target_pose.pose.orientation:
        #     waypoint.orientation = target_pose.pose.orientation
        #     waypoint_list.poses.append(waypoint)


        return waypoint_list

    def static_orientation(self,start_pose:PoseStamped, target_pose:PoseStamped, waypoint_list:PoseArray):
        
        if not self.desired_path.poses:    
            self.desired_path.poses.append(start_pose)

    
        
        for waypoint in waypoint_list.poses:
            waypoint.orientation = start_pose.pose.orientation

            if not self.desired_path.poses:    
                self.desired_path.poses.append(start_pose)
                self.int_waypoint_list.poses.append(start_pose.pose)

            else:
                next_pose = PoseStamped()
                next_pose.header.frame_id = "NED"
                next_pose.pose = waypoint
                self.desired_path.poses.append(next_pose)

                self.int_waypoint_list.poses.append(next_pose.pose)



        if not waypoint_list.poses[-1] == target_pose.pose:
        
            waypoint_list.poses.append(target_pose.pose)
            self.desired_path.poses.append(target_pose)
            self.int_waypoint_list.poses.append(target_pose.pose)
            
        
        return waypoint_list

    # def arc_path(self,start_pose:PoseStamped, target_pose:PoseStamped, spacing = 0.2):



def generate_square(start_pose:PoseStamped = PoseStamped(), size = 5):
    # test values
    list = PoseArray()
    list.header.frame_id ="NED"


    q = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.w, start_pose.pose.orientation.z]
    initial_roll, initial_pitch, initial_yaw = euler_from_quaternion(q)

    pose2= PoseStamped()
    yaw = initial_yaw + (90*np.pi/180)
    q = quaternion_from_euler(0,0,yaw)
    pose2.header.frame_id ="NED"
    pose2.pose.position.x = start_pose.pose.position.x +size
    pose2.pose.position.y = start_pose.pose.position.y
    pose2.pose.position.z = start_pose.pose.position.z 
    pose2.pose.orientation.x =q[0]
    pose2.pose.orientation.y =q[1]
    pose2.pose.orientation.z =q[2]
    pose2.pose.orientation.w =q[3]  



    pose3= PoseStamped()
    yaw = initial_yaw + (180*np.pi/180)
    q = quaternion_from_euler(0,0,yaw)
    pose3.header.frame_id ="NED"
    pose3.pose.position.x = start_pose.pose.position.x +size
    pose3.pose.position.y = start_pose.pose.position.y +size
    pose3.pose.position.z = start_pose.pose.position.z 
    pose3.pose.orientation.x =q[0]
    pose3.pose.orientation.y =q[1]
    pose3.pose.orientation.z =q[2]
    pose3.pose.orientation.w =q[3]

    pose4= PoseStamped()
    yaw = initial_yaw + (270*np.pi/180)
    q = quaternion_from_euler(0,0,yaw)
    pose4.header.frame_id ="NED"
    pose4.pose.position.x = start_pose.pose.position.x
    pose4.pose.position.y = start_pose.pose.position.y+size
    pose4.pose.position.z = start_pose.pose.position.z 
    pose4.pose.orientation.x =q[0]
    pose4.pose.orientation.y =q[1]
    pose4.pose.orientation.z =q[2]
    pose4.pose.orientation.w =q[3]

    pose5= PoseStamped()
    yaw = initial_yaw + (0*np.pi/180)
    q = quaternion_from_euler(0,0,yaw)
    pose5.header.frame_id ="NED"
    pose5.pose.position.x = start_pose.pose.position.x
    pose5.pose.position.y = start_pose.pose.position.y
    pose5.pose.position.z = start_pose.pose.position.z 
    pose5.pose.orientation.x =q[0]
    pose5.pose.orientation.y =q[1]
    pose5.pose.orientation.z =q[2]
    pose5.pose.orientation.w =q[3]

    


    list.poses.append(start_pose.pose)
    list.poses.append(pose2.pose)
    list.poses.append(pose3.pose)
    list.poses.append(pose4.pose)
    list.poses.append(pose5.pose)

    return list 




def main():
    # Initialize the ROS node
    rospy.init_node('intermediate_waypoints_generator')
    
    # initialize the waypoint manager
    gen = IntermediateWaypointGeneration()
    
    # gen.state = "waypoint"
    # gen.create_path_flag = True
    list  = PoseArray()
    list.header.frame_id ="NED"
    
   

    while not rospy.is_shutdown():
        if gen.state == "waypoint" and gen.create_path_flag:
            rospy.loginfo("Generating path")

            
            gen.target_waypoints.poses.clear()
            gen.int_waypoint_list.poses.clear()


            # square test pattern
            gen.target_waypoints = generate_square(start_pose=gen.current_pose, size = 2)


            
            # for waypoint, in gen.target_waypoints.poses:
            for index, waypoint in enumerate(gen.target_waypoints.poses, start=0):
                
                if not index == 0:
                    start_pose = PoseStamped()
                    start_pose.header.frame_id = "NED"
                    start_pose.pose = gen.target_waypoints.poses[index-1]


                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "NED"
                    target_pose.pose = gen.target_waypoints.poses[index]
                    
                    if gen.path_type == 1:
                        
                        temp_list  = gen.straight_line_path(start_pose, target_pose)
                        if index > 1:
                            del temp_list.poses[0]

                    elif gen.path_type == 2:
                        temp_list =  gen.straight_line_path(start_pose, target_pose)
                        if index > 1:
                            del temp_list.poses[0]
                        
            
                    if gen.orientation_type == 1:
                        list = gen.static_orientation(start_pose, target_pose,temp_list)


            try:
                rospy.loginfo("Generated path")
                rospy.loginfo(list)
                gen.poses_pub.publish(gen.int_waypoint_list)
                
                gen.path_pub.publish(gen.desired_path)

            
                gen.create_path_flag = False

            except:
                rospy.logerr_once("could not build path")
                gen.create_path_flag = False
    
            

            


                


                

            
            


            #  this works 
            
            # # check if i have waypoints lined up 
            # rospy.loginfo("Generating path")

            # if gen.path_type == 1:
            #     list = gen.straight_line_path(start_pose, pose2)
            

            # if gen.orientation_type == 1:
            #     list = gen.static_orientation(start_pose, pose2,list)

            
            # try:
            #     rospy.loginfo("Generated path")
            #     rospy.loginfo(list)
            #     gen.poses_pub.publish(list)
            #     gen.path_pub.publish(gen.desired_path)
            
            #     gen.create_path_flag = False

            # except:
            #     rospy.logerr_once("could not build path")
            #     gen.create_path_flag = False

            


        gen.rate.sleep()
    





if __name__ == "__main__":

    main()