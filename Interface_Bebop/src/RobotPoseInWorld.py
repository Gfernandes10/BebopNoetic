#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import rotation_matrix, euler_from_matrix, quaternion_from_matrix, translation_matrix, quaternion_matrix, quaternion_from_euler, euler_from_quaternion
import math


class RobotPoseToWorld:
    def __init__(self):
        rospy.init_node('Bebop_pose_word', anonymous=False)

        # Subscribers
        rospy.Subscriber('/aruco_single/pose', PoseStamped, self.robot_pose_rel_tag_callback)

        # Publisher
        self.robot_pose_world_pub = rospy.Publisher('/robot_pose_world', PoseStamped, queue_size=10)

        # Pose tag word
        TagPx = 0.4
        TagPy = 0.0
        TagPz = 1.5
        TagRoll = 0.0
        TagPitch = 1.57
        TagYaw = 0.0
        Tagquaternion = quaternion_from_euler(TagRoll, TagPitch, TagYaw,'rxyz')
        
        TagOx = Tagquaternion[0]
        TagOy = Tagquaternion[1]
        TagOz = Tagquaternion[2]
        TagOw = Tagquaternion[3]
        translation = translation_matrix([TagPx,TagPy,TagPz])
        
        rotation = quaternion_matrix([TagOx, TagOy, TagOz, TagOw])
        
        self.T_tag_world = np.dot(translation, rotation)

        # Transformation matrices
        PoseTag = transform_matrix_to_pose(self.T_tag_world)
        
        
        
        # print(PoseTag)
        self.T_robot_tag = None


    def robot_pose_rel_tag_callback(self, robot_pose_rel_tag):
        # Transform robot pose to transformation matrix
        self.T_robot_tag = pose_to_transform_matrix(robot_pose_rel_tag.pose)
        
        # If both robot-to-tag and tag-to-world transformations are available
        if self.T_robot_tag is not None:
            # Compose transformations
            inv_T_robot_tag = np.linalg.inv(self.T_robot_tag)
            T_robot_world = np.dot(self.T_tag_world, inv_T_robot_tag)
            # T_robot_world = np.dot(self.T_tag_world,self.T_robot_tag)
            
            # print(T_robot_world)
            # Convert the composed transformation back to Pose
            robot_pose_world = transform_matrix_to_pose(T_robot_world)
            
            # Publish the robot pose in the world frame
            self.robot_pose_world_pub.publish(robot_pose_world)

def pose_to_transform_matrix(pose):
    translation = translation_matrix([pose.position.x, pose.position.y, pose.position.z])
    rotation = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return np.dot(translation, rotation)

def transform_matrix_to_pose(transform_matrix):
    pose = PoseStamped()
    # print(transform_matrix)
    translation = transform_matrix[0:3, 3]
    
    pose.pose.position.x = translation[0]
    pose.pose.position.y = translation[1]
    pose.pose.position.z = translation[2]   
    # transform_matrix[0:3, 3] = [0, 0, 0]
    
    # rotation_matrix = 
    rotation_matrix = transform_matrix[0:4, 0:4]
    
    # print(rotation_matrix)
    rotation = quaternion_from_matrix(rotation_matrix)

    pose.header.stamp = rospy.Time.now()

    pose.pose.orientation.x = rotation[0]
    pose.pose.orientation.y = rotation[1]
    pose.pose.orientation.z = rotation[2]
    pose.pose.orientation.w = rotation[3]
    # pose.pose.orientation.x = rotation[2]
    # pose.pose.orientation.y = rotation[3]
    # pose.pose.orientation.z = rotation[0]
    # pose.pose.orientation.w = rotation[1]

    print(pose)
    return pose

if __name__ == '__main__':
    try:
        robot_pose_to_world_node = RobotPoseToWorld()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
