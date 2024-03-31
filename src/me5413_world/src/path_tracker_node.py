#!/usr/bin/env python3
# Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
# MIT License
# ROS Node for robot to track a given path

import rospy
import math
import numpy as np

import tf2_ros
import tf.transformations

from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry, Path
from dynamic_reconfigure.server import Server

from me5413_world.cfg import path_trackerConfig

from controllers.pid import PID

# Global dynamic parameters
SPEED_TARGET = None
PID_Kp = None
PID_Ki = None
PID_Kd = None
STANLEY_K = None
PARAMS_UPDATED = False


def dynamic_param_callback(config, level):
    global SPEED_TARGET, PID_Kp, PID_Ki, PID_Kd, STANLEY_K, PARAMS_UPDATED
    # Update dynamic parameters
    SPEED_TARGET = config.speed_target
    PID_Kp = config.PID_Kp
    PID_Ki = config.PID_Ki
    PID_Kd = config.PID_Kd
    STANLEY_K = config.stanley_K
    PARAMS_UPDATED = True
    return config

class PathTrackerNode:
    def __init__(self):
        self.server = Server(path_trackerConfig, dynamic_param_callback)
        
        self.sub_robot_odom = rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.robot_odom_callback)
        self.sub_local_path = rospy.Subscriber("/me5413_world/planning/local_path", Path, self.local_path_callback)
        self.pub_cmd_vel = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=1)

        self.robot_frame = "base_link"
        self.world_frame = "world"
        self.odom_world_robot = Odometry()

        dt = 0.1  # Time step in seconds
        max_value = 1.0  # Maximum output value
        min_value = -1.0  # Minimum output value

        self.pid = PID(dt, max_value, min_value, PID_Kp, PID_Ki, PID_Kd)

    def local_path_callback(self, path):
        # Calculate control outputs and publish
        pose_world_goal = path.poses[11].pose
        cmd_vel = self.compute_control_outputs(self.odom_world_robot, pose_world_goal)
        self.pub_cmd_vel.publish(cmd_vel)

    def robot_odom_callback(self, odom):
        self.world_frame = odom.header.frame_id
        self.robot_frame = odom.child_frame_id
        self.odom_world_robot = odom

    def compute_control_outputs(self, odom_robot, pose_goal):
        # Compute control outputs based on current robot odometry and goal pose
        cmd_vel = Twist()

        # Extract orientation quaternions
        q_robot = [odom_robot.pose.pose.orientation.x, odom_robot.pose.pose.orientation.y,
                odom_robot.pose.pose.orientation.z, odom_robot.pose.pose.orientation.w]
        q_goal = [pose_goal.orientation.x, pose_goal.orientation.y,
                pose_goal.orientation.z, pose_goal.orientation.w]

        # Convert quaternions to Euler angles
        euler_robot = tf.transformations.euler_from_quaternion(q_robot)
        euler_goal = tf.transformations.euler_from_quaternion(q_goal)

        # Heading error
        yaw_robot = euler_robot[2]
        yaw_goal = euler_goal[2]
        heading_error = yaw_goal - yaw_robot

        # Normalize the heading error to the range [-pi, pi]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Lateral error calculation
        # Position difference between robot and goal
        dx = pose_goal.position.x - odom_robot.pose.pose.position.x
        dy = pose_goal.position.y - odom_robot.pose.pose.position.y
        # Calculate error perpendicular to goal's orientation
        lateral_error = np.cos(yaw_goal) * dy - np.sin(yaw_goal) * dx

        # Velocity
        velocity = np.sqrt(odom_robot.twist.twist.linear.x ** 2 + odom_robot.twist.twist.linear.y ** 2)

        # PID control for linear velocity (just an example, adjust as necessary)
        linear_velocity = self.pid.calculate(SPEED_TARGET, velocity)  # Assuming SPEED_TARGET and self.pid are defined

        # Stanley control for angular velocity
        stanley_term = np.arctan2(STANLEY_K * lateral_error, max(velocity, 0.3))
        angular_velocity = -(heading_error + stanley_term)
        angular_velocity = max(-2.2, min(2.2, angular_velocity))  # Assuming max/min angular velocity limits

        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity

        return cmd_vel

if __name__ == '__main__':
    rospy.init_node('path_tracker_node')
    node = PathTrackerNode()
    rospy.spin()

