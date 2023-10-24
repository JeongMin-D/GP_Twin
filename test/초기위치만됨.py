#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class PoseSender:

    def __init__(self):
        rospy.init_node('pose_sender', anonymous=True)

    def get_pose(self):
        x = float(input("Enter x coordinate: "))
        y = float(input("Enter y coordinate: "))
        return x, y

    def publish_initial_pose(self, x, y):
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        rate = rospy.Rate(10)

        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0

        while not rospy.is_shutdown():
            pub.publish(initial_pose_msg)
            rate.sleep()

    def publish_goal_pose(self, x, y):
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rate = rospy.Rate(10)

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = rospy.Time.now()
        goal_pose_msg.header.frame_id = "map"
        goal_pose_msg.pose.position.x = x
        goal_pose_msg.pose.position.y = y
        goal_pose_msg.pose.position.z = 0.0
        goal_pose_msg.pose.orientation.w = 1.0

        while not rospy.is_shutdown():
            pub.publish(goal_pose_msg)
            rate.sleep()

if __name__ == '__main__':
    pose_sender = PoseSender()

    initial_x, initial_y = pose_sender.get_pose()
    goal_x, goal_y = pose_sender.get_pose()

    pose_sender.publish_initial_pose(initial_x, initial_y)
    pose_sender.publish_goal_pose(goal_x, goal_y)
