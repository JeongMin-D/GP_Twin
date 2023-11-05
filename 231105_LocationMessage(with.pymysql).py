#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import pymysql

class PoseSender:

    def __init__(self):
        rospy.init_node('pose_sender', anonymous=True)
        rospy.Subscriber('/odometry_topic', Odometry, self.odometry_callback)
        self.db = pymysql.connect(host='192.168.1.155',
                                  user='turtlebot',
                                  password='0000',
                                  database='test')
        self.cursor = self.db.cursor()

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z  # Assuming theta is in the orientation z field
        self.cursor.execute("UPDATE odometry_table SET x=%s, y=%s, theta=%s", (x, y, theta))
        self.db.commit()

        # Optionally, you can print a debug message to confirm the update
        rospy.loginfo(f"Updated odometry data: x={x}, y={y}, theta={theta}")

    def get_pose(self, pose_type):
        x = float(input(f"Enter {pose_type} x coordinate: "))
        y = float(input(f"Enter {pose_type} y coordinate: "))
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

        for _ in range(10):  # Publish for a short duration
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

        for _ in range(10):  # Publish for a short duration
            pub.publish(goal_pose_msg)
            rate.sleep()

    def move_to_new_goal(self):
        while not rospy.is_shutdown():
            new_x, new_y = self.get_pose("new pose")
            self.publish_goal_pose(new_x, new_y)

if __name__ == '__main__':
    pose_sender = PoseSender()

    initial_x, initial_y = pose_sender.get_pose("initial pose")
    pose_sender.publish_initial_pose(initial_x, initial_y)

    # Wait for a short duration to confirm the initial pose
    rospy.sleep(2)  # Adjust the duration as needed

    # Move to new goals until program is terminated
    pose_sender.move_to_new_goal()

rospy.spin()
