#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import pymysql

class PoseSender:

    def __init__(self):
        rospy.init_node('pose_sender', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.db = pymysql.connect(host='192.168.1.155',
                                  user='turtlebot',
                                  password='0000',
                                  database='test')
        self.cursor = self.db.cursor()

    def odom_callback(self, odom_msg):  # 추가된 부분
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        theta = odom_msg.pose.pose.orientation.z  # 예시로 z축 값을 사용. 필요에 따라 수정할 것
        self.update_odometry_to_database(x, y, theta)

    def update_odometry_to_database(self, x, y, theta):
        self.cursor.execute("UPDATE one SET x=%s, y=%s, theta=%s where idone = 1", (x, y, theta))
        self.db.commit()

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
