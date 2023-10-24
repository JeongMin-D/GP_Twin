#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class PoseSender:

    def __init__(self):
        rospy.init_node('pose_sender', anonymous=True)
        self.initial_pose_published = False

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

        pub.publish(initial_pose_msg)
        self.initial_pose_published = True
        rospy.loginfo("Initial pose published: x={}, y={}".format(x, y))

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
            rospy.loginfo("Goal pose published: x={}, y={}".format(x, y))
            rate.sleep()

if __name__ == '__main__':
    pose_sender = PoseSender()

    initial_x, initial_y = pose_sender.get_pose()
    goal_x, goal_y = None, None

    pose_sender.publish_initial_pose(initial_x, initial_y)
    while not pose_sender.initial_pose_published:
        pass

    while goal_x is None or goal_y is None:
        goal_x, goal_y = pose_sender.get_pose()

    pose_sender.publish_goal_pose(goal_x, goal_y)
