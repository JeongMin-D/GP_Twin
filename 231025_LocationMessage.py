#!/usr/bin/env python

# rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {stamp: now, frame_id: "map"}, pose: {pose: {position: {x: -2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
#
# rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}'
#
# rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: -2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

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
            goal_x, goal_y = self.get_pose()
            self.publish_goal_pose(goal_x, goal_y)

if __name__ == '__main__':
    pose_sender = PoseSender()

    initial_x, initial_y = pose_sender.get_pose()
    pose_sender.publish_initial_pose(initial_x, initial_y)

    # Wait for a short duration to confirm the initial pose
    rospy.sleep(2)  # Adjust the duration as needed

    # Move to new goals until program is terminated
    pose_sender.move_to_new_goal()
