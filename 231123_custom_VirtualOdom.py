#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import pymysql
import time

class TurtlebotController:
    def __init__(self):
        self.turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.virtual_odom_x = 0.0
        self.virtual_odom_y = 0.0
        self.virtual_odom_theta = 0.0

        self.virtual_odom_increment = 0.01  # Increment value for virtual_odom_x

        self.max_speed = 0.1

        self.current_pose = None  # Initialize current_pose attribute

        # # 데이터베이스 연결 설정
        # self.db = pymysql.connect(host='your_host', user='your_username', password='your_password', database='your_db')
        # self.cursor = self.db.cursor()

    def odom_callback(self, data):
        self.current_pose = data.pose.pose  # Update current pose

    def get_virtual_odom_from_db(self):
        # y와 theta 값을 0으로 고정, x 값을 0에서 1까지 랜덤하게 상승시키기
        self.virtual_odom_y = 0.0
        self.virtual_odom_theta = 0.0
        # Increment virtual_odom_x until it reaches 1
        if self.virtual_odom_x < 1:
            self.virtual_odom_x += self.virtual_odom_increment
            if self.virtual_odom_x > 1:  # Make sure it doesn't exceed 1
                self.virtual_odom_x = 1

        # 임의의 시간 지연 (0.1초)
        time.sleep(0.1)

    def move_turtlebot(self):
        rate = rospy.Rate(10)  # Publish 주기 설정 (10Hz)

        while not rospy.is_shutdown():
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            current_orientation = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            _, _, current_yaw = euler_from_quaternion(current_orientation)

            self.get_virtual_odom_from_db()

            dx = self.virtual_odom_x - current_x
            dy = self.virtual_odom_y - current_y
            dtheta = self.virtual_odom_theta - current_yaw

            rospy.loginfo("distance: ({}, {}, {})".format(dx, dy, dtheta))

            cmd_vel = Twist()
            cmd_vel.linear.x = min(dx, self.max_speed)
            cmd_vel.linear.y = min(dy, self.max_speed)
            cmd_vel.angular.z = min(dtheta, self.max_speed)

            self.turtlebot_pub.publish(cmd_vel)
            rate.sleep()

    # def __del__(self):
    #     # 데이터베이스 연결 닫기
    #     self.db.close()

def main():
    rospy.init_node('robot_control_node', anonymous=True)
    controller = TurtlebotController()
    controller.move_turtlebot()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
