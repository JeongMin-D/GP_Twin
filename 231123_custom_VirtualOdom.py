#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import pymysql

class TurtlebotController:
    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=False)

        self.turtlebot_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.virtual_odom_x = 0.0
        self.virtual_odom_y = 0.0
        self.virtual_odom_theta = 0.0

        self.virtual_odom_increment = 0.01  # Increment value for virtual_odom_x

        self.max_linear_velocity = 0.1
        self.max_angular_velocity = 1.0

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

    def move_turtlebot(self):
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

            distance_to_target = math.sqrt((self.virtual_odom_x - current_x) ** 2 + (self.virtual_odom_y - current_y) ** 2)

            target_angle = math.atan2(self.virtual_odom_y - current_y, self.virtual_odom_x - current_x)

            angle_difference = target_angle - current_yaw
            angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

            linear_velocity = self.max_linear_velocity * distance_to_target

            if abs(angle_difference) > math.pi / 2:
                linear_velocity = -self.max_linear_velocity
                angular_velocity = 0.0
            else:
                angular_velocity = self.max_angular_velocity * angle_difference

            cmd_vel = Twist()
            cmd_vel.linear.x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_velocity))
            cmd_vel.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))

            self.turtlebot_cmd_vel.publish(cmd_vel)

    # def __del__(self):
    #     # 데이터베이스 연결 닫기
    #     self.db.close()

if __name__ == '__main__':
    try:
        controller = TurtlebotController()
        controller.move_turtlebot()
    except rospy.ROSInterruptException:
        pass
