#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import pymysql

class Turtlebot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.turtlebot_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.virtual_odom_x = 0.0
        self.virtual_odom_y = 0.0
        self.virtual_odom_theta = 0.0

        self.virtual_odom_increment = 0.01  # Increment value for virtual_odom_x

        self.max_linear_velocity = 0.1
        self.max_angular_velocity = 1.0

        self.current_pose = None  # Initialize current_pose attribute

    def odom_callback(self, data):
        self.current_pose = data.pose.pose  # Update current pose

    def get_virtual_odom_from_db(self):
        try:
            self.db = pymysql.connect(host='192.168.1.155',
                                      user='turtlebot',
                                      password='0000',
                                      database='custom')
            self.cursor = self.db.cursor()
            # 데이터베이스에서 x값 가져오기 (임의의 테이블과 컬럼명을 가정합니다)
            self.cursor.execute("SELECT x,y,theta FROM amr WHERE idamr = 1")
            result = self.cursor.fetchall()

            if result:
                self.virtual_odom_x, self.virtual_odom_y, self.virtual_odom_theta = result[0]  # 가져온 값을 float 형태로 변환하여 저장

        except pymysql.Error as e:
            rospy.logerr(f"Error retrieving data from the database: {e}")

        finally:
            self.db.close()  # 함수 실행이 끝날 때마다 연결 종료

    def move_turtlebot(self):
        while not rospy.is_shutdown():
            self.get_virtual_odom_from_db()
            rospy.loginfo("Virtual odom: ({}, {}, {})".format(self.virtual_odom_x, self.virtual_odom_y, self.virtual_odom_theta))
            # 기다리면서 self.current_pose가 None이 아닌지 확인
            while self.current_pose is None and not rospy.is_shutdown():
                rospy.loginfo("Waiting for valid odometry data...")
                rospy.sleep(1)

            if self.current_pose is not None:
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y
                current_orientation = (
                    self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w
                )
                _, _, current_yaw = euler_from_quaternion(current_orientation)

                distance_to_target = math.sqrt((self.virtual_odom_x - current_x) ** 2 + (self.virtual_odom_y - current_y) ** 2)

                target_angle = math.atan2(self.virtual_odom_y - current_y, self.virtual_odom_x - current_x)

                angle_difference = target_angle - current_yaw
                angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

                if distance_to_target < 0.05:  # Set your desired threshold for stopping (e.g., 5 cm)
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                else:
                    # linear_velocity = self.max_linear_velocity * distance_to_target
                    linear_velocity = self.max_linear_velocity

                    if abs(angle_difference) > math.pi / 2:
                        linear_velocity = -self.max_linear_velocity
                        angular_velocity = 0.0
                    else:
                        angular_velocity = self.max_angular_velocity * angle_difference

                    cmd_vel = Twist()
                    cmd_vel.linear.x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_velocity))
                    cmd_vel.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))

                self.turtlebot_cmd_vel.publish(cmd_vel)
                self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = Turtlebot()
        controller.move_turtlebot()
    except rospy.ROSInterruptException:
        pass
