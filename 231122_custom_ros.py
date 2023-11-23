#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import pymysql

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.target_coordinates = []  # List to store target coordinates
        self.current_target_index = 0  # Index of the current target coordinate

        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 1.0

        self.current_pose = None  # Store current pose

        # MySQL 데이터베이스 연결 설정
        self.db_params = {
            'host': '192.168.1.155',
            'user': 'turtlebot',
            'password': '0000',
            'database': 'custom'
        }

    def odom_callback(self, data):
        self.current_pose = data.pose.pose  # Update current pose

    def update_odometry(self, x, y, theta):
        # Connect to the database, execute the update, and close the connection
        with pymysql.connect(**self.db_params) as conn:
            with conn.cursor() as cursor:
                sql = "UPDATE amr SET x = %s, y = %s, theta = %s WHERE idamr = 1"
                cursor.execute(sql, (x, y, theta))
                conn.commit()

    def move_to_target(self, target_coordinates):
        self.target_coordinates = target_coordinates

        for i in range(len(self.target_coordinates)):
            target_x, target_y = self.target_coordinates[i]
            rospy.loginfo("Moving to target: ({}, {})".format(target_x, target_y))

            while not rospy.is_shutdown():
                # Get the current target coordinates
                target_x, target_y = self.target_coordinates[i]

                # Calculate the distance to the current target
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y

                # Calculate the angular velocity to align with the target angle
                current_orientation = (
                    self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w
                )
                _, _, current_yaw = euler_from_quaternion(current_orientation)

                self.update_odometry(current_x, current_y, current_yaw)

                distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

                # Calculate the angle to the target
                target_angle = math.atan2(target_y - current_y, target_x - current_x)

                angle_difference = target_angle - current_yaw

                # Normalize the angle difference to the range [-pi, pi]
                angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

                # Calculate the linear velocity based on the distance to the target
                linear_velocity = self.max_linear_velocity * distance_to_target

                # If the target is behind the TurtleBot, set reverse motion and no rotation
                if abs(angle_difference) > math.pi / 2:
                    linear_velocity = -self.max_linear_velocity
                    angular_velocity = 0.0
                else:
                    angular_velocity = self.max_angular_velocity * angle_difference

                # Limit the linear velocity within the maximum limits
                linear_velocity = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_velocity))

                # Limit the angular velocity within the maximum limits
                angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))

                # Publish the velocities to move towards the target while aligning with the target angle
                twist_cmd = Twist()
                twist_cmd.linear.x = linear_velocity
                twist_cmd.angular.z = angular_velocity
                self.cmd_vel.publish(twist_cmd)

                # If the distance to the target is less than a threshold, stop and break the loop
                if distance_to_target <= 0.05:
                    rospy.loginfo("Target reached: ({}, {})".format(target_x, target_y))
                    twist_cmd = Twist()
                    twist_cmd.angular.z = 0.0
                    twist_cmd.linear.x = 0.0
                    self.cmd_vel.publish(twist_cmd)
                    break

                self.rate.sleep()

            # Stop the TurtleBot before moving to the next target
            twist_cmd = Twist()
            twist_cmd.angular.z = 0.0
            twist_cmd.linear.x = 0.0
            self.cmd_vel.publish(twist_cmd)
            self.rate.sleep()

        rospy.loginfo("All target positions reached!")
        rospy.signal_shutdown("All target positions reached!")  # Shutdown the node

    def shutdown(self):
        rospy.loginfo("Stopping the TurtleBot...")
        self.cmd_vel.publish(Twist())  # Stop the TurtleBot
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        turtlebot = TurtleBot()

        target_coordinates = []
        while True:
            target_input = input("Enter target coordinates (x,y) or 'done' to finish: ")
            if target_input.lower() == 'done':
                break
            else:
                x, y = target_input.split(",")
                target_coordinates.append((float(x), float(y)))

        turtlebot.move_to_target(target_coordinates)
    except rospy.ROSInterruptException:
        pass