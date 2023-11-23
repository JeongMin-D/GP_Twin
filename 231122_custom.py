#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import pymysql.cursors
import math


class ExternalRobotController:
    def __init__(self):
        rospy.init_node('external_robot_controller', anonymous=True)

        # Publisher to send velocity commands to Turtlebot 3
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # MySQL database connection parameters
        self.db_host = '192.168.1.155'  # Replace with your actual database host
        self.db_user = 'turtlebot'  # Replace with your actual database username
        self.db_password = '0000'  # Replace with your actual database password
        self.db_name = 'test'  # Replace with your actual database name

        # Connect to the MySQL database
        self.connection = pymysql.connect(host=self.db_host,
                                          user=self.db_user,
                                          password=self.db_password,
                                          db=self.db_name,
                                          cursorclass=pymysql.cursors.DictCursor)

        # Fixed speed for Turtlebot 3
        self.linear_speed = 0.1  # Adjust the speed as needed
        self.angular_speed = 0.5  # Adjust the angular speed as needed

    def get_coordinates_from_db(self):
        try:
            with self.connection.cursor() as cursor:
                # Query the database to get the latest absolute coordinates
                cursor.execute("SELECT x, y, theta FROM one")
                result = cursor.fetchone()

                if result:
                    x = result['x']
                    y = result['y']
                    theta = result['theta']
                    return x, y, theta
                else:
                    return None
        except Exception as e:
            rospy.logerr("Error fetching coordinates from the database: %s", str(e))
            return None

    def calculate_movement(self, current_x, current_y, target_x, target_y):
        # Calculate linear and angular velocities based on the desired absolute coordinates
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        desired_angle = math.atan2(target_y - current_y, target_x - current_x)
        angle_diff = desired_angle - 0  # Assuming the robot always faces the x-axis in the map

        linear_vel = self.linear_speed if distance > 0.05 else 0.0  # Stop when close to the target
        angular_vel = self.angular_speed * angle_diff

        return linear_vel, angular_vel

    def move_turtlebot(self):
        while not rospy.is_shutdown():
            # Get current robot position (replace with actual robot's position retrieval logic if needed)
            current_x, current_y, _ = 0, 0, 0  # Replace these with actual robot's coordinates

            # Get coordinates from the database
            coordinates = self.get_coordinates_from_db()

            if coordinates is not None:
                target_x, target_y, _ = coordinates

                # Calculate linear and angular velocities based on target coordinates
                linear_vel, angular_vel = self.calculate_movement(current_x, current_y, target_x, target_y)

                # Check if Turtlebot has reached the target coordinates
                distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
                if distance_to_target <= 0.1:  # Set a threshold for reaching the target
                    linear_vel = 0.0
                    angular_vel = 0.0
                    rospy.loginfo("Reached target coordinates: ({}, {})".format(target_x, target_y))
                    # Stop publishing commands
                    self.cmd_vel_pub.publish(Twist())  # Publish an empty Twist message to stop Turtlebot

                # Create a Twist message to control the Turtlebot 3
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel

                # Publish the Twist message to control the Turtlebot 3
                self.cmd_vel_pub.publish(cmd_vel)

            # Sleep to control the rate of commands being sent (adjust as needed)
            rospy.sleep(0.1)  # Reduce sleep time for more frequent updates

    def run(self):
        self.move_turtlebot()


if __name__ == '__main__':
    controller = ExternalRobotController()
    controller.run()