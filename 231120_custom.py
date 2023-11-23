#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Import Odometry message
from tf.transformations import euler_from_quaternion
import math
import pymysql

class TurtlebotController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('actual_turtlebot_controller')

        # ROS Publisher to control the actual Turtlebot
        self.turtlebot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ROS Subscriber to access odometry data
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Initialize variables for odometry-based x, y, and theta
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

    def odometry_callback(self, odom):
        # Extract x, y, and orientation (theta) from odometry data
        self.odom_x = odom.pose.pose.position.x
        self.odom_y = odom.pose.pose.position.y

        # Convert quaternion to yaw angle (theta)
        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.odom_theta = euler_from_quaternion(orientation_list)

    def control_turtlebot(self):
        # Log odometry-based x, y, and theta values to console
        rospy.loginfo(f"Odometry-based Coordinates - x: {self.odom_x}, y: {self.odom_y}, theta: {self.odom_theta}")

        # Retrieve data from MySQL
        connection = pymysql.connect(host='192.168.1.155', user='turtlebot', password='0000', database='test')
        cursor = connection.cursor()
        sql_query = "SELECT x,y,theta FROM one"
        cursor.execute(sql_query)
        result = cursor.fetchone()
        connection.close()

        if result:
            # Extract x, y, z coordinates
            x, y, z = result[0], result[1], result[2]

            # Calculate distance to target
            distance_to_target = math.sqrt((self.odom_x - x) ** 2 + (self.odom_y - y) ** 2)

            # Define a threshold distance to stop the robot
            threshold_distance = 0.01  # Adjust as needed

            if distance_to_target > threshold_distance:
                # Interpret coordinates and create movement commands
                # Example: Adjust the linear and angular velocities based on the received coordinates
                linear_velocity = 0.1  # Adjust scaling factor as needed
                angular_velocity = z * 0.3  # Adjust scaling factor as needed
            else:
                # If the robot is close to the target, stop
                linear_velocity = 0.0
                angular_velocity = 0.0

            # Create Twist message with linear and angular velocities
            twist_cmd = Twist()
            twist_cmd.linear.x = linear_velocity
            twist_cmd.angular.z = angular_velocity

            # Publish Twist message to control Turtlebot's movement
            self.turtlebot_pub.publish(twist_cmd)

    def run(self):
        # ROS Rate
        rate = rospy.Rate(10)  # 10 Hz

        # Control loop
        while not rospy.is_shutdown():
            self.control_turtlebot()
            rate.sleep()

if __name__ == "__main__":
    try:
        # Create an instance of the TurtlebotController class
        controller = TurtlebotController()

        # Run the control loop
        controller.run()

    except rospy.ROSInterruptException:
        pass