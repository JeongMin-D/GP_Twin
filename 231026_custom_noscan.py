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
        self.max_linear_acceleration = 0.1
        self.max_angular_acceleration = 1.0

        self.current_pose = None  # Store current pose

        # Connect to the database
        self.conn = pymysql.connect(host='192.168.1.155', user='turtlebot', password='0000', database='test')
        self.cursor = self.conn.cursor()

    def insert_coordinates(self, x, y, theta):
        try:
            # Connect to the database
            conn = pymysql.connect(host='192.168.1.155', user='turtlebot', password='0000', database='test')
            cursor = conn.cursor()

            query = "UPDATE one SET x = %s, y = %s, theta = %s WHERE idone = 1"
            self.cursor.execute(query, (x, y, theta))
            self.conn.commit()

        except pymysql.Error as e:
            print(f"Error in update_position: {e}")

    def check_stop(self):
        try:
            # Retrieve the collision value from the three table
            query = "SELECT stop FROM three order by idthree desc limit 1"
            self.cursor.execute(query)
            result = self.cursor.fetchone()

            if result is None:
                return 0
            else:
                return result[0]

        except pymysql.Error as e:
            print(f"Error in check_collision: {e}")
            return 0

    def odom_callback(self, data):
        self.current_pose = data.pose.pose  # Update current pose

        # Check if there are any target coordinates
        if len(self.target_coordinates) == 0:
            return

        # Calculate the Euclidean distance between the current position and the current target position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        distance_to_target = math.sqrt((self.target_coordinates[self.current_target_index][0] - current_x) ** 2 +
                                       (self.target_coordinates[self.current_target_index][1] - current_y) ** 2)

        # Check if the TurtleBot has reached the current target position
        if distance_to_target <= 0.1:
            rospy.loginfo("Target reached: {}".format(self.target_coordinates[self.current_target_index]))
            self.current_target_index += 1  # Move to the next target coordinate

            if self.current_target_index >= len(self.target_coordinates):
                self.cmd_vel.publish(Twist())  # Stop the TurtleBot
                rospy.loginfo("All target positions reached!")
                rospy.signal_shutdown("All target positions reached!")  # Shutdown the node

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
                distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

                # Calculate the angle to the target
                target_angle = math.atan2(target_y - current_y, target_x - current_x)

                # Calculate the angular velocity to align with the target angle
                current_orientation = (
                    self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w
                )
                _, _, current_yaw = euler_from_quaternion(current_orientation)
                angle_difference = target_angle - current_yaw

                # Insert current coordinates into the database
                self.insert_coordinates(current_x, current_y, current_yaw)

                # Normalize the angle difference to the range [-pi, pi]
                angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

                # Calculate the linear velocity based on the distance to the target
                linear_velocity = self.max_linear_velocity * distance_to_target

                # If the target is behind the TurtleBot, set reverse motion
                if abs(angle_difference) > math.pi / 2:
                    linear_velocity = -linear_velocity

                # Limit the linear velocity within the maximum limits
                linear_velocity = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_velocity))

                # Calculate the angular velocity to align with the target angle
                angular_velocity = self.max_angular_velocity * angle_difference

                # Limit the angular velocity within the maximum limits
                angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))

                # Publish the velocities to move towards the target while aligning with the target angle
                twist_cmd = Twist()
                twist_cmd.linear.x = linear_velocity
                twist_cmd.angular.z = angular_velocity
                self.cmd_vel.publish(twist_cmd)

                # If the distance to the target is less than a threshold, stop and break the loop
                if distance_to_target <= 0.1:
                    rospy.loginfo("Target reached: ({}, {})".format(target_x, target_y))
                    twist_cmd = Twist()
                    twist_cmd.angular.z = 0.0
                    twist_cmd.linear.x = 0.0
                    self.cmd_vel.publish(twist_cmd)
                    break

                # Check for stop signal from the database
                if self.check_stop() == 1:
                    twist_cmd = Twist()
                    twist_cmd.angular.z = 0.0
                    twist_cmd.linear.x = 0.0
                    self.cmd_vel.publish(twist_cmd)
                    rospy.loginfo("Stopped by external signal.")
                    return

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
