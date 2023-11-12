#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random
from tf.transformations import euler_from_quaternion

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.target_coordinates = []  # List to store target coordinates
        self.current_target_index = 0  # Index of the current target coordinate

        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 1.0
        self.max_linear_acceleration = 0.1
        self.max_angular_acceleration = 1.0

        self.scan_ranges = None
        self.scan_angle_min = None
        self.scan_angle_increment = None

        self.current_pose = None  # Store current pose

        self.current_linear_velocity = 0.0  # Add this line
        self.current_angular_velocity = 0.0  # Add this line

    def odom_callback(self, data):

        self.current_pose = data.pose.pose  # Update current pose
        self.current_linear_velocity = data.twist.twist.linear.x  # Update current linear velocity
        self.current_angular_velocity = data.twist.twist.angular.z  # Update current angular velocity

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

    def scan_callback(self, data):
        self.scan_ranges = data.ranges
        self.scan_angle_min = data.angle_min
        self.scan_angle_increment = data.angle_increment

    def move_to_target(self, target_coordinates):
        self.target_coordinates = target_coordinates

        for i in range(len(self.target_coordinates)):
            target_x, target_y = self.target_coordinates[i]
            rospy.loginfo("Moving to target: ({}, {})".format(target_x, target_y))

            while not rospy.is_shutdown():
                # Get the current target coordinates
                target_x, target_y = self.target_coordinates[i]

                # Calculate the best linear and angular velocities using DWA
                best_linear_velocity, best_angular_velocity = self.calculate_dwa()

                # Publish the velocities to move towards the target
                twist_cmd = Twist()
                twist_cmd.linear.x = best_linear_velocity
                twist_cmd.angular.z = best_angular_velocity
                self.cmd_vel.publish(twist_cmd)

                # Calculate the distance to the current target
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y
                distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

                # If the distance to the target is less than a threshold, stop and break the loop
                if distance_to_target <= 0.1:
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

    def calculate_dwa(self):
        if self.scan_ranges is None or self.scan_angle_min is None or self.scan_angle_increment is None:
            return 0.0, 0.0

        # DWA algorithm implementation
        min_cost = float('inf')
        best_linear_velocity = 0.0
        best_angular_velocity = 0.0

        for linear_velocity in self.generate_linear_velocities():
            for angular_velocity in self.generate_angular_velocities():
                # Simulate the robot's trajectory to calculate the cost of the trajectory
                simulated_trajectory_cost = self.simulate_trajectory(linear_velocity, angular_velocity)

                # Consider obstacle avoidance cost
                collision_risk = self.calculate_collision_risk(
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.calculate_target_yaw()
                )

                # Consider linear and angular acceleration costs
                acceleration_cost = self.calculate_acceleration_cost(linear_velocity, angular_velocity)

                # Combine simulated trajectory cost, collision risk, and acceleration cost for total cost
                total_cost = simulated_trajectory_cost + collision_risk + acceleration_cost

                # Choose the trajectory with the lowest cost
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_linear_velocity = linear_velocity
                    best_angular_velocity = angular_velocity

        return best_linear_velocity, best_angular_velocity

    def calculate_acceleration_cost(self, linear_velocity, angular_velocity):
        # Calculate cost based on linear and angular accelerations

        # Scale the accelerations to be within [0, 1] for cost calculation
        linear_acceleration = abs(linear_velocity - self.current_linear_velocity) / self.max_linear_acceleration
        angular_acceleration = abs(angular_velocity - self.current_angular_velocity) / self.max_angular_acceleration

        # Return the sum of squared accelerations as cost
        return linear_acceleration ** 2 + angular_acceleration ** 2

    def calculate_target_yaw(self):
        target_x, target_y = self.target_coordinates[self.current_target_index]
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        return math.atan2(target_y - current_y, target_x - current_x)

    def simulate_trajectory(self, linear_velocity, angular_velocity):
        # Simulate the robot's trajectory using the provided linear and angular velocities
        # and calculate the cost of the trajectory

        # Define some simulation parameters
        num_steps = 50
        dt = 0.1
        trajectory_cost = 0.0

        # Copy the current pose for simulation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_orientation = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        _, _, current_yaw = euler_from_quaternion(current_orientation)

        # Simulate the trajectory and accumulate cost
        for step in range(num_steps):
            # Simulate the robot's motion using the given velocities
            linear_distance = linear_velocity * dt
            angular_distance = angular_velocity * dt

            # Update the robot's position and orientation
            new_x = current_x + linear_distance * math.cos(current_yaw)
            new_y = current_y + linear_distance * math.sin(current_yaw)
            new_yaw = current_yaw + angular_distance

            # Calculate the cost based on the distance to the target and collision risk
            distance_to_target = math.sqrt((self.target_coordinates[self.current_target_index][0] - new_x) ** 2 +
                                           (self.target_coordinates[self.current_target_index][1] - new_y) ** 2)
            collision_risk = self.calculate_collision_risk(new_x, new_y, new_yaw)

            # Update the trajectory cost with a combination of distance to target and collision risk
            trajectory_cost += distance_to_target + collision_risk

            # Update current position and orientation for next iteration
            current_x = new_x
            current_y = new_y
            current_yaw = new_yaw

        return trajectory_cost

    def calculate_collision_risk(self, x, y, yaw):
        # Calculate the collision risk for the given pose (x, y, yaw) using laser scan data

        # Define some collision risk parameters
        collision_threshold = 0.3
        half_scan_range = len(self.scan_ranges) // 2

        # Get the index of the scan range closest to the robot's forward direction
        robot_heading_index = half_scan_range

        # Calculate the collision risk by checking scan ranges in the robot's heading direction
        collision_risk = 0.0
        for i in range(half_scan_range - 45, half_scan_range + 45):
            scan_range = self.scan_ranges[i]

            # Convert the scan index to the corresponding angle
            angle = self.scan_angle_min + self.scan_angle_increment * i

            # Calculate the position of the obstacle in the scan range
            obstacle_x = x + scan_range * math.cos(yaw + angle)
            obstacle_y = y + scan_range * math.sin(yaw + angle)

            # Calculate the distance to the obstacle
            distance_to_obstacle = math.sqrt((obstacle_x - x) ** 2 + (obstacle_y - y) ** 2)

            # Increase collision risk if obstacle is close
            if distance_to_obstacle < collision_threshold:
                collision_risk += 1.0

        return collision_risk

    def generate_linear_velocities(self):
        # Generate a range of linear velocities
        num_steps = 5
        step_size = self.max_linear_velocity / num_steps
        velocities = [i * step_size for i in range(num_steps + 1)]
        return velocities

    def generate_angular_velocities(self):
        # Generate a range of angular velocities
        num_steps = 10
        step_size = self.max_angular_velocity / num_steps
        velocities = [-self.max_angular_velocity + i * step_size for i in range(num_steps + 1)]
        return velocities

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