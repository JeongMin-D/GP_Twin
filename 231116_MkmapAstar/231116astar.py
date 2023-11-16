#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from heapq import heappush, heappop
import math

class AStarPlanner:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.odom = None
        self.map_info = None
        self.map_data = None

        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None

        self.twist = Twist()

    def odom_callback(self, msg):
        self.odom = msg

    def map_callback(self, msg):
        self.map_info = msg.info
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.map_data = msg.data

    def heuristic(self, a, b):
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, current):
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                neighbor = (current[0] + i, current[1] + j)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.map_data[neighbor[1] * self.width + neighbor[0]] == 0:  # Assuming 0 is free space
                        neighbors.append(neighbor)
        return neighbors

    def move_to_goal(self, goal):
        if self.map_info is None or self.odom is None or self.map_data is None:
            return

        start = (int((self.odom.pose.pose.position.x - self.origin.position.x) / self.resolution),
                 int((self.odom.pose.pose.position.y - self.origin.position.y) / self.resolution))
        goal = (int((goal.x - self.origin.position.x) / self.resolution),
                int((goal.y - self.origin.position.y) / self.resolution))

        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            current_cost, current_node = heappop(open_set)

            if current_node == goal:
                break

            for next_node in self.get_neighbors(current_node):
                new_cost = cost_so_far[current_node] + self.heuristic(current_node, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal)
                    heappush(open_set, (priority, next_node))
                    came_from[next_node] = current_node

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        # Move along the planned path with odometry-based control
        for point in path:
            current_odom_x = self.odom.pose.pose.position.x
            current_odom_y = self.odom.pose.pose.position.y

            # Calculate linear and angular velocities based on odometry and goal point
            dist_to_goal = math.sqrt((point[0] * self.resolution - current_odom_x) ** 2 +
                                     (point[1] * self.resolution - current_odom_y) ** 2)
            angle_to_goal = math.atan2(point[1] * self.resolution - current_odom_y,
                                       point[0] * self.resolution - current_odom_x)

            # Adjust linear and angular velocities based on distance and angle to the goal
            linear_speed = min(0.2, dist_to_goal)  # Adjust linear speed as needed
            angular_speed = 0.3 * (angle_to_goal - self.odom.pose.pose.orientation.z)  # Adjust angular speed as needed

            self.twist.linear.x = linear_speed
            self.twist.angular.z = angular_speed
            self.vel_pub.publish(self.twist)
            rospy.sleep(1)  # Adjust duration or use odometry feedback to control movement
            # Here you might check for obstacle proximity using sensor data and update the velocities accordingly

    def get_user_input(self):
        goal_x = float(input("Enter goal x-coordinate: "))
        goal_y = float(input("Enter goal y-coordinate: "))
        goal = PoseStamped()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        return goal

    def main(self):
        rospy.init_node('a_star_planner')
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            user_goal = self.get_user_input()  # Get user input for goal coordinates
            self.move_to_goal(user_goal)
            rate.sleep()

if __name__ == '__main__':
    planner = AStarPlanner()
    planner.main()
