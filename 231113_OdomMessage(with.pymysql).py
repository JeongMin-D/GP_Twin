#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from math import atan2, degrees
from tf.transformations import euler_from_quaternion
import pymysql

class OdometrySubscriber:
    def __init__(self):
        rospy.init_node('odometry_subscriber', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Connect to the MySQL database
        self.db = pymysql.connect(
            host='192.168.1.155',
            user='turtlebot',
            password='0000',
            database='test'
        )
        self.cursor = self.db.cursor()

    def odometry_callback(self, msg):
        # Extract x, y, and theta from the odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract quaternion and convert it to Euler angles (theta)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.theta = euler_from_quaternion(quaternion)

        # # Update the database
        # self.update_database()

    def update_database(self):
        # Update x, y, and theta values in the existing row with ID 1
        update_query = "UPDATE one SET x=%s, y=%s, theta=%s WHERE idone=1"
        self.cursor.execute(update_query, (self.x, self.y, degrees(self.theta)))
        self.db.commit()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                # Uncomment the following line to update the database in real-time
                self.update_database()
            except Exception as e:
                print(f"Exception during database update: {e}")

            rate.sleep()

    def __del__(self):
        # Close the database connection when the object is deleted
        self.db.close()

if __name__ == '__main__':
    try:
        odometry_subscriber = OdometrySubscriber()
        odometry_subscriber.run()
    except rospy.ROSInterruptException:
        pass
