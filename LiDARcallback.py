#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import pymysql
import ast

class LaserScanModifier:
    def __init__(self):
        rospy.init_node('laser_scan_modifier')

        # Set the desired message output rate (Hz)
        self.rate = rospy.Rate(10)  # 10 Hz, adjust as needed

        # Create a new topic and configure publishing
        self.pub = rospy.Publisher('/custom_laser_topic', LaserScan, queue_size=10)

        # Subscribe to the existing '/scan' topic
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)

    def get_modified_ranges_from_database(self):
        # Connect to the MySQL database
        connection = pymysql.connect(
            host='192.168.1.155',
            user='turtlebot',
            password='0000',
            db='test',
            cursorclass=pymysql.cursors.DictCursor
        )

        try:
            with connection.cursor() as cursor:
                # Execute the SQL query to retrieve modified_ranges from the database
                cursor.execute('SELECT vlidar FROM three WHERE idthree = %s', (1,))
                result = cursor.fetchone()
                return result.get('vlidar') if result else None
        finally:
            connection.close()

    def generate_intensities_from_ranges(self, ranges):
        min_intensity = 0.0
        max_intensity = 234.0

        # Set the range limits
        min_range = 0.0
        max_range = 8.0

        # Generate intensities based on ranges
        intensities = [
            (max_intensity - min_intensity) * (max_range - r) / (max_range - min_range) + min_intensity
            for r in ranges
        ]

        return intensities

    def laser_scan_callback(self, msg):
        # LaserScan 메시지 수신 시 호출되는 콜백 함수
        # msg.ranges를 변경하여 원하는 대로 데이터를 수정한다.

        # 예제: ranges 값 모두 2.0으로 변경
        modified_ranges = self.get_modified_ranges_from_database()
        modified_ranges = ast.literal_eval(modified_ranges) if modified_ranges else [1.0] * len(msg.ranges)

        # Generate intensities
        modified_intensities = self.generate_intensities_from_ranges(modified_ranges)

        # Create a new LaserScan message
        custom_laser_topic = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=modified_ranges,
            intensities=modified_intensities
        )

        # Publish the custom laser topic
        self.pub.publish(custom_laser_topic)

    def run(self):
        while not rospy.is_shutdown():
            # Custom logic goes here (if any)

            # Sleep to maintain the desired output rate
            self.rate.sleep()

if __name__ == '__main__':
    laser_scan_modifier = LaserScanModifier()
    laser_scan_modifier.run()
