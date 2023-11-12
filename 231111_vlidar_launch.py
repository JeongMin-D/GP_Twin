#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import pymysql
import ast

class LaserScanPublisher:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('laser_scan_publisher', anonymous=True)

        # LaserScan 메세지를 발행할 토픽 이름 설정
        self.scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)

        # 루프 주기 설정 (여기서는 1Hz로 설정)
        self.rate = rospy.Rate(10)

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

    def create_laser_scan_message(self,modified_ranges):
        modified_intensities = self.generate_intensities_from_ranges(modified_ranges)

        # LaserScan 메세지 생성
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_scan'

        custom_laser_topic = LaserScan(
            header=header,
            angle_min=0.0,
            angle_max=6.28319,
            angle_increment=0.0174533,
            time_increment=0.0,
            scan_time=0.0,
            range_min=0.0,
            range_max=100.0,
            ranges=modified_ranges,
            intensities=modified_intensities
        )

        return custom_laser_topic

    def publish_laser_scan(self):
        while not rospy.is_shutdown():
            # Get new data from the database
            modified_ranges = self.get_modified_ranges_from_database()
            modified_ranges = ast.literal_eval(modified_ranges)

            if modified_ranges is not None:
                # Check if there are obstacles in the modified_ranges
                if any(value > 0 for value in modified_ranges):
                    # Update LaserScan message with new data from the database
                    self.scan_publisher.publish(self.create_laser_scan_message(modified_ranges))
                else:
                    # If no obstacles are detected, update LaserScan with empty ranges
                    empty_ranges = [0.0] * len(modified_ranges)
                    self.scan_publisher.publish(self.create_laser_scan_message(empty_ranges))

            # 지정된 주기로 루프를 반복
            self.rate.sleep()

if __name__ == '__main__':
    try:
        laser_scan_publisher = LaserScanPublisher()
        laser_scan_publisher.publish_laser_scan()
    except rospy.ROSInterruptException:
        pass
