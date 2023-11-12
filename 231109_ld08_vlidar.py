#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import pymysql
import ast

def get_modified_ranges_from_database():
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
            if result:
                modified_ranges = result['vlidar']
                return modified_ranges
            else:
                return None
    finally:
        connection.close()

def generate_intensities_from_ranges(ranges):
    min_intensity = 0.0
    max_intensity = 234.0

    # ranges의 최소값과 최대값을 구합니다.
    min_range = 0.0
    max_range = 8.0

    # ranges에 비례하여 intensities를 생성합니다.
    intensities = [(max_intensity - min_intensity) * (max_range - r) / (max_range - min_range) + min_intensity for r in ranges]

    return intensities

def scan_publisher():
    rospy.init_node('scan_publisher', anonymous=True)
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():

        modified_ranges = get_modified_ranges_from_database()
        modified_ranges = ast.literal_eval(modified_ranges)

        modified_intensities = generate_intensities_from_ranges(modified_ranges)

        scan_msg = LaserScan()
        # 여기서 필요한 /scan 메시지의 정보를 설정합니다.
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = 'base_scan' # 적절한 frame_id로 변경해야 할 수 있습니다.
        scan_msg.angle_min = 0.006205635145306587
        scan_msg.angle_max = 6.264987945556641
        scan_msg.angle_increment = 0.0256507471203804
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.0
        scan_msg.range_min = 0.0
        scan_msg.range_max = 100.0
        scan_msg.ranges = modified_ranges
        scan_msg.intensities = modified_intensities

        pub.publish(scan_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        scan_publisher()
    except rospy.ROSInterruptException:
        pass
