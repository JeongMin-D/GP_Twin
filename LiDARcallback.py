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

def laser_scan_callback(msg):
    # LaserScan 메시지 수신 시 호출되는 콜백 함수
    # msg.ranges를 변경하여 원하는 대로 데이터를 수정한다.

    # 예제: ranges 값 모두 2.0으로 변경
    modified_ranges = get_modified_ranges_from_database()
    modified_ranges = ast.literal_eval(modified_ranges)
    # modified_ranges = [1.0] * len(msg.ranges)

    # intensities 값 생성
    modified_intensities = generate_intensities_from_ranges(modified_ranges)


    # 새로운 LaserScan 메시지 생성
    custom_laser = LaserScan(
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

    # # 새로운 LaserScan 메시지 생성
    # custom_laser = LaserScan(
    #     header=msg.header,
    #     angle_min=0,
    #     angle_max=6.26573,
    #     angle_increment=0.0174533,
    #     time_increment=msg.time_increment,
    #     scan_time=msg.scan_time,
    #     range_min=msg.range_min,
    #     range_max=msg.range_max,
    #     ranges=modified_ranges,
    #     intensities=modified_intensities
    # )

    # 새로운 토픽으로 발행
    pub.publish(custom_laser)

if __name__ == '__main__':
    rospy.init_node('laser_scan_modifier')

    # 새로운 토픽 생성 및 발행 설정
    pub = rospy.Publisher('/custom_laser_topic', LaserScan, queue_size=10)

    # 기존 라이다 토픽 구독 설정
    rospy.Subscriber('/scan', LaserScan, laser_scan_callback)

    rospy.spin()
