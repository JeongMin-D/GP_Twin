#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import euler_from_quaternion
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool


class TurtlebotNavigation:

    def __init__(self):
        rospy.init_node('custom_turtlebot3_navigation', anonymous=True)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_service = rospy.ServiceProxy('/static_map', GetMap)
        self.move_completed = rospy.Publisher('/move_completed', Bool, queue_size=10)

        self.goals = []  # 목표 좌표를 저장할 리스트 추가

    def goal_callback(self, msg):
        self.goal_publisher.publish(msg)

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        rospy.loginfo(f"Position (x, y, theta): ({position_x:.2f}, {position_y:.2f}, {yaw:.2f})")

    def load_map(self):
        try:
            map_data = self.map_service().map
            rospy.loginfo("Map data loaded successfully.")
            return map_data
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to load map data: {e}")

    def run(self):
        rospy.loginfo("Press 'Ctrl + C' to exit.")
        map_data = self.load_map()

        try:
            while True:
                x = input("목표 좌표를 입력하세요 (띄어쓰기로 구분): ")
                if x.lower() == 'done':
                    break
                x, y = map(float, x.split())

                # 목표 좌표를 설정합니다.
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                goal_msg.pose.orientation.w = 1.0  # 오리엔테이션을 초기화합니다.

                self.goals.append(goal_msg)

            for goal_msg in self.goals:
                self.goal_callback(goal_msg)
                rospy.sleep(2)  # 목표를 설정한 후 2초 대기

                # 이동이 완료될 때까지 기다림
                while not rospy.is_shutdown():
                    try:
                        rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=5)
                        break
                    except rospy.ROSException as e:
                        rospy.logwarn(f"대기 중... {e}")

                rospy.sleep(1)  # 1초 대기

                # 다음 목표로 이동하기 전에 move_completed를 False로 설정
                self.move_completed.publish(Bool(False))

        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    try:
        nav = TurtlebotNavigation()
        nav.run()
    except rospy.ROSInterruptException:
        pass
