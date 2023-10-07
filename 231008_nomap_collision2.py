#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
import pymysql
import threading

class TurtlebotNavigation:

    def __init__(self):
        rospy.init_node('custom_turtlebot3_navigation', anonymous=True)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.move_completed = rospy.Publisher('/move_completed', Bool, queue_size=10)

        self.stop_flag = False
        self.stop_lock = threading.Lock()

        try:
            # Establish database connection
            self.connection = pymysql.connect(host='192.168.1.155',
                                              user='turtlebot',
                                              password='0000',
                                              database='test')
            self.cursor = self.connection.cursor()

            # Create table if not exists
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS one (
                    id INT AUTO_INCREMENT PRIMARY KEY,
                    x FLOAT,
                    y FLOAT,
                    theta FLOAT,
                    stop INT DEFAULT 0
                )
            ''')
            self.connection.commit()

            print("Database connection successful.")
        except Exception as e:
            print(f"Error: {e}")

        self.stop_thread = threading.Thread(target=self.monitor_stop)
        self.stop_thread.daemon = True
        self.stop_thread.start()

    def goal_callback(self, msg):
        self.goal_publisher.publish(msg)

    def odom_callback(self, msg):
        with self.stop_lock:
            if self.stop_flag:
                rospy.loginfo("Navigation is paused. Waiting for resume.")
                rospy.sleep(5)  # Pause for 5 seconds before checking again.
                return

            position_x = msg.pose.pose.position.x
            position_y = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            rospy.loginfo(f"Position (x, y, theta): ({position_x:.2f}, {position_y:.2f}, {yaw:.2f})")

            # Insert the coordinates into the database
            self.cursor.execute('''
                INSERT INTO one (x, y, theta) VALUES (%s, %s, %s)
            ''', (position_x, position_y, yaw))
            self.connection.commit()

    def check_stop(self):
        self.cursor.execute("SELECT stop FROM three ORDER BY id DESC LIMIT 1")
        result = self.cursor.fetchone()
        return result[0] if result else 0

    def monitor_stop(self):
        while not rospy.is_shutdown():
            with self.stop_lock:
                self.stop_flag = bool(self.check_stop())
            rospy.sleep(1)  # Check stop value every second

    def run(self):
        rospy.loginfo("Press 'Ctrl + C' to exit.")

        try:
            while True:
                with self.stop_lock:
                    if self.stop_flag:
                        rospy.loginfo("Navigation is paused. Waiting for resume.")
                        rospy.sleep(5)  # Pause for 5 seconds before checking again.
                        continue

                x = input("목표 좌표를 입력하세요 (띄어쓰기로 구분, 'done'으로 종료): ")
                if x.lower() == 'done':
                    break
                x, y = map(float, x.split())

                # 목표 좌표를 설정합니다.
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                goal_msg.pose.orientation.w = 1.0  # 오리엔테이션을 초기화합니다.

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
        finally:
            self.connection.close()

if __name__ == '__main__':
    try:
        nav = TurtlebotNavigation()
        nav.run()
    except rospy.ROSInterruptException:
        pass
