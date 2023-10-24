#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def movebase_client(x, y):
    rospy.init_node('movebase_client_py')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0
    while not rospy.is_shutdown():
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        x = float(input("x 좌표를 입력하세요: "))
        y = float(input("y 좌표를 입력하세요: "))
        movebase_client(x, y)
    except rospy.ROSInterruptException:
        pass
