import rospy
from heapq import heappush, heappop
from nav_msgs.msg import OccupancyGrid, Odometry  # Odometry 메시지 추가
from geometry_msgs.msg import PoseStamped

class AStarPlanner:
    def __init__(self):
        self.resolution = 0.001  # 맵 해상도 (meter per pixel)
        self.map_width = 1.5  # 맵 가로 길이
        self.map_height = 1.5  # 맵 세로 길이
        self.start = (0, 0)  # 시작 위치 (x, y)
        self.goal = (1, 0)   # 목표 위치 (x, y)
        self.obstacles = [((0.5, 0), (1, 1))]  # 장애물의 절대 좌표와 크기

        # ROS 노드 초기화
        rospy.init_node('a_star_planner')
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)  # Odometry 퍼블리셔

    def heuristic(self, a, b):
        # 휴리스틱 함수 (Manhattan 거리)
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_collision(self, point):
        # 주어진 좌표가 장애물과 충돌하는지 확인
        for obstacle in self.obstacles:
            if obstacle[0][0] <= point[0] <= obstacle[1][0] and obstacle[0][1] <= point[1] <= obstacle[1][1]:
                return True
        return False

    def get_neighbors(self, node):
        # 주어진 노드의 이웃 노드 반환
        x, y = node
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        return [(i, j) for i, j in neighbors if 0 <= i < self.map_width and 0 <= j < self.map_height and not self.is_collision((i, j))]

    def a_star(self):
        open_set = []
        heappush(open_set, (0, self.start))
        came_from = {}
        cost_so_far = {self.start: 0}

        while open_set:
            current = heappop(open_set)[1]

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path.reverse()

                # 경로를 이동 명령으로 변환하여 발행
                for position in path:
                    pose = PoseStamped()
                    pose.pose.position.x = position[0] * self.resolution
                    pose.pose.position.y = position[1] * self.resolution
                    self.goal_pub.publish(pose)
                    rospy.sleep(1)  # 로봇 속도 및 경로 이동 간격에 따라 조절
                break

            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1  # 이동 비용은 일정하다고 가정 (맵이 이진으로 표현되어 있음)

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(self.goal, neighbor)
                    heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

                    # Odometry 정보 생성 및 발행
                    odom = Odometry()
                    odom.pose.pose.position.x = neighbor[0] * self.resolution
                    odom.pose.pose.position.y = neighbor[1] * self.resolution
                    rospy.loginfo("Odometry: x=%f, y=%f", odom.pose.pose.position.x, odom.pose.pose.position.y)
                    self.odom_pub.publish(odom)

    def run(self):
        rospy.loginfo("A* planner started.")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.a_star()
            rate.sleep()

if __name__ == '__main__':
    planner = AStarPlanner()
    planner.run()
