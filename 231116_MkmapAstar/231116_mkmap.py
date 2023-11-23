import cv2
import numpy as np
import yaml
import os
import pymysql
import time

class MapGenerator:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.obstacles_data = []

    def fetch_obstacle_data(self):
        connection = pymysql.connect(
            host=self.host,
            user=self.user,
            password=self.password,
            database=self.database,
            cursorclass=pymysql.cursors.DictCursor
        )

        try:
            with connection.cursor() as cursor:
                sql = "SELECT x_cm, y_cm, w, h FROM ue"
                cursor.execute(sql)
                self.obstacles_data = cursor.fetchall()
        finally:
            connection.close()

    def generate_map(self):
        width_meters = 1.5
        height_meters = 1.5
        resolution_mm = 1
        resolution_m = resolution_mm / 1000
        resolution_px = int(width_meters / resolution_m)

        map_img = np.zeros((resolution_px, resolution_px), dtype=np.uint8)

        obstacles = []
        for obstacle in self.obstacles_data:
            obstacles.append({
                "x": obstacle["x_cm"],
                "y": obstacle["y_cm"],
                "width": obstacle["w"],
                "height": obstacle["h"]
            })

        for obstacle in obstacles:
            x_px = int(obstacle["x"] / width_meters * resolution_px)
            y_px = int(obstacle["y"] / height_meters * resolution_px)
            width_px = int(obstacle["width"] / width_meters * resolution_px)
            height_px = int(obstacle["height"] / height_meters * resolution_px)

            cv2.rectangle(
                map_img,
                (x_px - width_px // 2, y_px - height_px // 2),
                (x_px + width_px // 2, y_px + height_px // 2),
                255,
                thickness=-1,
            )

        map_file_path = os.path.expanduser("~/catkin_ws/src/turtlebot3/turtlebot3_astar/maps")
        map_pgm_path = os.path.join(map_file_path, "map.pgm")
        cv2.imwrite(map_pgm_path, map_img)

        metadata = {
            "image": "map.pgm",
            "resolution": 0.0001,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

        map_yaml_path = os.path.join(map_file_path, "map.yaml")
        with open(map_yaml_path, "w") as yaml_file:
            yaml.dump(metadata, yaml_file, default_flow_style=False)

        print(f"맵 및 YAML 파일이 {map_file_path}에 생성되었습니다.")

    def continuous_map_generation(self):
        while True:
            self.fetch_obstacle_data()
            self.generate_map()
            time.sleep(10)

# Usage example
map_generator = MapGenerator('192.168.1.155', 'turtlebot', '0000', 'custom')
map_generator.continuous_map_generation()  # Generates maps continuously with a default interval of 60 seconds
