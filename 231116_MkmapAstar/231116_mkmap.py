import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap
from std_msgs.msg import Header
import numpy as np
import pymysql

class MapUploader:
    def __init__(self):
        rospy.init_node('map_upload_node', anonymous=True)

    def fetch_obstacles_from_db(self):
        # Connect to the database
        db = pymysql.connect(host='192.168.1.155',
                             user='turtlebot',
                             password='0000',
                             database='custom')

        cursor = db.cursor()

        # Example query to fetch obstacle information from the database
        query = "SELECT x, y, w, h FROM ue"

        cursor.execute(query)

        obstacles = cursor.fetchall()

        db.close()

        return obstacles

    def create_map_with_obstacles(self, obstacles, map_size_mm=1500, resolution=0.05):
        # Calculate map size in pixels
        map_size_px = int(map_size_mm / resolution)

        # Create a numpy array to represent the map
        self.occupancy_grid = np.zeros((map_size_px, map_size_px), dtype=np.uint8)

        # Add obstacles to the map
        for obstacle in obstacles:
            # Extract obstacle parameters
            x, y, width, height = obstacle

            # Convert obstacle coordinates and size from mm to pixels
            x_px = int(x / resolution)
            y_px = int(y / resolution)
            width_px = int(width / resolution)
            height_px = int(height / resolution)

            # Place the obstacle in the map
            self.occupancy_grid[
                max(0, y_px - height_px // 2): min(map_size_px, y_px + height_px // 2),
                max(0, x_px - width_px // 2): min(map_size_px, x_px + width_px // 2)
            ] = 100  # Setting obstacle value to 100 (could be any value representing an obstacle)

        return self.occupancy_grid

    def upload_map_to_map_server(self):
        rospy.wait_for_service('/dynamic_map')  # Waiting for the map service to become available
        try:
            set_map = rospy.ServiceProxy('/dynamic_map', SetMap)
            map_msg = OccupancyGrid()
            map_msg.header = Header()
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = "map"

            map_msg.info.width = self.occupancy_grid.shape[1]
            map_msg.info.height = self.occupancy_grid.shape[0]
            map_msg.info.resolution = 0.05  # Change this to your desired resolution
            map_msg.info.origin.position.x = 0.0
            map_msg.info.origin.position.y = 0.0
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.x = 0.0
            map_msg.info.origin.orientation.y = 0.0
            map_msg.info.origin.orientation.z = 0.0
            map_msg.info.origin.orientation.w = 1.0

            map_msg.data = np.reshape(self.occupancy_grid.flatten(order='C'),
                                      (map_msg.info.width * map_msg.info.height)).tolist()

            response = set_map(map_msg)
            rospy.loginfo("Map uploaded to Map Server successfully!")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to upload map: %s" % e)

if __name__ == '__main__':
    map_uploader = MapUploader()

    # Fetch obstacles from the database
    obstacles_info = map_uploader.fetch_obstacles_from_db()

    # Create the map with obstacles
    obstacles_list = list(obstacles_info)
    map_uploader.create_map_with_obstacles(obstacles_list)

    # Upload the map to Map Server
    map_uploader.upload_map_to_map_server()
