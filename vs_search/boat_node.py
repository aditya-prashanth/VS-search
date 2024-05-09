import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

class BoatNode(Node):
    def __init__(self, boat_id=1, total_boats=2, initial_latitude=34.0522, initial_longitude=-118.2437):
        super().__init__('boat_node_' + str(boat_id))
        self.publisher = self.create_publisher(NavSatFix, 'boat_gps_position', 10)
        self.boat_id = boat_id  # keep boat_id as an integer
        self.total_boats = total_boats
        self.base_angle = (360 / self.total_boats) * (boat_id - 1)
        self.current_angle = self.base_angle
        self.speed = 0.00005  # Speed adjusted for GPS coordinates
        self.latitude = initial_latitude
        self.longitude = initial_longitude
        self.timer = self.create_timer(1.0 / self.total_boats, self.update_position)

    def update_position(self):
        radian_angle = math.radians(self.current_angle)
        self.latitude += self.speed * math.sin(radian_angle)
        self.longitude += self.speed * math.cos(radian_angle)
        nav_sat_fix_msg = NavSatFix()
        nav_sat_fix_msg.header.stamp = self.get_clock().now().to_msg()
        nav_sat_fix_msg.header.frame_id = str(self.boat_id)  # Convert boat_id to string and store in frame_id
        nav_sat_fix_msg.latitude = self.latitude
        nav_sat_fix_msg.longitude = self.longitude
        self.publisher.publish(nav_sat_fix_msg)
        self.get_logger().info(f'Updated Boat {self.boat_id} Position: Latitude {self.latitude}, Longitude {self.longitude}')

def main(args=None):
    rclpy.init(args=args)
    node = BoatNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
