import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json

class DatumNode(Node):
    def __init__(self, max_positions=50):
        super().__init__('datum_node')
        self.max_positions = max_positions  # Maximum number of positions to store
        self.publisher = self.create_publisher(String, 'boat_positions', 10)
        self.subscription = self.create_subscription(
            NavSatFix, 'boat_gps_position', self.handle_boat_update, 10)
        self.positions = []

    def handle_boat_update(self, msg):
        """Handle incoming NavSatFix messages by storing the latest position and publishing the aggregated data."""
        # Extract boat_id from frame_id and store it with the position
        position = {
            'boat_id': msg.header.frame_id,  # Use the frame_id as boat_id
            'latitude': msg.latitude,
            'longitude': msg.longitude
        }
        self.positions.append(position)
        # Maintain the list size within the specified maximum
        if len(self.positions) > self.max_positions:
            self.positions.pop(0)
        self.publish_positions()

    def publish_positions(self):
        """Publish the list of positions as a JSON-encoded string."""
        try:
            msg = String()
            msg.data = json.dumps(self.positions)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish positions: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    datum_node = DatumNode(max_positions=50)  # Adjustable maximum positions
    rclpy.spin(datum_node)
    datum_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
