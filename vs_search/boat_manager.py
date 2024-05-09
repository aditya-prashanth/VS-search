import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from vs_search.boat_node import BoatNode


class BoatManager(Node):
    def __init__(self):
        super().__init__('boat_manager')
        self.boat_nodes = []
        self.executor = MultiThreadedExecutor()
        self.subscription = self.create_subscription(String, 'boat_count', self.boat_count_callback, 10)

    def boat_count_callback(self, msg):
        try:
            total_boats = int(msg.data)
            if total_boats < 0:
                raise ValueError("Boat count must be non-negative")
            self.manage_boats(total_boats)
        except ValueError as e:
            self.get_logger().error(f'Invalid boat count received: {msg.data}. Error: {str(e)}')

    def manage_boats(self, total_boats):
        for node in self.boat_nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.boat_nodes.clear()

        for boat_id in range(1, total_boats + 1):
            new_boat_node = BoatNode(boat_id, total_boats)
            self.boat_nodes.append(new_boat_node)
            self.executor.add_node(new_boat_node)
        self.get_logger().info(f'Managing {total_boats} boats.')

    def run(self):
        try:
            if self.executor:
                self.executor.spin()
        finally:
            if self.executor:
                self.executor.shutdown()
            for node in self.boat_nodes:
                node.destroy_node()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    boat_manager = BoatManager()
    boat_manager.run()

if __name__ == '__main__':
    main()
