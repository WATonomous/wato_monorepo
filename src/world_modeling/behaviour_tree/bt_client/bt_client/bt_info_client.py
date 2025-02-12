import rclpy
from rclpy.node import Node 
from world_modeling_msgs.srv import BehaviourTreeInfo
from geometry_msgs.msg import Point

class BtClient(Node):
    def __init__ (self):
        super().__init__('bt_info_client')
        self.client = self.create_client(BehaviourTreeInfo, 'behaviour_tree_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = BehaviourTreeInfo.Request()

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    bt_info_client = BtClient()
    response = bt_info_client.send_request()
    bt_info_client.get_logger().info(f'Received response: {response}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()