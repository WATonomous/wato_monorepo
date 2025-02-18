import rclpy
from rclpy.node import Node 
from world_modeling_msgs.srv import BehaviourTreeInfo

from behaviour.root import Root
import py_trees

class Behaviour(Root, Node):
    def __init__ (self):
        # Setup node 
        super(Node, self).__init__('bt_info_client')
        self.client = self.create_client(BehaviourTreeInfo, 'behaviour_tree_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = BehaviourTreeInfo.Request()

        # Setup behaviour tree
        super(Root, self).__init__()
        setup_status = self.setup()   
        if setup_status == py_trees.common.Status.SUCCESS:
            print("Setup for behaviour tree completed successfully.")
        else:
            raise RuntimeError("Setup for behaviou tree failed")
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)  
        py_trees.display.render_dot_tree(self, target_directory="../")  
        py_trees.trees.BehaviourTree(self).tick_tock(period_ms=1000) # 1000 should be made a parameter

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def update(self):
        """
        Executes the sequence of actions by ticking the behavior tree.
        """
        self.logger.debug("Root is updating.")
        return super().tick()

def main(args=None):
    rclpy.init(args=args)
    behaviour_node = Behaviour()

    response = behaviour_node.send_request()
    behaviour_node.get_logger().info(f'Received response: {response}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
