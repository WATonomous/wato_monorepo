import rclpy
from rclpy.node import Node 
from world_modeling_msgs.srv import BehaviourTreeInfo

from behaviour.root import Root
import py_trees

class BehaviourNode(Root, Node):
    def __init__ (self):
        # Setup node 
        Node.__init__(self, 'behaviour')
        self.client = self.create_client(BehaviourTreeInfo, 'behaviour_tree_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = BehaviourTreeInfo.Request()

        # Setup behaviour tree
        Root.__init__(self)
        setup_status = self.setup()   
        if setup_status == py_trees.common.Status.SUCCESS:
            print("Setup for behaviour tree completed successfully.")
        else:
            raise RuntimeError("Setup for behaviour tree failed")
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)  
        py_trees.display.render_dot_tree(self, target_directory="../")  

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def query_state(self, _):
        response = self.send_request()
        self.get_logger().debug(f'Received response: {response}')

def main(args=None):
    rclpy.init(args=args)
    behaviour_node = BehaviourNode()

    # This is blocking so always keep it 
    py_trees.trees.BehaviourTree(behaviour_node).tick_tock(period_ms=1000, pre_tick_handler=behaviour_node.query_state) # 1000 should be made a parameter

    rclpy.shutdown()

if __name__ == '__main__':
    main()
