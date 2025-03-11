import rclpy
from rclpy.node import Node 
from world_modeling_msgs.srv import BehaviourTreeInfo

from behaviour.root import Root
import py_trees

class BehaviourNode(Node):
    def __init__ (self):
        # Setup node 
        super().__init__('behaviour')
        self.client = self.create_client(BehaviourTreeInfo, 'behaviour_tree_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = BehaviourTreeInfo.Request()
        
        #Query
        #self.timer = self.create_timer(1.0, self.query_state)

        # Setup behaviour tree
        self.root = Root()
        setup_status = self.root.setup()   
        if setup_status == py_trees.common.Status.SUCCESS:
            print("Setup for behaviour tree completed successfully.")
        else:
            raise RuntimeError("Setup for behaviour tree failed")
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)  
        py_trees.display.render_dot_tree(self.root, target_directory="../")  

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        print("Sent result")
        return self.future.result()
    
    def query_state(self):
        response = self.send_request()
        self.get_logger().debug(f'Received response: {response}')
        
        #update blackboard
        self.root.blackboard.current_pos = response.current_point
        self.root.blackboard.goal_pos = response.goal_point
        self.root.blackboard.route_list = response.route_list
        print(f"Updated blackboard: current_pos={response.current_point}, goal_pos={response.goal_point}, route_list={response.route_list}")
        print("Queried")

def main(args=None):
    rclpy.init(args=args)
    behaviour_node = BehaviourNode()
    print("Check 1")
    index = 0;
    rate = behaviour_node.create_rate(1)
    # This is blocking so always keep it 
    try: 
        while rclpy.ok():
            print("Check 2")
            behaviour_node.query_state()
            print("Check 3")
            behaviour_node.root.tick() 
            print("Ticked")
            rate.sleep()
    except KeyboardInterrupt:
        pass   
    finally:
        rclpy.shutdown()
    #py_trees.trees.BehaviourTree(behaviour_node).tick_tock(period_ms=1000, pre_tick_handler=behaviour_node.query_state) # 1000 should be made a parameter
    
    #while loop  
        #RCLPY spin once
        #RCLPY tick once
    rclpy.shutdown()

if __name__ == '__main__':
    main()
