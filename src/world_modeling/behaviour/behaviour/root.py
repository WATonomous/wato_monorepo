import py_trees
from behaviour.condition_nodes import AtGoalLocation, LastNodeInRoute
from behaviour.question_nodes import EnsureCarAtNextNode, EnsureCarAtGoal
from behaviour.action_nodes import MakeCarGoToNextNodeInRouteList
from behaviour.node import Node

class Root(py_trees.composites.Selector):
    def __init__(self):
        super().__init__(name="Root", memory=True)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
        self.blackboard = py_trees.blackboard.Client(name="Global")

        # Register keys with read/write access for tracking purposes
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="route_list", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="desired_road", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="node_lane", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="last_node", access=py_trees.common.Access.WRITE)

        self.add_children([
            AtGoalLocation(),           # Check if car is at the goal
            EnsureCarAtGoal(),
        ])

    def setup(self):
        """
        Set up the root node. No hardware or external processes at the moment.
        """
        n0 = Node(name="Node 0", position=[0, 0], lane=1, segment=1)
        n1 = Node(name="Node 1", position=[1, 1], lane=1, segment=1)
        n2 = Node(name="Node 2", position=[5, 5], lane=1, segment=1)
        n3 = Node(name="Node 3", position=[10, 10], lane=1, segment=1)
        
        print("Setting up the root node and blackboard")
        self.blackboard.current_pos = n0   # Starting position of the car
        self.blackboard.goal_pos = n3      # Goal position for the car
        self.blackboard.route_list = [n0, n1, n2, n3]  # Example route with 3 nodes
        self.blackboard.next_node = self.blackboard.route_list[0]
        self.blackboard.desired_road = self.blackboard.route_list[1]
        self.blackboard.node_lane = self.blackboard.route_list[2]
        self.blackboard.last_node = self.blackboard.route_list[-1]
        
        print(self.blackboard)

        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
        print("Blackboard setup complete")
        return py_trees.common.Status.SUCCESS
