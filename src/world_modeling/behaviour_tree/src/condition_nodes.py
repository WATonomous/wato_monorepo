import py_trees


# Condition Node: Check if the car is at the goal location
class AtGoalLocation(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(AtGoalLocation, self).__init__(name="At Goal Location")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_pos", access=py_trees.common.Access.READ)

    def setup(self):
        print("running setup")

    def update(self):
        if self.blackboard.current_pos.position == self.blackboard.goal_pos.position:
            print("Car is at the goal location.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Car is not at the goal location yet.")
            return py_trees.common.Status.FAILURE

# Condition Node: Check if the last node in the route is reached
class LastNodeInRoute(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(LastNodeInRoute, self).__init__(name="Last Node In Route")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="route_list", access=py_trees.common.Access.READ)

    def update(self):
        if len(self.blackboard.route_list) == 0:  # Assume last node when none left 
            print("Last node in route reached.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Still more nodes in the route.")
            return py_trees.common.Status.FAILURE


# Condition Node: Check if the car is at the next node in the route
class AtNextNode(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(AtNextNode, self).__init__(name="At Next Node")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.WRITE)

    def update(self):
        if self.blackboard.current_pos.position == self.blackboard.next_node.position:
            print("Car is at the next node.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Car is not yet at the next node.")
            return py_trees.common.Status.FAILURE
 
 
# Condition Node: Check if the car is on the desired road       
class CheckCarOnDesiredRoad(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(CheckCarOnDesiredRoad, self).__init__(name="Check Car On Desired Road")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="desired_road", access=py_trees.common.Access.WRITE)

    def update(self):
        if self.blackboard.current_pos.position == self.blackboard.desired_road.position:
            print("Car is on the desired road.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Car is not on the desired road.")
            return py_trees.common.Status.FAILURE
        
        
# Condition Node: Check if the car is on the correct lane
class CheckCarOnCorrectLane(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(CheckCarOnCorrectLane, self).__init__(name="Check Car On Correct Lane")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_lane", access=py_trees.common.Access.WRITE)

    def update(self):
        # TODO: IMPLEMENT CHECK if the car is on the correct lane
        print("Car is on the correct lane.")
        return py_trees.common.Status.SUCCESS
        """
        if self.blackboard.current_pos == self.blackboard.current_lane:
            print("Car is on the correct lane.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Car is not on the correct lane.")
            return py_trees.common.Status.FAILURE
        """