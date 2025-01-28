import py_trees
import condition_nodes
import action_nodes
import question_nodes

# Question Node: Check if the car is at the goal
class EnsureCarAtGoal(py_trees.composites.Selector):
    def __init__(self):
        super(EnsureCarAtGoal, self).__init__(name="Ensure Car At Goal",memory=True )
        self.add_children([condition_nodes.LastNodeInRoute(), question_nodes.EnsureCarAtNextNode()])

    def update(self):
        print("Checking if the car is at the goal.")
        return super(EnsureCarAtGoal, self).tick()

# Question Node: Check if the car is at the next node in the route
class EnsureCarAtNextNode(py_trees.composites.Selector):
    def __init__(self):
        super(EnsureCarAtNextNode, self).__init__(name="Ensure Car At Next Node",memory=True)
        self.add_children([ action_nodes.RemoveNodeFromRoute(), action_nodes.MakeCarGoToNextNodeInRouteList()])

    def update(self):
        print("Checking if the car is at the next node in the route.")
        return super(EnsureCarAtNextNode, self).tick()

# Question Node: Check if the car is on the correct road
class EnsureCarOnCorrectRoad(py_trees.composites.Selector):
    def __init__(self):
        super(EnsureCarOnCorrectRoad, self).__init__(name="Ensure Car On Correct Road",memory=True)
        self.add_children([condition_nodes.CheckCarOnDesiredRoad(), action_nodes.MakeTurnToReachRoad()])

    def update(self):
        print("Checking if the car is on the correct road.")
        return super(EnsureCarOnCorrectRoad, self).tick()

# Question Node: Check if the car is in the correct lane
class EnsureCarOnCorrectLane(py_trees.composites.Selector):
    def __init__(self):
        super(EnsureCarOnCorrectLane, self).__init__(name="Ensure Car On Correct Lane",memory=True)
        self.add_children([condition_nodes.CheckCarOnCorrectLane(), action_nodes.SwapToNodeLane()])

    def update(self):
        print("Checking if the car is in the correct lane.")
        return super(EnsureCarOnCorrectLane, self).tick()
    
class AtNextNode(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(AtNextNode, self).__init__(name="At Next Node")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.current_pos.position == self.blackboard.next_node.position:
            print("Car is at the next node.")
            return py_trees.common.Status.SUCCESS
        else:
            print("Car is not yet at the next node.")
            return py_trees.common.Status.FAILURE