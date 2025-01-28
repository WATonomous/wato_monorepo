import py_trees
import question_nodes

# Action Node: Drive straight at the node
class CarDriveStraightAtNode(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(CarDriveStraightAtNode, self).__init__(name="Car Drive Straight At Node")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.WRITE)
    
    def update(self):
        print("Car is driving straight towards the node.")
        # Set current position to next node (assuming car has driven straight towards the node)
        self.blackboard.current_pos = self.blackboard.next_node
        return py_trees.common.Status.SUCCESS

# Action Node: Make the car go to the next node in the route list
class MakeCarGoToNextNodeInRouteList(py_trees.composites.Sequence):
    def __init__(self):
        super(MakeCarGoToNextNodeInRouteList, self).__init__(name="Make Car Go To Next Node In Route List", memory=True)
        
        # Add children in sequence for ensuring road, lane, and driving towards node
        self.add_children([
            question_nodes.EnsureCarOnCorrectRoad(),
            question_nodes.EnsureCarOnCorrectLane(),
            CarDriveStraightAtNode()
        ])

    def tick(self):
        # Execute each child in sequence by calling the parent tick method
        result = super(MakeCarGoToNextNodeInRouteList, self).tick() 

        # After iterating, 'result' should hold the final status of the sequence
        if result == py_trees.common.Status.SUCCESS:
            print("Making the car go to the next node in the route list.")
        elif result == py_trees.common.Status.FAILURE:
            print("Failed to make the car go to the next node in the route list.")
        else:
            print("MakeCarGoToNextNodeInRouteList is still running.")

        return result

# Action Node: Make the car turn to reach the desired road
class MakeTurnToReachRoad(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(MakeTurnToReachRoad, self).__init__(name="Make Turn To Reach Road")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.WRITE)
        
        
    def update(self):
        print("Making a turn to reach the desired road.")
        return py_trees.common.Status.SUCCESS
        """
        # assuming dtype of each node has road attribute
        if self.blackboard.current_pos.road == self.blackboard.next_node.road:
            print("Car is making a turn to reach the desired road.")
            self.blackboard.current_pos.road = self.blackboard.next_node.road
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        """
        
        

# Action Node: Swap to the node lane
class SwapToNodeLane(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(SwapToNodeLane, self).__init__(name="Swap To Node Lane")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.WRITE)

    def update(self):
        """
        if self.blackboard.current_pos.lane != self.blackboard.next_node.lane:
            print("Swapping to the node lane.")
            self.blackboard.current_pos.lane = self.blackboard.next_node.lane
        else:
            print("Car is already on the node lane.")
        """
            
        return py_trees.common.Status.SUCCESS

# Action Node: Mark node as completed and remove from the route
class RemoveNodeFromRoute(py_trees.composites.Sequence):
    def __init__(self):
        super(RemoveNodeFromRoute, self).__init__(name="Remove Node From Route", memory=True)
        self.add_children([ question_nodes.AtNextNode(), MarkNodeAsCompleted()])

    def tick(self):
        # Tick through each child in sequence
        result = super(RemoveNodeFromRoute, self).tick()
        
        # Check if all children have completed successfully
        if result == py_trees.common.Status.SUCCESS:
            print("Removed Node From Route.")
    
            # Logic to remove the node
            return py_trees.common.Status.SUCCESS
        
        return result

# Action Node: Mark the current node as completed
class MarkNodeAsCompleted(py_trees.behaviour.Behaviour):
    def __init__(self):
        super(MarkNodeAsCompleted, self).__init__(name="Mark Node As Completed")
        self.blackboard = self.attach_blackboard_client(name="Global")
        self.blackboard.register_key(key="route_list", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="next_node", access=py_trees.common.Access.WRITE)

    def update(self):
        if len(self.blackboard.route_list) != 0:
            print("Marking the current node as completed.")
            self.blackboard.current_pos = self.blackboard.next_node
            self.blackboard.route_list.pop(0)
            # Check if there's a new next node after removing the current one
            if len(self.blackboard.route_list) > 0:
                self.blackboard.next_node = self.blackboard.route_list[0]
            else:
                self.blackboard.next_node = None  # No more nodes left
            print(self.blackboard.route_list)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE