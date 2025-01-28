import py_trees

class Root(py_trees.composites.Sequence):
    def __init__(self):
        """
        Defines the children nodes of this root.
        """
        super(Root, self).__init__(name="root", memory=False)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
    
    def setup(self):
        """
        Starts seperate threads/proccesses, connect to hardware, etc.
        """
        print("Called .setup()")
    
    def initialise(self):
        """
        What to run right before the node can start ticking.
        """
        print("Called .initialised() for root")
    
    def update(self):
        """
        What to run when behaviour is ticked
        """
        self.logger.debug("Root updated!")

class check_at_goal(py_trees.behaviour.Behaviour):
    def __init__(self):
        """
        Condition node
        """
        super(check_at_goal, self).__init__("check_at_goal")
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.counter = 0
    
    def setup(self):
        pass
    
    def initialise(self):
        print("Called .initialised() for check_at_goal")
    
    def update(self):
        """
        What to run when behaviour is ticked
        """
        if self.counter < 7:
            self.counter += 1
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
        

def create_root():
    """
    Demo single thread BT w/ 2 nodes, just for me to get something to run :)
    """
    # Create a root node
    root = Root()
    root.add_child(check_at_goal())

    return root

if __name__ == '__main__':
    # Create the root of the tree
    root = create_root()
    tree = py_trees.trees.BehaviourTree(root)

    # some ticks
    for i in range(10):
        tree.tick()
        print(f"Tree status: {tree.root.status}")
