import graph as gp


# Define Agent class
class Agent:
    # Initialize Agent object
    def __init__(self, name, graph: gp.Graph, initial_state: gp.Node, goal_state: gp.Node, heuristic_type):
        # Set attributes
        self.name = name
        self.state_space = graph
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.current_state = self.initial_state
        self.heuristic_type = heuristic_type
        self.fringe_nodes = {}
        self.closed_nodes = {}
        self.predecessors = {}

        # Initialize fringe nodes with initial state
        self.add_fringe_nodes()

        # Add initial state to closed nodes
        self.closed_nodes[self.current_state] = 0

    # Method to calculate the heuristic value of a node
    def heuristic(self, node, goal_node, traffic_level):

        # No heuristic
        def zero(node, goal_node, traffic_level):
            return 0

        # Euclidean distance heuristic
        def euclidean(node, goal_node, traffic_level):
            # Calculate euclidean distance from node to goal node
            return gp.euc_dist(node.pos, goal_node.pos)

        # Euclidean distance with traffic awareness heuristic
        def traffic_aware(node, goal_node, traffic_level):
            # Calculate euclidean distance from node to goal node
            distance = gp.euc_dist(node.pos, goal_node.pos)
            # Calculate traffic factor based on traffic level,
            # will be 50% higher for each unit increase in the traffic level
            traffic_factor = 1 + traffic_level * 0.5
            # Return euclidean distance multiplied by traffic factor
            return distance * traffic_factor

        # Dictionary of heuristic functions
        heuristic_functions = {
            'None': zero,
            'Euclidean': euclidean,
            'Euclidean + Traffic Aware': traffic_aware
        }

        # Select the appropriate heuristic function based on the heuristic_type attribute
        # Essentially, this is a switch statement
        heuristic_function = heuristic_functions.get(self.heuristic_type, euclidean)

        # Call the selected heuristic function
        return heuristic_function(node, goal_node, traffic_level)

    # Method to add fringe nodes from current state
    def add_fringe_nodes(self, travel_time=0):
        # Loop through all neighbors of current state
        for node in self.current_state.neighbors:
            # Calculate euclidean distance from node to goal state and use as heuristic
            heuristic = self.heuristic(node, self.goal_state, self.current_state.neighbors[node][1])
            # Calculate actual cost from current state to node in travel time
            actual_cost = self.current_state.neighbors[node][2] + travel_time
            # Calculate f cost of node
            f_cost = heuristic + actual_cost
            # Check if node is in closed nodes
            if node in self.closed_nodes.keys():
                # Check if actual cost is less than cost in closed nodes
                if actual_cost < self.closed_nodes[node]:
                    # Add node to fringe nodes with f cost as key
                    self.fringe_nodes[f_cost] = (node, actual_cost)
            # If node is not in closed nodes
            else:
                # Add node to fringe nodes with f cost as key
                self.fringe_nodes[f_cost] = (node, actual_cost)
            # Update predecessor of node
            if node not in self.predecessors.keys() or actual_cost < self.predecessors[node][1]:
                # Add node to predecessors with current state as predecessor and actual cost as cost
                self.predecessors[node] = (self.current_state, actual_cost)

    # Method to expand node with lowest f cost
    def expand_node(self, f_cost):
        # Update current state
        self.current_state = self.fringe_nodes[f_cost][0]  # Get the Node instance
        # Add fringe nodes from current state, gives actual cost
        self.add_fringe_nodes(self.fringe_nodes[f_cost][1])
        # Add current state to closed nodes, gives actual cost
        self.closed_nodes[self.current_state] = self.fringe_nodes[f_cost][1]
        # Remove current state from fringe nodes
        del self.fringe_nodes[f_cost]

    # Method to get the best path from initial state to goal state
    def get_best_path(self):
        # Initialize path list
        path = []
        # Initialize node as goal state
        node = self.goal_state
        # Loop until node is initial state
        while node != self.initial_state:
            # Append node to path
            path.append(node)
            # Update node to predecessor of node
            node = self.predecessors[node][0]
        # Append initial state to path
        path.append(self.initial_state)
        # Return path in reverse order and cost of best path
        return path[::-1], self.predecessors[self.goal_state][1]

    # Method to perform A* search
    def astar_search(self):
        # Loop until goal state is reached
        while self.current_state != self.goal_state:
            # Check if fringe nodes is empty
            if not self.fringe_nodes:
                # Return None, None if goal state is not reached
                return None, None
            # If fringe nodes is not empty
            else:
                # Get node with the lowest total cost from fringe nodes
                min_f_cost = min(self.fringe_nodes.keys())
                # Check if node is in closed nodes
                if self.fringe_nodes[min_f_cost] in self.closed_nodes.keys():
                    # Check if total cost of node in fringe nodes is less than total cost of node in closed nodes
                    if min_f_cost < self.closed_nodes[self.fringe_nodes[min_f_cost]]:
                        self.expand_node(min_f_cost)
                    # Remove current state from fringe nodes
                    else:
                        del self.fringe_nodes[min_f_cost]
                # If node is not in closed nodes
                else:
                    self.expand_node(min_f_cost)
        # Return best path
        return self.get_best_path()

    # Method to get the explored nodes
    def explored_nodes(self):
        # Return keys of closed nodes
        return self.closed_nodes.keys()