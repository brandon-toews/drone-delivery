import random
import math
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.cm as cm
from matplotlib.colors import Normalize

# Default names for nodes
default_names = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z'
    ]


# Graph class
class Graph:
    # Initialize graph with width, height, number of nodes, and input names
    def __init__(self, width, height, num_nodes, input_names=None):
        # Set default names if none are provided
        if input_names is None:
            input_names = default_names
        self.width = width
        self.height = height
        self.num_nodes = num_nodes
        self.names = input_names
        # Initialize node list
        self.nodes = {}

        # Create nodes and connections
        self.create_nodes()
        self.connect_nodes()

    # Create initial nodes specified at instantiation of graph
    def create_nodes(self):
        # Create Hub Node
        hub_pos = (int(self.width/2), int(self.height/2))
        self.nodes['Hub'] = Node('Hub', hub_pos)

        # Set minimum distance between nodes
        min_distance = (self.width+self.height/2)/10

        # Set maximum number of attempts to find a new position
        max_attempts = 1000

        # Create rest of specified nodes in graph
        if self.num_nodes < 28:
            # Loop through number of nodes specified
            for i in range(self.num_nodes-1):

                # Initialize position of node
                pos = None

                # Try to find a position that is
                # far enough from all other nodes for a certain number of attempts
                for attempt in range(max_attempts):
                    # Generate random position for node
                    pos = (random.randint(0, self.width), random.randint(0, self.height))
                    # Check if position is far enough from all nodes
                    if all(euc_dist(node.pos, pos) >= min_distance for node in self.nodes.values()):
                        # Position is far enough from all nodes, break the loop
                        break
                # Add node to list
                self.nodes[self.names[i]] = Node(self.names[i], pos)
        # If more than 26 nodes are specified, create nodes with two letter names
        else:
            # Loop through names list
            for i in range(len(self.names)):
                # Loop through names list again
                for j in range(len(self.names)):
                    # stop creating nodes when specified amount is reached
                    if len(self.nodes) == self.num_nodes:
                        break

                    # Initialize position of node
                    pos = None

                    # Try to find a position that is
                    # far enough from all other nodes for a certain number of attempts
                    for attempt in range(max_attempts):
                        # Generate random position for node
                        pos = (random.randint(0, self.width), random.randint(0, self.height))
                        # Check if position is far enough from all nodes
                        if all(euc_dist(node.pos, pos) >= min_distance for node in self.nodes.values()):
                            # Position is far enough from all nodes, break the loop
                            break

                    # Add node to list
                    self.nodes[self.names[i]+self.names[j]] = Node(self.names[i]+self.names[j], pos)

    # Create connections of nodes in graph
    def connect_nodes(self):
        # Loop through all nodes in graph
        for node in self.nodes.values():
            # store next closest node and distance just in case
            # there are no nodes within range to connect to
            next_closest_distance = sys.maxsize
            next_closest_node = None

            # Loop through all the other nodes in graph
            for other_node in self.nodes.values():
                # calculate euclidean distance of selected nodes
                distance = int(euc_dist(node.pos, other_node.pos))

                # if other node is not already a neighbor
                if other_node not in node.neighbors:
                    # if both nodes selected aren't the same and distance is lower
                    # than the closest node found so far
                    if not node == other_node and distance < next_closest_distance:
                        # store closest node
                        next_closest_distance = distance
                        next_closest_node = other_node

                    # if both nodes selected aren't the same and distance is close enough
                    if not node == other_node and distance < int(self.width/4):
                        # Add node to neighbors dict with distance
                        # and randomly generated traffic for connection
                        traffic = random.randint(1, 5)

                        # Calculate speed(mph) based on traffic level
                        # 1 = 30mph, 2 = 25mph, 3 = 20mph, 4 = 15mph, 5 = 10mph
                        speed = 35 - traffic * 5

                        # Calculate travel cost based on speed converted to minutes
                        travel_cost = int((distance / speed) * 60)

                        # Add node to neighbors dict with distance, traffic, and travel cost
                        node.neighbors[other_node] = [distance, traffic, travel_cost]
                        other_node.neighbors[node] = [distance, traffic, travel_cost]

            # If next closest node was found and not already a neighbor
            if next_closest_node not in node.neighbors:
                # if no nodes were close enough to connect to than connect to
                # the closest node
                if not len(node.neighbors):
                    # Add node to neighbors dict with distance
                    # and randomly generated traffic for connection
                    traffic = random.randint(1, 5)

                    # Calculate speed(mph) based on traffic level
                    # 1 = 30mph, 2 = 25mph, 3 = 20mph, 4 = 15mph, 5 = 10mph
                    speed = 35 - traffic * 5

                    # Calculate travel cost based on speed converted to minutes
                    travel_cost = int((next_closest_distance / speed) * 60)

                    # Add node to neighbors dict with distance, traffic, and travel cost
                    node.neighbors[next_closest_node] = [next_closest_distance, traffic, travel_cost]
                    next_closest_node.neighbors[node] = [next_closest_distance, traffic, travel_cost]

    # Function to plot graph with or without selected nodes
    def plot_graph(self, selected_node_names=None, astar_exploration=None, astar_path=None, drones=None):
        # Create figure
        f = plt.figure(figsize=(11, 10))

        # Traffic color dictionary
        traffic_colors = {1: "green", 2: "limegreen", 3: "gold",
                          4: "darkorange", 5: "red"}

        # Urgency color dictionary
        cmap = cm.get_cmap('cool')
        urgencies = {i: cmap(i / 5) for i in range(6)}

        # Plot connection to explored nodes
        if astar_exploration is not None:
            # Loop through all nodes in graph
            for node in astar_exploration:
                # Loop through neighbors of node
                for neighbor in node.neighbors:
                    # Check if neighbor is in astar_exploration
                    if neighbor in astar_exploration:
                        # Plot explored connections
                        plt.plot(
                            # Plot line between nodes
                            [node.pos[0], neighbor.pos[0]],
                            [node.pos[1], neighbor.pos[1]],
                            # Set color of line of explored nodes
                            color='hotpink',
                            # Set line width
                            linewidth=15)

        # Plot drone paths if drones are passed
        if drones is not None:
            # Create a color map for drones
            drone_colors = plt.cm.get_cmap('rainbow', len(drones))

            # Create a list to store the patches for the legend
            drone_patches = []

            # Loop through drones
            for i, drone in enumerate(drones):

                # Get the color for this drone
                drone_color = drone_colors(i)

                # Create a patch for this drone and add it to the list
                drone_patches.append(mpatches.Patch(color=drone_color, label=f'Drone {i}'))

                # Loop through each node in drone path and draw the path between nodes
                for j in range(len(drone.path) - 1):

                    # Get the current node and the next node
                    current_node = drone.path[j]
                    next_node = drone.path[j + 1]

                    # Calculate the interpolated position to draw the arrow only partially across the connection
                    ratio = 0.8
                    interp_pos = (ratio * next_node.pos[0] + (1 - ratio) * current_node.pos[0],
                                  ratio * next_node.pos[1] + (1 - ratio) * current_node.pos[1])

                    # Offset for the arrow so it doesn't overlap the connection line
                    offset = i * 1.5

                    # Show the direction of travel from node to node
                    plt.annotate("",
                                 xy=(interp_pos[0] + offset, interp_pos[1] + offset),
                                 xytext=(current_node.pos[0] + offset, current_node.pos[1] + offset),
                                 arrowprops=dict(arrowstyle="->", color=drone_color,
                                                 linewidth=5))
                # Plot drone legend
                f.legend(handles=drone_patches, title="Drones", bbox_to_anchor=(0.9, 0.5), loc='upper left')

        # Plot A*'s best path if astar_path is passed
        if astar_path is not None:
            # Loop through all nodes in graph
            for i in range(len(astar_path) - 1):

                # Get the current node and the next node
                node = astar_path[i]
                next_node = astar_path[i + 1]
                # Plot best path
                plt.plot(
                    # Plot line between nodes
                    [node.pos[0], next_node.pos[0]],
                    [node.pos[1], next_node.pos[1]],
                    # Set color of line based of best path
                    color='cyan',
                    # Set line width
                    linewidth=15)

        # Plot normal connections between nodes
        # Loop through all nodes in graph
        for node in self.nodes.values():
            # Loop through neighbors of node
            for neighbor in node.neighbors:

                # Plot connections
                plt.plot(
                    # Plot line between nodes
                    [node.pos[0], neighbor.pos[0]],
                    [node.pos[1], neighbor.pos[1]],
                    # Set color of line based on traffic level
                    color=traffic_colors[node.neighbors[neighbor][1]],
                    # Set line width
                    linewidth=4)

                # Calculate midpoint
                mid_x = (node.pos[0] + neighbor.pos[0]) / 2
                mid_y = (node.pos[1] + neighbor.pos[1]) / 2

                # Calculate speed(mph) based on traffic level
                # 1 = 30mph, 2 = 25mph, 3 = 20mph, 4 = 15mph, 5 = 10mph
                speed = 35 - node.neighbors[neighbor][1]*5

                # Calculate travel cost based on speed converted to minutes
                travel_cost = int((node.neighbors[neighbor][0] / speed) * 60)

                # Display distance
                # data[0] is the distance
                plt.text(mid_x, mid_y, str(node.neighbors[neighbor][0])+'mi')
                # data[2] is the travel cost
                plt.text(mid_x - 10.0, mid_y - 5.0, 'Cost: ' + str(node.neighbors[neighbor][2]) + 'min')

        # Plot nodes
        for node in self.nodes.values():
            # Place plots
            # Plot hub node
            if node.name == 'Hub':
                plt.plot(node.pos[0], node.pos[1], 'o', markersize=25, color='blue')
            # Plot normal nodes
            else:
                plt.plot(node.pos[0], node.pos[1], 'o', markersize=20, color='purple')

            # Display name of node and center the text
            plt.text(node.pos[0], node.pos[1], node.name, ha='center', va='center', color='white', weight='semibold')

        # Plot selected nodes in the genetic algorithm if selected_node_names is passed
        if selected_node_names is not None:
            # Select nodes to plot
            selected_nodes = self.select_nodes(selected_node_names)

            # Plot selected nodes with urgency values
            for node in selected_nodes:
                # Determine the size and color of the node based on its urgency value
                urgency = node.delivery_urgency
                node_size = 20 + 4 * urgency
                node_color = cm.ScalarMappable(norm=Normalize(vmin=0, vmax=5), cmap=cmap).to_rgba(urgency)

                # Plot selected nodes
                plt.plot(node.pos[0], node.pos[1], 'o', markersize=node_size, color=node_color)
                plt.text(node.pos[0], node.pos[1], node.name, ha='center', va='center',
                         color='white', weight='semibold')

        # Produce patches for plot legend from traffic colors dict
        traffic_patches = [mpatches.Patch(color=traffic_colors[key], label=key) for key in traffic_colors]

        # Produce patches for plot legend from urgency colors dict
        urgency_patches = [mpatches.Patch(color=urgencies[key], label=key if key != 0 else 'None') for key in urgencies]

        # Plot traffic legend
        f.legend(handles=traffic_patches, title="Traffic Levels", bbox_to_anchor=(0.9, 0.9), loc='upper left')

        # Plot urgency legend
        f.legend(handles=urgency_patches, title="Urgency Levels", bbox_to_anchor=(0.9, 0.75), loc='upper left')

        # Return figure
        return f

    # Function to return selected nodes
    def select_nodes(self, selected_names):
        # Initialize list of selected nodes
        selected_nodes = []
        # Loop through all nodes in graph
        for node in self.nodes.values():
            # Check if node name is in selected names
            if node.name in selected_names:
                # Add node to selected nodes list
                selected_nodes.append(node)
        # Return list of selected nodes
        return selected_nodes


# Euclidean distance function
def euc_dist(a, b):
    return math.sqrt(
        math.pow(a[0]-b[0], 2) +
        math.pow(a[1]-b[1], 2)
    )


# Function to print names of path
def retrieve_path_names(path):
    # Check if path is None
    if path is None:
        # Return no path found
        return 'No path found'
    # If path is not None
    else:
        # Initialize path string
        path_str = ''
        # Loop through all nodes in path
        for i in range(len(path)-1):
            # Add name of node to path string
            path_str += path[i].name + ' -> '
        # Add last node name to path string
        path_str += path[len(path)-1].name
        # Return path string
        return path_str


# Node class
class Node:
    # Initialize node with name and position
    def __init__(self, name, pos):
        self.name = name
        self.pos = pos
        # Initialize delivery urgency of node
        self.delivery_urgency = 0
        # Initialize neighbors of node
        self.neighbors = {}