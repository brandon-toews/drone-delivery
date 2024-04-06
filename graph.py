import random
import math
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.cm as cm
from matplotlib.colors import Normalize


default_names = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z'
    ]


# Graph class
class Graph:
    def __init__(self, width, height, num_nodes, input_names=None):
        if input_names is None:
            input_names = default_names
        self.width = width
        self.height = height
        self.num_nodes = num_nodes
        self.names = input_names
        # Initialize node list
        self.nodes = {}

        self.create_nodes()
        self.connect_nodes()

    # Create initial nodes specified at instantiation of graph
    def create_nodes(self):
        # Create Hub Node
        hub_pos = (int(self.width/2), int(self.height/2))
        self.nodes['Hub'] = Node('Hub', hub_pos)

        min_distance = (self.width+self.height/2)/10

        max_attempts = 1000  # Maximum number of attempts to find a new position

        # Create rest of specified nodes in graph
        if self.num_nodes < 28:
            # Loop through number of nodes specified
            for i in range(self.num_nodes-1):

                # Generate random position for node
                pos = None
                for attempt in range(max_attempts):
                    pos = (random.randint(0, self.width), random.randint(0, self.height))
                    if all(euc_dist(node.pos, pos) >= min_distance for node in self.nodes.values()):
                        # Position is far enough from all nodes, break the loop
                        break
                # Add node to list
                self.nodes[self.names[i]] = Node(self.names[i], pos)
        else:
            for i in range(len(self.names)):
                for j in range(len(self.names)):
                    # stop creating nodes when specified amount is reached
                    if len(self.nodes) == self.num_nodes:
                        break

                    # Generate random position for node
                    pos = None

                    for attempt in range(max_attempts):
                        pos = (random.randint(0, self.width), random.randint(0, self.height))
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
                    node.neighbors[other_node] = [distance, random.randint(1, 5)]

            # if no nodes were close enough to connect to than connect to 
            # the closest node
            if not len(node.neighbors):
                # Add node to neighbors dict with distance 
                # and randomly generated traffic for connection
                node.neighbors[next_closest_node] = [next_closest_distance, random.randint(1, 5)]

    # Function to plot graph with or without selected nodes
    def plot_graph(self, selected_node_names=None):
        f = plt.figure(figsize=(11, 10))

        # Traffic color dictionary
        colors = {1: "green", 2: "limegreen", 3: "gold",
                  4: "darkorange", 5: "red"}

        # Plot connections
        for node in self.nodes.values():
            for neighbor in node.neighbors:

                plt.plot(
                    [node.pos[0], neighbor.pos[0]],
                    [node.pos[1], neighbor.pos[1]],
                    color=colors[node.neighbors[neighbor][1]],
                    linewidth=4)

                # Calculate midpoint
                mid_x = (node.pos[0] + neighbor.pos[0]) / 2
                mid_y = (node.pos[1] + neighbor.pos[1]) / 2

                # Display distance
                plt.text(mid_x, mid_y, str(node.neighbors[neighbor][0]))  # data[0] is the distance

        # Plot nodes
        for node in self.nodes.values():
            # Place plots
            if node.name == 'Hub':
                plt.plot(node.pos[0], node.pos[1], 'o', markersize=25, color='blue')
            else:
                plt.plot(node.pos[0], node.pos[1], 'o', markersize=20, color='magenta')
            # Display name of node and center the text
            plt.text(node.pos[0], node.pos[1], node.name, ha='center', va='center', color='white', weight='semibold')

        if selected_node_names is not None:
            # Select nodes to plot
            selected_nodes = self.select_nodes(selected_node_names)

            for node in selected_nodes:
                plt.plot(node.pos[0], node.pos[1], 'o', markersize=20, color='cyan')
                plt.text(node.pos[0], node.pos[1], node.name, ha='center', va='center',
                         color='white', weight='semibold')

        # Produce patches for plot legend from traffic colors dict
        patches = [mpatches.Patch(color=colors[key], label=key) for key in colors]

        # Plot legend
        f.legend(handles=patches, title="Traffic Levels", bbox_to_anchor=(0.9, 0.9), loc='upper left')

        return f

    # Function to return selected nodes
    def select_nodes(self, selected_names):
        selected_nodes = []
        for node in self.nodes.values():
            if node.name in selected_names:
                selected_nodes.append(node)
        return selected_nodes


# Euclidean distance function
def euc_dist(a, b):
    return math.sqrt(
        math.pow(a[0]-b[0], 2) +
        math.pow(a[1]-b[1], 2)
    )
        

class Node:
    def __init__(self, name, pos):
        self.name = name
        self.pos = pos
        self.neighbors = {}
