import random
import math
import sys
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize

names = [
    'A','B','C','D','E','F','G','H','I','J',
    'K','L','M','N','O','P','Q','R','S','T',
    'U','V','W','X','Y','Z'
    ]

class Graph:
    def __init__(self, width, height, num_nodes, names=names):
        self.width = width
        self.height = height
        self.num_nodes = num_nodes
        self.names = names

        self.create_nodes()
        self.connect_nodes()

    # Create initial nodes specified at instantiation of graph
    def create_nodes(self):

        # Intitialize node list
        self.nodes = []

        # Create Hub Node
        hub_pos = (int(self.width/2), int(self.height/2))
        self.nodes.append(Node('Hub', hub_pos))

        min_distance = self.width/10

        # Create rest of specified nodes in graph
        if self.num_nodes < 28:
            for i in range(self.num_nodes-1):
                while True:
                    pos = (random.randint(0, self.width), random.randint(0, self.height))

                    if all(math.sqrt((node.pos[0] - pos[0])**2 + (node.pos[1] - pos[1])**2) >= min_distance for node in self.nodes):
                        # Position is far enough from all nodes, break the loop
                        break
                #pos = (random.randint(0,self.width), random.randint(0,self.height))
                self.nodes.append(Node(self.names[i], pos))
        else:
            for i in range(len(self.names)):
                for j in range(len(self.names)):
                    # stop creating nodes when specified amount is reached
                    if len(self.nodes) == self.num_nodes: break

                    while True:
                        pos = (random.randint(0, self.width), random.randint(0, self.height))

                        if all(math.sqrt((node.pos[0] - pos[0])**2 + (node.pos[1] - pos[1])**2) >= min_distance for node in self.nodes):
                            # Position is far enough from all nodes, break the loop
                            break
                    #pos = (random.randint(0,self.width), random.randint(0,self.height))
                    self.nodes.append(Node(self.names[i]+self.names[j], pos))

                    

    # Create connections of nodes in graph
    def connect_nodes(self):
        # Loop through all nodes in graph
        for node in self.nodes:
            # store next closest node and distance just in case
            # there are no nodes within range to connect to
            next_closest_distance = sys.maxsize
            next_closest_node = None

            # Loop through all the other nodes in graph
            for other_node in self.nodes:
                # calculate euclidean distance of selected nodes
                distance = int(euc_dist(node.pos,other_node.pos))

                # if both nodes selected aren't the same and distance is lower
                # than closest node found so far
                if not node == other_node and distance < next_closest_distance:
                    # store closest node
                    next_closest_distance = distance
                    next_closest_node = other_node

                # if both nodes selected aren't the same and distance is close enough
                if not node == other_node and distance < int(self.width/4):
                    # Add node to neighbors dict with distance 
                    # and randomly generated traffic for connection
                    node.neighbors[other_node] = [distance,random.randint(1,5)]

            # if no nodes were close enough to connect to than connect to 
            # closest node
            if not len(node.neighbors):
                # Add node to neighbors dict with distance 
                # and randomly generated traffic for connection
                node.neighbors[next_closest_node] = [next_closest_distance,random.randint(1,5)]

    # plot graph
    def plot_graph(self):
        plt.figure(figsize=(10,10))

        colors = {1:"darkgreen", 2:"green", 3:"darkorange",
                        4:"red", 5:"darkred"}

        # Plot connections
        for node in self.nodes:
            for neighbor in node.neighbors:

                plt.plot(
                    [node.pos[0], neighbor.pos[0]], 
                    [node.pos[1], neighbor.pos[1]],
                    color=colors[node.neighbors[neighbor][1]],
                    linewidth=2)

                # Calculate midpoint
                mid_x = (node.pos[0] + neighbor.pos[0]) / 2
                mid_y = (node.pos[1] + neighbor.pos[1]) / 2

                # Display distance
                plt.text(mid_x, mid_y, str(node.neighbors[neighbor][0]), color='red')  # data[0] is the distance

        # Plot nodes
        for node in self.nodes:
            # Place plots
            plt.plot(node.pos[0], node.pos[1], 'o', markersize=20)
            # Display name of node and center the text
            plt.text(node.pos[0], node.pos[1], node.name, ha='center', va='center', color='white')
            
        plt.show()


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