import random

import graph as gp
import drone as dr
import a_star as ar


# Class for running multiple drones with genetic algorithm for multiple deliveries
class Drones:

    def __init__(self, name, graph: gp.Graph, initial_state: gp.Node, goals: [gp.Node], num_drones):
        self.name = name
        self.state_space = graph
        self.initial_state = initial_state
        self.current_state = self.initial_state
        self.goal_states = goals
        self.num_drones = num_drones
        self.pop_size = 100
        self.population: [dr.Drone] = []

    # Method to create the initial population
    def create_population(self):
        # Create a population of size pop_size
        for i in range(self.pop_size):
            # Create a fleet of drones
            individual = [[dr.Drone(n) for n in range(self.num_drones)]]
            # Assign a random drone a random goal
            [individual[random.randint(0, self.num_drones - 1)].locations.append(goal) for goal in self.goal_states]
            # Add the fleet to the population
            self.population.append(individual)

    # Method to select individuals for breeding
    def select_individuals(self):
        # Sort the population by fitness
        parents = sorted(self.population, key=self.fitness_function, reverse=True)
        # Select the top half of the population as parents
        return parents[:self.pop_size // 2]

    # Method to calculate the fitness of an individual
    def fitness_function(self, individual):
        total_time = 0
        total_utility_cost = 0
        for drone in individual:
            cumulative_time = 0
            cumulative_utility_cost = 0
            for i in range(len(drone.locations)-1):
                # Create A* agent
                astar_agent = ar.Agent('A*',
                                       self.state_space,
                                       self.current_state,
                                       drone.locations[i],
                                       'Euclidean + Traffic Aware')
                # Perform A* search of first goal
                path, travel_time = astar_agent.astar_search()
                # Add travel time to cumulative time
                cumulative_time += travel_time
                # Calculate utility cost for first goal based on travel time and delivery urgency
                utility_cost = cumulative_time * drone.locations[i].delivery_urgency/10
                # Add utility cost to cumulative utility cost
                cumulative_utility_cost += utility_cost
                # Update current state to goal
                self.current_state = drone.locations[i]
            # Add cumulative time and cumulative utility cost to total time and total utility cost
            total_time += cumulative_time
            total_utility_cost += cumulative_utility_cost
        # Return the fitness of the individual
        return total_time + total_utility_cost

    # Method to breed individuals
    def breed(self, parents):
        # Create a list to store children
        children: [dr.Drone] = []
        # Iterate through the parents
        for i in range(0, len(parents)-1):
            # Select two parents
            parent1 = parents[i]
            parent2 = parents[i + 1]
            # Perform crossover
            child1, child2 = self.crossover(parent1, parent2)
            # Add children to the list of children
            children.append(child1)
            children.append(child2)

        # Perform crossover on the last parent and the first parent
        child1, child2 = self.crossover(parents[0], parents[-1])
        # Add last two children to the list of children
        children.append(child1)
        children.append(child2)

        return children

    # Method to perform crossover
    def crossover(self, parent1, parent2):
        # pick a random index to perform crossover for every drone
        # and swap the goals of the corresponding drones of each parent as the crossover point

        chrome1 = [
            [node0, node1, node7, node9]
            [node4]
            [node3, node5, node2]
        ]

        chrome2 = [
            [node1, node2]
            [node0, node5]
            [node4, node3]
        ]

        ch1 = [
            [node0, node2]
            [node4, node5]
            [node3, node3, node2]
        ]

        ch2 = [
            [node1, node2]
            [node4]
            [node3, node5, node0]
        ]

        par1 = node0, node1, node4, node3, node5, node2

        par2 = node1, node2, node0, node5, node4, node3

        child1 = node0, node1, node5, node4, node3, node2

        child2 = node1, node2, node4, node3, node5, node0



        # Select a random index to perform crossover
        crossover_point = random.randint(0, len(parent1))
        pass

    # Method to mutate individuals
    def mutation(self):
        pass

    def genetic_alg(self):
        self.create_population()
        parents = self.select_individuals()
        self.population = self.breed(parents)

        pass

