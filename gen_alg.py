import random

import graph as gp
import drone as dr
import a_star as ar


# Class for running multiple drones with genetic algorithm for multiple deliveries
def print_drone_paths(individual):
    for drone in individual:
        print(f'Drone {drone.name} Path: {[node.name for node in drone.locations]}')


class Drones:

    def __init__(self, name, graph: gp.Graph, initial_state: gp.Node, goals: [gp.Node], num_drones, mutation_rate):
        self.name = name
        self.state_space = graph
        self.initial_state = initial_state
        self.current_state = self.initial_state
        self.goal_states = goals
        self.num_drones = num_drones
        self.pop_size = 100
        self.population: [dr.Drone] = []
        self.mutation_rate = mutation_rate

    # Method to create the initial population
    def create_population(self):
        # Create a population of size pop_size
        for _ in range(self.pop_size):
            # Create a fleet of drones
            individual = [dr.Drone(n) for n in range(self.num_drones)]
            # Shuffle the goals
            shuffled_goals = random.sample(self.goal_states, len(self.goal_states))
            # Assign each drone a roughly equal portion of the goals
            for i, drone in enumerate(individual):
                start = i * len(shuffled_goals) // self.num_drones
                end = (i + 1) * len(shuffled_goals) // self.num_drones
                drone.locations.extend(shuffled_goals[start:end])
            print_drone_paths(individual)
            # Add the fleet to the population
            self.population.append(individual)

    # Method to select individuals for breeding
    def select_individuals(self):
        # Sort the population by fitness
        parents = sorted(self.population, key=self.fitness_function)
        # Select the top half of the population as parents
        return parents[:self.pop_size // 2]

    # Method to calculate the fitness of an individual
    def fitness_function(self, individual):
        total_time = 0
        total_utility_cost = 0
        self.current_state = self.initial_state
        for drone in individual:
            cumulative_time = 0
            cumulative_utility_cost = 0
            #print(len(drone.locations))
            if len(drone.locations):
                for i in range(len(drone.locations)-1):
                    #print(drone.locations[i].name, drone.locations[i+1].name)
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

            # Store the total cost of the drone
            drone.cost = cumulative_time + cumulative_utility_cost
            # Add cumulative time and cumulative utility cost to total time and total utility cost
            total_time += cumulative_time
            total_utility_cost += cumulative_utility_cost
            self.current_state = self.initial_state

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
        # Get the length of the parents
        len_parent = len(parent1)
        # Create a list to store crossover points
        crossover_points_drone = []
        # Select a random index for crossover at the drone level
        while not crossover_points_drone:
            # Iterate through the drones
            for i in range(0, len(parent1)):
                # 50% chance to include the drone
                if random.random() < 0.5:
                    # Add the drone index to the list of crossover points
                    crossover_points_drone.append(i)

        # Create children
        child1 = [dr.Drone(i) for i in range(len_parent)]
        child2 = [dr.Drone(i) for i in range(len_parent)]
        '''# Add drones that are not in the crossover points to the children as is
        for drone in [i for i in range(len_parent) if i not in crossover_points_drone]:
            child1[drone] = parent1[drone]
            child2[drone] = parent2[drone]'''

        # Perform crossover at the node level on the drones in the drone crossover points
        for drone in crossover_points_drone:
            parent1_len_drone = len(parent1[drone].locations)
            parent2_len_drone = len(parent2[drone].locations)

            # Get the length of the drone's locations
            len_drone = 0

            # Select the length with the least locations
            if parent1_len_drone < parent2_len_drone:
                len_drone = parent1_len_drone
            else:
                len_drone = parent2_len_drone

            if len_drone > 1:
                # Select a random index for crossover at the node level
                crossover_point_node = random.randint(0, len_drone - 1)
            else:
                crossover_point_node = 0

            child1[drone] = dr.Drone(drone)
            child2[drone] = dr.Drone(drone)

            # Fill the first part of the crossover
            for i in range(crossover_point_node):
                child1[drone].locations.append(parent1[drone].locations[i])
                child2[drone].locations.append(parent2[drone].locations[i])

            # Fill the second part of the crossover for child1
            # only up to the length drone that is contributing its locations in parent2
            for i in range(crossover_point_node, parent2_len_drone):
                if parent2[drone].locations[i] not in [loc for d in child1 for loc in d.locations]:
                    child1[drone].locations.append(parent2[drone].locations[i])

            # Fill the second part of the crossover for child2
            # only up to the length drone that is contributing its locations in parent1
            for i in range(crossover_point_node, parent1_len_drone):
                if parent1[drone].locations[i] not in [loc for d in child2 for loc in d.locations]:
                    child2[drone].locations.append(parent1[drone].locations[i])

        for drone in range(len(parent2)):
            for goal in parent2[drone].locations:
                if goal not in [loc for d in child1 for loc in d.locations]:
                    child1[drone].locations.append(goal)

        for drone in range(len(parent1)):
            for goal in parent1[drone].locations:
                if goal not in [loc for d in child2 for loc in d.locations]:
                    child2[drone].locations.append(goal)

        print_drone_paths(child1)
        print_drone_paths(child2)

        return self.mutate(child1), self.mutate(child2)

    # Method to mutate individuals
    def mutate(self, individual):
        # 2% chance to mutate
        if random.random() < self.mutation_rate:
            # Swap goals
            individual = self.swap_goals(individual)

        return individual

    # Method to swap goals
    def swap_goals(self, individual):
        #print_drone_paths(individual)
        print('Mutating')
        # Select two random drones
        drone1, drone2 = (
            random.randint(0, len(individual)-1), random.randint(0, len(individual)-1))

        # Select random goal in drone1
        if len(individual[drone1].locations) == 1:
            goal1 = 0
        elif len(individual[drone1].locations) == 0:
            goal1 = None
        else:
            goal1 = random.randint(0, len(individual[drone1].locations)-1)

        if len(individual[drone2].locations) == 1:
            goal2 = 0
        elif len(individual[drone2].locations) == 0:
            goal2 = None

        else:
            goal2 = random.randint(0, len(individual[drone2].locations)-1)

        '''# Ensure that the goals are not the same
        while individual[drone1].locations[goal1] == individual[drone2].locations[goal2]:
            goal1 = random.randint(0, len(individual[drone1].locations)-1)
            goal2 = random.randint(0, len(individual[drone2].locations)-1)'''

        if goal1 is None:
            if goal2 is not None:
                individual[drone1].locations.append(individual[drone2].locations[goal2])
        elif goal2 is None:
            individual[drone2].locations.append(individual[drone1].locations[goal1])
        else:
            # Swap the goals
            individual[drone1].locations[goal1], individual[drone2].locations[goal2] = (
                individual[drone2].locations[goal2], individual[drone1].locations[goal1])

        #print_drone_paths(individual)
        return individual

    # Method to run the genetic algorithm
    def genetic_alg(self):
        self.create_population()

        best_overall_solution = (self.population[0], float('inf'))

        for i in range(100):
            #print(self.current_state.name)
            parents = self.select_individuals()
            best_solution = self.fitness_function(parents[0])
            if best_solution < best_overall_solution[1]:
                best_overall_solution = (parents[0], best_solution)
            # Print best individual's fitness
            print(f'Best Solution for Generation {i}: {best_solution}')
            print_drone_paths(parents[0])
            self.population = self.breed(parents)

        print(f'Best Overall Solution: {best_overall_solution[1]}')

        return best_overall_solution

