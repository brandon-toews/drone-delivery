import random

import graph as gp
import drone as dr
import a_star as ar


# Class for running multiple drones with genetic algorithm for multiple deliveries
def print_drone_paths(individual):
    for drone in individual:
        print(f'Drone {drone.name} Path: {[node.name for node in drone.locations]}')


class Drones:

    def __init__(self, name, graph: gp.Graph, initial_state: gp.Node, goals: [gp.Node], num_drones, crossover_selection, mutation_rate):
        self.costs = dict()
        self.name = name
        self.state_space = graph
        self.initial_state = initial_state
        self.current_state = self.initial_state
        self.goal_states = goals
        self.costs[initial_state] = {}
        self.costs = {goal: {} for goal in self.goal_states}
        self.goal_cost_range = {}
        self.calculate_travel_times()
        self.num_drones = num_drones
        self.pop_size = 100
        self.population: [dr.Drone] = []
        self.mutation_rate = mutation_rate
        self.crossover_type = crossover_selection
        # Dictionary of heuristic functions
        self.crossover_function = {
            'Uniform': self.uniform_crossover,
            'One-Point': self.one_point_crossover,
            'Heuristic': self.heuristic_crossover
        }

    # Method to calculate the travel times and paths between all goal states
    # and store them in a dictionary for future reference
    def calculate_travel_times(self):
        hub_to_goals = {}
        # Iterate through all goal states
        for goal in self.goal_states:
            # Create A* agent for the initial state and goal
            astar_agent = ar.Agent('A*',
                                   self.state_space,
                                   self.initial_state,
                                   goal,
                                   'Euclidean + Traffic Aware')
            # Perform A* search of the initial state and goal
            path, travel_time = astar_agent.astar_search()
            # Add the travel time to the costs dictionary
            hub_to_goals[goal] = (path, travel_time)
            # Iterate through all other goal states
            for other_goal in self.goal_states:
                # If the goal is not the same as the other goal
                if not goal == other_goal:
                    # Create A* agent for the goal and other goal
                    astar_agent = ar.Agent('A*',
                                           self.state_space,
                                           goal,
                                           other_goal,
                                           'Euclidean + Traffic Aware')
                    # Perform A* search of the goal and other goal
                    path, travel_time = astar_agent.astar_search()
                    # Add the travel time to the costs dictionary
                    self.costs[goal][other_goal] = (path, travel_time)

        # Add the travel times from the hub to the goals to the costs dictionary
        self.costs[self.initial_state] = hub_to_goals

        for key in self.costs:
            max_time = max(self.costs[key][key2][1] for key2 in self.costs[key])
            min_time = min(self.costs[key][key2][1] for key2 in self.costs[key])
            self.goal_cost_range[key] = (min_time, max_time)


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
        #max_locations = max(len(drone.locations) for drone in individual)
        #min_locations = min(len(drone.locations) for drone in individual)
        #balance_penalty = max_locations - min_locations
        #self.current_state = self.initial_state
        for drone in individual:
            cumulative_time = 0
            cumulative_utility_cost = 0
            #print(len(drone.locations))
            if len(drone.locations) > 1:
                for i in range(len(drone.locations)):
                    #print(drone.locations[i].name, drone.locations[i+1].name)
                    travel_time = self.costs[self.current_state][drone.locations[i]][1]
                    for node in self.costs[self.current_state][drone.locations[i]][0]:
                        drone.path.append(node)
                    #print(self.current_state.name, drone.locations[i].name, travel_time)
                    # Add travel time to cumulative time
                    cumulative_time += travel_time
                    #print(f'Cumulative Time: {cumulative_time}')
                    # Calculate utility cost for first goal based on travel time and delivery urgency
                    utility_cost = cumulative_time * drone.locations[i].delivery_urgency/10
                    #print(f'Utility Cost: {utility_cost}')
                    # Add utility cost to cumulative utility cost
                    cumulative_utility_cost += utility_cost
                    #print(f'Cumulative Utility Cost: {cumulative_utility_cost}')
                    # Update current state to goal
                    self.current_state = drone.locations[i]
            elif len(drone.locations) == 1:
                # Perform A* search of first goal
                travel_time = self.costs[self.current_state][drone.locations[0]][1]
                #print(self.current_state.name, drone.locations[0].name, travel_time)
                # Add travel time to cumulative time
                cumulative_time += travel_time
                #print(f'Cumulative Time: {cumulative_time}')
                # Calculate utility cost for first goal based on travel time and delivery urgency
                utility_cost = cumulative_time * drone.locations[0].delivery_urgency/10
                #print(f'Utility Cost: {utility_cost}')
                # Add utility cost to cumulative utility cost
                cumulative_utility_cost += utility_cost
                #print(f'Cumulative Utility Cost: {cumulative_utility_cost}')
                # Update current state to goal
                self.current_state = drone.locations[0]

            # Store the total cost of the drone
            drone.cost = cumulative_time + cumulative_utility_cost
            #print(f'Drone {drone.name} Cost: {drone.cost}')
            # Add cumulative time and cumulative utility cost to total time and total utility cost
            total_time += cumulative_time
            # Update total utility cost
            total_utility_cost += cumulative_utility_cost
            # Update current state to initial state
            self.current_state = self.initial_state

        # Return the fitness of the individual
        return total_time + total_utility_cost # * balance_penalty

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
            child1, child2 = self.crossover_function[self.crossover_type](parent1, parent2)
            # Add children to the list of children
            children.append(child1)
            children.append(child2)

        # Perform crossover on the last parent and the first parent
        child1, child2 = self.crossover_function[self.crossover_type](parents[0], parents[-1])

        # Add last two children to the list of children
        children.append(child1)
        children.append(child2)

        return children

    # Method to perform uniform crossover
    def uniform_crossover(self, parent1, parent2):
        # Create children
        child1 = [dr.Drone(i) for i in range(self.num_drones)]
        child1_assigned_goals = set()
        child2 = [dr.Drone(i) for i in range(self.num_drones)]
        child2_assigned_goals = set()

        for drone in range(self.num_drones):
            for goal in parent1[drone].locations:
                if random.random() < 0.5 and goal not in child1_assigned_goals:
                    child1[drone].locations.append(goal)
                    child1_assigned_goals.add(goal)
                elif goal not in child2_assigned_goals:
                    child2[drone].locations.append(goal)
                    child2_assigned_goals.add(goal)

            for goal in parent2[drone].locations:
                if random.random() < 0.5 and goal not in child2_assigned_goals:
                    child2[drone].locations.append(goal)
                    child2_assigned_goals.add(goal)
                elif goal not in child1_assigned_goals:
                    child1[drone].locations.append(goal)
                    child1_assigned_goals.add(goal)

        for goal in self.goal_states:
            if goal not in child1_assigned_goals:
                drone = random.randint(0, self.num_drones - 1)
                child1[drone].locations.append(goal)
                child1_assigned_goals.add(goal)

        for goal in self.goal_states:
            if goal not in child2_assigned_goals:
                drone = random.randint(0, self.num_drones - 1)
                child2[drone].locations.append(goal)
                child2_assigned_goals.add(goal)

        return self.mutate(child1), self.mutate(child2)

    # Method to perform one point crossover
    def one_point_crossover(self, parent1, parent2):
        '''print('Parent 1')
        print_drone_paths(parent1)
        print('Parent 2')
        print_drone_paths(parent2)'''

        # Create children
        child1 = [dr.Drone(i) for i in range(self.num_drones)]
        child1_assigned_goals = set()
        child2 = [dr.Drone(i) for i in range(self.num_drones)]
        child2_assigned_goals = set()

        if self.num_drones > 1:
            if self.num_drones == 2:
                crossover_point = 1
            else:
                crossover_point = random.randint(1, self.num_drones - 1)

            for drone in range(self.num_drones):
                if drone < crossover_point:
                    for goal in parent1[drone].locations:
                        if goal not in child1_assigned_goals:
                            child1[drone].locations.append(goal)
                            child1_assigned_goals.add(goal)
                    for goal in parent2[drone].locations:
                        if goal not in child2_assigned_goals:
                            child2[drone].locations.append(goal)
                            child2_assigned_goals.add(goal)
                else:
                    for goal in parent2[drone].locations:
                        if goal not in child1_assigned_goals:
                            child1[drone].locations.append(goal)
                            child1_assigned_goals.add(goal)
                    for goal in parent1[drone].locations:
                        if goal not in child2_assigned_goals:
                            child2[drone].locations.append(goal)
                            child2_assigned_goals.add(goal)

            for goal in self.goal_states:
                if goal not in child1_assigned_goals:
                    drone = random.randint(0, self.num_drones - 1)
                    child1[drone].locations.append(goal)
                    child1_assigned_goals.add(goal)

            for goal in self.goal_states:
                if goal not in child2_assigned_goals:
                    drone = random.randint(0, self.num_drones - 1)
                    child2[drone].locations.append(goal)
                    child2_assigned_goals.add(goal)

            '''print('Child 1')
            print_drone_paths(child1)
            print('Child 2')
            print_drone_paths(child2)'''


        return self.mutate(child1), self.mutate(child2)

    # Method to perform heuristic crossover
    def heuristic_crossover(self, parent1, parent2):
        # Create children
        child1 = [dr.Drone(i) for i in range(self.num_drones)]
        child1_assigned_goals = set()
        child2 = [dr.Drone(i) for i in range(self.num_drones)]
        child2_assigned_goals = set()

        for drone in range(self.num_drones):
            for i in range(len(parent1[drone].locations)):
                if parent1[drone].locations[i] not in child1_assigned_goals:
                    end = len(child1[drone].locations) - 1
                    if end < 0:
                        child1[drone].locations.append(parent1[drone].locations[i])
                        child1_assigned_goals.add(parent1[drone].locations[i])
                    else:
                        #end = len(child2[drone].locations)-1
                        #if self.is_high_quality(parent1[drone].locations[i], child1[drone].locations[end]):
                        if random.random() < self.is_high_quality(parent1[drone].locations[i], child1[drone].locations[end]):
                            child1[drone].locations.append(parent1[drone].locations[i])
                            child1_assigned_goals.add(parent1[drone].locations[i])
                        elif parent1[drone].locations[i] not in child2_assigned_goals:
                            child2[drone].locations.append(parent1[drone].locations[i])
                            child2_assigned_goals.add(parent1[drone].locations[i])

            for i in range(len(parent2[drone].locations)):
                if parent2[drone].locations[i] not in child2_assigned_goals:
                    end = len(child2[drone].locations)-1
                    if end < 0:
                        child2[drone].locations.append(parent2[drone].locations[i])
                        child2_assigned_goals.add(parent2[drone].locations[i])
                    else:
                        #if self.is_high_quality(parent2[drone].locations[i], child2[drone].locations[end]):
                        if random.random() < self.is_high_quality(parent2[drone].locations[i], child2[drone].locations[end]):
                            child2[drone].locations.append(parent2[drone].locations[i])
                            child2_assigned_goals.add(parent2[drone].locations[i])
                        elif parent2[drone].locations[i] not in child1_assigned_goals:
                            child1[drone].locations.append(parent2[drone].locations[i])
                            child1_assigned_goals.add(parent2[drone].locations[i])

        for goal in self.goal_states:
            if goal not in child1_assigned_goals:
                drone = random.randint(0, self.num_drones - 1)
                child1[drone].locations.append(goal)
                child1_assigned_goals.add(goal)

        for goal in self.goal_states:
            if goal not in child2_assigned_goals:
                drone = random.randint(0, self.num_drones - 1)
                child2[drone].locations.append(goal)
                child2_assigned_goals.add(goal)

        return self.mutate(child1), self.mutate(child2)

    def is_high_quality(self, goal1, goal2):
        #print(goal1.name, goal2.name)
        cost = self.costs[goal1][goal2][1]
        min_cost, max_cost = self.goal_cost_range[goal1]
        where = (cost - min_cost) / (max_cost - min_cost)
        return 1-where

    # Method to mutate individuals
    def new_mutate(self, individual):
        # 2% chance to mutate
        if random.random() < self.mutation_rate:
            for _ in range(2):
                # Select the drone with the most locations
                drone_most = max(individual, key=lambda drone: len(drone.locations))
                # Select the drone with the least locations
                drone_least = min(individual, key=lambda drone: len(drone.locations))
                # Select a random goal from the drone with the most locations
                goal = random.randint(0, len(drone_most.locations) - 1)
                # Remove the goal from the drone with the most locations
                location = drone_most.locations.pop(goal)
                # Add the goal to the drone with the least locations
                drone_least.locations.append(location)

        return individual

    # Method to mutate individuals
    def mutate(self, individual):
        # 2% chance to mutate
        if random.random() < self.mutation_rate:
            for _ in range(2):
                # Select a random drone
                drone = random.randint(0, len(individual)-1)
                # Ensure that the drone has goals
                while len(individual[drone].locations) == 0:
                    # Select a random drone
                    drone = random.randint(0, len(individual)-1)

                # If the drone has only one goal
                if len(individual[drone].locations) == 1:
                    # Select the only goal
                    goal = 0
                # If the drone has more than one goal
                else:
                    # Select a random goal
                    goal = random.randint(0, len(individual[drone].locations)-1)

                # Remove the goal from the drone
                location = individual[drone].locations.pop(goal)
                # Select drones with no goals
                empty_drones = [d for d in individual if len(d.locations) == 0]

                # If there are drones with no goals
                if empty_drones:
                    # Select a random drone with no goals
                    drone = random.choice(empty_drones)
                    # Add the goal to the drone
                    drone.locations.append(location)
                # If there are no drones with no goals
                else:
                    # Select a random drone to add the goal to
                    individual[random.randint(0, len(individual)-1)].locations.append(location)
            # Swap goals
            #individual = self.swap_goals(individual)

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

