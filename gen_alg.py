import random
import graph as gp
import drone as dr
import a_star as ar


# Class for running multiple drones with genetic algorithm for multiple deliveries
def print_drone_paths(individual):
    for drone in individual:
        print(f'Drone {drone.name} Path: {[node.name for node in drone.locations]}')
        print(f'Drone {drone.name} Path: {[node.name for node in drone.path]}')


# Class for running multiple drones with genetic algorithm for multiple deliveries
class Drones:
    # Initialize Drones object
    def __init__(self, name, graph: gp.Graph, initial_state: gp.Node, goals: [gp.Node], num_drones, crossover_selection, mutation_rate):
        # Set attributes
        # Dictionary to store travel times and paths between all goal states
        self.costs = dict()
        # Set name attribute
        self.name = name
        # Set state space to graph provided
        self.state_space = graph
        # Set initial state to initial state provided
        self.initial_state = initial_state
        # Set current state to initial state
        self.current_state = self.initial_state
        # Set goal states to goals provided
        self.goal_states = goals
        # Add initial state to costs dictionary
        self.costs[initial_state] = {}
        # Add goal states to costs dictionary
        self.costs = {goal: {} for goal in self.goal_states}
        # Dictionary to store the range of costs for each goal state
        self.goal_cost_range = {}
        # Calculate the travel times and paths between all goal states
        self.calculate_travel_times()
        # Set number of drones to number of drones provided
        self.num_drones = num_drones
        # Set population size to 100
        self.pop_size = 100
        # List to store the population
        self.population: [dr.Drone] = []
        # Set mutation rate to mutation rate provided
        self.mutation_rate = mutation_rate
        # Set crossover type to crossover type provided
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
        # Initialize dictionary to store all travel times and paths from the hub to each goal state
        hub_to_goals = {}
        # Iterate through all goal states
        for goal in self.goal_states:
            # Create A* agent for the initial state(Hub) and goal
            astar_agent = ar.Agent('A*',
                                   self.state_space,
                                   self.initial_state,
                                   goal,
                                   'Euclidean + Traffic Aware')
            # Perform A* search of the initial state(hub) and goal
            path, travel_time = astar_agent.astar_search()
            # Add the travel time to the hub_to_goal costs dictionary
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

        # Calculate the range of costs for each goal state
        for key in self.costs:
            # Get the maximum and minimum travel times for each goal state
            max_time = max(self.costs[key][key2][1] for key2 in self.costs[key])
            min_time = min(self.costs[key][key2][1] for key2 in self.costs[key])
            # Add the range of costs to the goal_cost_range dictionary
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
            # print_drone_paths(individual)
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
        # Initialize total time and total utility cost
        total_time = 0
        total_utility_cost = 0
        # Loop through each drone in the individual
        for drone in individual:
            # Initialize drone path, cumulative time, and cumulative utility cost
            drone.path = []
            cumulative_time = 0
            cumulative_utility_cost = 0

            # If the drone has more than one goal
            if len(drone.locations) > 1:
                # Loop through each goal in the drone
                for i in range(len(drone.locations)):
                    # Grab precalculated travel time and paths from costs dictionary from current state to goal
                    travel_time = self.costs[self.current_state][drone.locations[i]][1]
                    paths = self.costs[self.current_state][drone.locations[i]][0]
                    # For each path in the paths
                    for n in range(len(paths)):
                        # If the i which is the index of the goal is the last goal in the drone
                        if i == len(drone.locations)-1:
                            # Add the path to the drone
                            drone.path.append(paths[n])
                        # Else if the i which is the index of the goal is not the last goal in the drone
                        # and the n which is the index of the path is not the last path in the paths
                        elif n < len(paths)-1:
                            # Add the path to the drone
                            drone.path.append(paths[n])
                    # Add travel time to cumulative time
                    cumulative_time += travel_time
                    # Calculate utility cost for first goal based on travel time and delivery urgency
                    utility_cost = cumulative_time * drone.locations[i].delivery_urgency/10
                    # Add utility cost to cumulative utility cost
                    cumulative_utility_cost += utility_cost
                    # Update current state to goal
                    self.current_state = drone.locations[i]
            # If the drone has only one goal
            elif len(drone.locations) == 1:
                # Perform A* search from current state to goal
                travel_time = self.costs[self.current_state][drone.locations[0]][1]
                # For each node in the path from current state to goal
                for node in self.costs[self.current_state][drone.locations[0]][0]:
                    # Add the node to the drone path
                    drone.path.append(node)
                # Add travel time to cumulative time
                cumulative_time += travel_time
                # Calculate utility cost for first goal based on travel time and delivery urgency
                utility_cost = cumulative_time * drone.locations[0].delivery_urgency/10
                # Add utility cost to cumulative utility cost
                cumulative_utility_cost += utility_cost
                # Update current state to goal
                self.current_state = drone.locations[0]

            # Store the total cost of the drone
            drone.cost = cumulative_time + cumulative_utility_cost
            # Add cumulative time and cumulative utility cost to total time and total utility cost
            total_time += cumulative_time
            # Update total utility cost
            total_utility_cost += cumulative_utility_cost
            # Update current state to initial state
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
            # Perform crossover based selected crossover type
            child1, child2 = self.crossover_function[self.crossover_type](parent1, parent2)
            # Add children to the list of children
            children.append(child1)
            children.append(child2)

        # Perform crossover on the last parent and the first parent
        child1, child2 = self.crossover_function[self.crossover_type](parents[0], parents[-1])

        # Add last two children to the list of children
        children.append(child1)
        children.append(child2)

        # Return the list of children
        return children

    # Method to perform uniform crossover
    def uniform_crossover(self, parent1, parent2):
        # Create children
        child1 = [dr.Drone(i) for i in range(self.num_drones)]
        child2 = [dr.Drone(i) for i in range(self.num_drones)]
        # Set to store assigned goals
        child1_assigned_goals = set()
        child2_assigned_goals = set()
        # Iterate through each drone
        for drone in range(self.num_drones):
            # Iterate through each goal in the drone of parent1
            for goal in parent1[drone].locations:
                # If random number is less than 0.5 and goal is not in child1_assigned_goals
                if random.random() < 0.5 and goal not in child1_assigned_goals:
                    # Add goal to child1
                    child1[drone].locations.append(goal)
                    # Add goal to child1_assigned_goals
                    child1_assigned_goals.add(goal)
                # Else if goal is not in child2_assigned_goals
                elif goal not in child2_assigned_goals:
                    # Add goal to child2
                    child2[drone].locations.append(goal)
                    # Add goal to child2_assigned_goals
                    child2_assigned_goals.add(goal)

            # Iterate through each goal in the drone of parent2
            for goal in parent2[drone].locations:
                # If random number is less than 0.5 and goal is not in child2_assigned_goals
                if random.random() < 0.5 and goal not in child2_assigned_goals:
                    # Add goal to child2
                    child2[drone].locations.append(goal)
                    # Add goal to child2_assigned_goals
                    child2_assigned_goals.add(goal)
                # Else if goal is not in child1_assigned_goals
                elif goal not in child1_assigned_goals:
                    # Add goal to child1
                    child1[drone].locations.append(goal)
                    # Add goal to child1_assigned_goals
                    child1_assigned_goals.add(goal)

        # Add missing goals to children if any
        self.add_missing_goals(child1, child1_assigned_goals)
        self.add_missing_goals(child2, child2_assigned_goals)

        # Return mutated children
        return self.mutate(child1), self.mutate(child2)

    # Method to perform one point crossover
    def one_point_crossover(self, parent1, parent2):
        # Create children
        child1 = [dr.Drone(i) for i in range(self.num_drones)]
        child2 = [dr.Drone(i) for i in range(self.num_drones)]
        # Set to store assigned goals
        child1_assigned_goals = set()
        child2_assigned_goals = set()

        # If there are more than one drone perform one point crossover at the drone level
        if self.num_drones > 1:
            # If there are two drones
            if self.num_drones == 2:
                # Set crossover point to 1
                crossover_point = 1
            # If there are more than two drones
            else:
                # Set crossover point to random number between 1 and number of drones - 1
                crossover_point = random.randint(1, self.num_drones - 1)

            # Iterate through each drone
            for drone in range(self.num_drones):
                # If drone is less than crossover point
                if drone < crossover_point:
                    # Iterate through each goal in the drone of parent1
                    for goal in parent1[drone].locations:
                        # If goal is not in child1_assigned_goals
                        if goal not in child1_assigned_goals:
                            # Add goal to child1
                            child1[drone].locations.append(goal)
                            # Add goal to child1_assigned_goals
                            child1_assigned_goals.add(goal)
                    # Iterate through each goal in the drone of parent2
                    for goal in parent2[drone].locations:
                        # If goal is not in child2_assigned_goals
                        if goal not in child2_assigned_goals:
                            # Add goal to child2
                            child2[drone].locations.append(goal)
                            # Add goal to child2_assigned_goals
                            child2_assigned_goals.add(goal)
                # If drone is greater than or equal to crossover point
                else:
                    # Iterate through each goal in the drone of parent2
                    for goal in parent2[drone].locations:
                        # If goal is not in child1_assigned_goals
                        if goal not in child1_assigned_goals:
                            # Add goal to child1
                            child1[drone].locations.append(goal)
                            # Add goal to child1_assigned_goals
                            child1_assigned_goals.add(goal)
                    # Iterate through each goal in the drone of parent1
                    for goal in parent1[drone].locations:
                        # If goal is not in child2_assigned_goals
                        if goal not in child2_assigned_goals:
                            # Add goal to child2
                            child2[drone].locations.append(goal)
                            # Add goal to child2_assigned_goals
                            child2_assigned_goals.add(goal)

            # Add missing goals to children if any
            self.add_missing_goals(child1, child1_assigned_goals)
            self.add_missing_goals(child2, child2_assigned_goals)
        # If there is only one drone perform one point crossover at the goal level
        else:
            # Set crossover point to random number between 1 and number of goals - 1
            crossover_point = random.randint(1, len(parent1[0].locations) - 1)
            # Iterate through each goal in the drone of parent1
            for goal in range(len(parent1[0].locations)-2):
                # If goal is less than crossover point
                if goal < crossover_point:
                    # If goal from parent1 is not in child1_assigned_goals
                    if parent1[0].locations[goal] not in child1_assigned_goals:
                        # Add goal to child1
                        child1[0].locations.append(parent1[0].locations[goal])
                        # Add goal to child1_assigned_goals
                        child1_assigned_goals.add(parent1[0].locations[goal])
                    # If goal from parent2 is not in child2_assigned_goals
                    if parent2[0].locations[goal] not in child2_assigned_goals:
                        # Add goal to child2
                        child2[0].locations.append(parent2[0].locations[goal])
                        # Add goal to child2_assigned_goals
                        child2_assigned_goals.add(parent2[0].locations[goal])
                # If goal is greater than or equal to crossover point
                else:
                    # If goal from parent2 is not in child1_assigned_goals
                    if parent2[0].locations[goal] not in child1_assigned_goals:
                        # Add goal to child1
                        child1[0].locations.append(parent2[0].locations[goal])
                        # Add goal to child1_assigned_goals
                        child1_assigned_goals.add(parent2[0].locations[goal])
                    # If goal from parent1 is not in child2_assigned_goals
                    if parent1[0].locations[goal] not in child2_assigned_goals:
                        # Add goal to child2
                        child2[0].locations.append(parent1[0].locations[goal])
                        # Add goal to child2_assigned_goals
                        child2_assigned_goals.add(parent1[0].locations[goal])

            # Add missing goals to children if any
            self.add_missing_goals(child1, child1_assigned_goals)
            self.add_missing_goals(child2, child2_assigned_goals)

        # Return mutated children
        return self.mutate(child1), self.mutate(child2)

    # Method to perform heuristic crossover
    def heuristic_crossover(self, parent1, parent2):
        # Create children
        child1 = [dr.Drone(i) for i in range(self.num_drones)]
        child2 = [dr.Drone(i) for i in range(self.num_drones)]
        # Set to store assigned goals
        child1_assigned_goals = set()
        child2_assigned_goals = set()

        # Iterate through each drone
        for drone in range(self.num_drones):
            # Iterate through each goal in the drone of parent1
            for i in range(len(parent1[drone].locations)):
                # If goal is not in child1_assigned_goals
                if parent1[drone].locations[i] not in child1_assigned_goals:
                    # set end to the last index of the goals in the drone
                    end = len(child1[drone].locations) - 1
                    # If end is less zero this means the drone has no goals yet
                    # will check goal quality in comparison to the initial state (hub)
                    if end < 0:
                        # If goal is a high quality goal based on the heuristic it will have a higher score
                        # making it more likely to be added to the child
                        if random.random() < self.is_high_quality(self.initial_state, parent1[drone].locations[i]):
                            # Add goal to child1
                            child1[drone].locations.append(parent1[drone].locations[i])
                            # Add goal to child1_assigned_goals
                            child1_assigned_goals.add(parent1[drone].locations[i])
                    # If end is greater than or equal to zero this means the drone has goals
                    # will check goal quality in comparison to the last goal in the drone
                    else:
                        # If goal is a high quality goal based on the heuristic it will have a higher score
                        # making it more likely to be added to the child
                        if random.random() < self.is_high_quality(parent1[drone].locations[i], child1[drone].locations[end]):
                            # Add goal to child1
                            child1[drone].locations.append(parent1[drone].locations[i])
                            # Add goal to child1_assigned_goals
                            child1_assigned_goals.add(parent1[drone].locations[i])
                        # If goal is not a high quality goal based on the heuristic
                        # and goal in not in child2_assigned_goals
                        elif parent1[drone].locations[i] not in child2_assigned_goals:
                            # Add goal to child2
                            child2[drone].locations.append(parent1[drone].locations[i])
                            # Add goal to child2_assigned_goals
                            child2_assigned_goals.add(parent1[drone].locations[i])

            # Iterate through each goal in the drone of parent2
            for i in range(len(parent2[drone].locations)):
                # If goal is not in child2_assigned_goals
                if parent2[drone].locations[i] not in child2_assigned_goals:
                    # set end to the last index of the goals in the drone
                    end = len(child2[drone].locations)-1
                    # If end is less zero this means the drone has no goals yet
                    if end < 0:
                        # If goal is a high quality goal based on the heuristic it will have a higher score
                        # making it more likely to be added to the child
                        if random.random() < self.is_high_quality(self.initial_state, parent2[drone].locations[i]):
                            # Add goal to child2
                            child2[drone].locations.append(parent2[drone].locations[i])
                            # Add goal to child2_assigned_goals
                            child2_assigned_goals.add(parent2[drone].locations[i])
                    # If end is greater than or equal to zero this means the drone has goals
                    # will check goal quality in comparison to the last goal in the drone
                    else:
                        # If goal is a high quality goal based on the heuristic it will have a higher score
                        # making it more likely to be added to the child
                        if random.random() < self.is_high_quality(parent2[drone].locations[i], child2[drone].locations[end]):
                            # Add goal to child2
                            child2[drone].locations.append(parent2[drone].locations[i])
                            # Add goal to child2_assigned_goals
                            child2_assigned_goals.add(parent2[drone].locations[i])
                        # If goal is not a high quality goal based on the heuristic
                        # and goal in not in child1_assigned_goals
                        elif parent2[drone].locations[i] not in child1_assigned_goals:
                            # Add goal to child1
                            child1[drone].locations.append(parent2[drone].locations[i])
                            # Add goal to child1_assigned_goals
                            child1_assigned_goals.add(parent2[drone].locations[i])

        # Add missing goals to children if any
        self.add_missing_goals(child1, child1_assigned_goals)
        self.add_missing_goals(child2, child2_assigned_goals)

        # Return mutated children
        return self.mutate(child1), self.mutate(child2)

    # Method to calculate the quality of a goal based on the heuristic
    def is_high_quality(self, goal1, goal2):
        # Get the cost of traveling from goal1 to goal2
        cost = self.costs[goal1][goal2][1]
        # Get the minimum and maximum cost of traveling to goal1 from any other goal
        min_cost, max_cost = self.goal_cost_range[goal1]
        # Calculate the quality of the goal based on the heuristic
        # The closer the cost is to the minimum cost the higher the quality of the goal
        where = 1 - ((cost - min_cost) / (max_cost - min_cost))
        # Return the quality of the goal
        return where

    # Method to mutate individuals towards balancing the number of goals assigned to each drone
    def balance_mutate(self, individual):
        # Percentage based on mutation rate given
        if random.random() < self.mutation_rate:
            # Move a goal from one drone to another
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
        # Return the mutated individual
        return individual

    # Method to mutate individuals by moving goals from one drone to another
    def mutate(self, individual):
        # Percentage based on mutation rate given
        if random.random() < self.mutation_rate:
            # Move a goal from one drone to another
            for _ in range(2):
                # Select a random drone
                drone = random.randint(0, len(individual)-1)
                # Ensure that the selected drone has goals
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
        # Return the mutated individual
        return individual

    # Method to add missing goals to children if any
    def add_missing_goals(self, individual, assigned_goals):
        # Iterate through each goal state in a random order
        for goal in random.sample(self.goal_states, len(self.goal_states)):
            # If goal is not in child1_assigned_goals
            if goal not in assigned_goals:
                # Select a random drone
                drone = random.randint(0, self.num_drones - 1)
                # Add goal to child1
                individual[drone].locations.append(goal)
                # Add goal to child1_assigned_goals
                assigned_goals.add(goal)

    # Method to run the genetic algorithm
    def genetic_alg(self):
        # Create the initial population
        self.create_population()
        # Set the best overall solution to the first individual in the population with infinite fitness
        best_overall_solution = (self.population[0], float('inf'))

        # Loop through 100 generations
        for i in range(100):
            # Select individuals for breeding
            parents = self.select_individuals()
            # Get the fitness of the best individual
            best_solution = self.fitness_function(parents[0])
            # If the fitness of the best individual is less than the fitness of the best overall solution
            # the lowest fitness is the best solution
            if best_solution < best_overall_solution[1]:
                # Update the best overall solution
                best_overall_solution = (parents[0], best_solution)
            # Print best individual's fitness
            print(f'Best Solution for Generation {i}: {best_solution}')
            # Breed individuals to create the next generation
            self.population = self.breed(parents)

        # Return the best overall solution
        print(f'Best Overall Solution: {best_overall_solution[1]}')

        # Return the best overall solution
        return best_overall_solution