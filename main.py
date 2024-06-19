import random
import graph as gp
import streamlit as st
import a_star as ar
import gen_alg as ga
import numpy as np


# Main function
def main():
    # Create UI sidebar
    sidebar = st.sidebar
    
    # Create tabs in sidebar
    tab1, tab2, tab3 = sidebar.tabs(['Graph', 'A*', 'Genetic Algorithm'])

    # Initialize A* variables
    goal = None
    astar_exploration = None
    astar_best_path = None

    # Initialize Genetic Algorithm variables
    ga_paths = None

    # Sidebar header
    with sidebar:
        
        # Graph tab
        with tab1:
            # Tab header
            st.header('Create Graph')

            # Anytime sliders are changed entire script runs again creating a new graph
            # with changed slider values
            graph_width = st.slider(label='Width', min_value=50, max_value=1000, value=250, step=5)
            graph_height = st.slider(label='Height', min_value=50, max_value=1000, value=250, step=5)
            num_nodes = st.slider(label='Number of Nodes', min_value=5, max_value=60, value=27)
            
            # Anytime button is pushed entire script runs again creating a new graph
            # with the same values but different configuration
            create_graph_button = st.button(label='Reset Graph')

    # Check if any of the slider values have changed
    sliders_changed = (
        'graph_width' not in st.session_state or st.session_state.graph_width != graph_width or
        'graph_height' not in st.session_state or st.session_state.graph_height != graph_height or
        'num_nodes' not in st.session_state or st.session_state.num_nodes != num_nodes
    )

    # Check if the create graph button was pressed or if the graph is not in the session state
    if create_graph_button or 'graph' not in st.session_state or sliders_changed:

        # Create graph
        graph = gp.Graph(graph_width, graph_height, num_nodes)

        # Save the graph and slider values in the session state
        st.session_state.graph = graph
        st.session_state.graph_width = graph_width
        st.session_state.graph_height = graph_height
        st.session_state.num_nodes = num_nodes

    # Check if the A* goal has changed
    goal_change = 'goal' not in st.session_state or st.session_state.goal != goal

    # If the A* goal has changed, update the goal in the session state
    if goal_change:
        st.session_state.goal = goal

    # Check if the genetic algorithm goals have changed
    goals_change = 'goals' not in st.session_state  # or st.session_state.goals != goals

    # If the genetic algorithm goals have changed, update the goals in the session state
    if goals_change:
        st.session_state.goals = []

    # A* tab
    with tab2:
        # Tab header
        st.header('A* Search')

        # Tell user source node
        st.text_input(
            "Source node:",
            disabled=True,
            placeholder='Hub',
        )

        # Select destination node
        if 'graph' in st.session_state:
            goal = st.selectbox(label='Select destination node:',
                                options=[key for key in st.session_state.graph.nodes if not key == 'Hub'])

        # Calculate euclidean distance from source to destination
        astar_euc_dist = gp.euc_dist(st.session_state.graph.nodes['Hub'].pos,
                                     st.session_state.graph.nodes[goal].pos)

        # Tell user source node
        st.text_input(
            "Euclidean distance from source to destination:",
            disabled=True,
            placeholder=astar_euc_dist,
        )

        # Select heuristic type
        heuristic_type = st.selectbox('Heuristic Type', ['None', 'Euclidean', 'Euclidean + Traffic Aware'], index=2)

        # Perform A* search button
        astar_search_button = st.button('Perform A* Search')

        # If the A* search button is pressed and the graph and goal are in the session state
        if astar_search_button and 'graph' in st.session_state and goal is not None:
            # Reset delivery urgency of all nodes to 0 if genetic algorithm was previously run
            for key in st.session_state.goals:
                st.session_state.graph.nodes[key].delivery_urgency = 0

            # Reset goals if genetic algorithm was previously run
            st.session_state.goals = []
            # Save goal in session state
            st.session_state.goal = goal
            # Create agent object
            astar_agent = ar.Agent('A*',
                                   st.session_state.graph,
                                   st.session_state.graph.nodes['Hub'],
                                   st.session_state.graph.nodes[goal],
                                   heuristic_type)

            # Perform A* search
            astar_path, astar_cost = astar_agent.astar_search()

            # Display path on streamlit
            st.write(f'**Path:** {gp.retrieve_path_names(astar_path)}')
            # Display path cost on streamlit
            if astar_cost is not None:
                st.write(f'**Path cost:** {astar_cost}min')

            # Save exploration and best path
            astar_exploration = astar_agent.explored_nodes()
            astar_best_path = astar_path

    # Genetic Algorithm tab
    with tab3:
        # Tab header
        st.header('Genetic Algorithm')

        # Number of drones slider
        num_drones = st.slider(label='Number of Drones', min_value=1, max_value=10, value=1)
        # Crossover type select box
        crossover_type = st.selectbox('Crossover Type', ['Uniform', 'One-Point', 'Heuristic'], index=2)
        # Mutation rate slider
        mutation_rate = st.slider(label='Mutation Rate %', min_value=0, max_value=100, value=2, step=1)

        # Tell user source node
        st.text_input(
            "Source:",
            disabled=True,
            placeholder='Hub',
        )

        # Generate goals button
        generate_goals_button = st.button('Randomly Select Destination Nodes')

        # randomly select 10 nodes as goals
        if generate_goals_button:
            random_goals = np.random.choice([key for key in st.session_state.graph.nodes if not key == 'Hub'], 10,
                                            replace=False)
            random_goals = list(random_goals)
            st.session_state.goals = list(random_goals)
            for key in st.session_state.goals:
                st.session_state.graph.nodes[key].delivery_urgency = random.randint(1, 5)

        # Display goals on streamlit
        st.write(f'**Goals:** {st.session_state.goals}')

        # Perform Genetic Algorithm button
        gen_alg_button = st.button('Perform Genetic Algorithm')

        # If the Genetic Algorithm button is pressed and the graph and goals are in the session state
        if gen_alg_button:
            # Create drones object
            drones = ga.Drones('Drones', st.session_state.graph, st.session_state.graph.nodes['Hub'],
                               [st.session_state.graph.nodes[goal] for goal in st.session_state.goals],
                               num_drones, crossover_type, mutation_rate/100)
            # Perform Genetic Algorithm
            ga_paths, cost = drones.genetic_alg()
            # Display drone paths on streamlit
            for drone in ga_paths:
                st.write(f'**Drone {drone.name} '
                         f'Path:** {[node.name for node in drone.locations]} '
                         f'**Cost:** {drone.cost}')

            # Display total cost on streamlit
            st.write(f'**Total Cost:** {cost}')

    # If A* search button is pressed, generate plot from graph with exploration and best path
    if astar_search_button:
        # Generate plot from graph
        plot = st.session_state.graph.plot_graph(st.session_state.goal, astar_exploration, astar_best_path)

        # Display plot on streamlit
        st.pyplot(plot)

    # If Genetic Algorithm button is pressed, generate plot from graph with drone paths
    elif generate_goals_button or gen_alg_button:
        # Generate plot from graph
        plot = st.session_state.graph.plot_graph(st.session_state.goals, None, None, ga_paths)

        # Display plot on streamlit
        st.pyplot(plot)

    # If neither A* search button nor Genetic Algorithm button is pressed, generate plot from graph
    else:
        # Generate plot from graph
        plot = st.session_state.graph.plot_graph()

        # Display plot on streamlit
        st.pyplot(plot)


# Run main function
if __name__ == '__main__':
    main()