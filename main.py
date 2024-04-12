import random

import graph as gp
import streamlit as st
import a_star as ar
import gen_alg as ga
import pandas as pd
import numpy as np


def main():
    # Create UI sidebar
    sidebar = st.sidebar
    
    # Create tabs in sidebar
    tab1, tab2, tab3 = sidebar.tabs(['Graph', 'A*', 'Genetic Algorithm'])

    # Initialize A* variables
    goal = None
    astar_exploration = None
    astar_best_path = None

    # Initialize GA variables
    #goals = []

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

    if create_graph_button or 'graph' not in st.session_state or sliders_changed:

        # Create graph
        graph = gp.Graph(graph_width, graph_height, num_nodes)

        # Save the graph and slider values in the session state
        st.session_state.graph = graph
        st.session_state.graph_width = graph_width
        st.session_state.graph_height = graph_height
        st.session_state.num_nodes = num_nodes

    goal_change = 'goal' not in st.session_state or st.session_state.goal != goal

    if goal_change:
        st.session_state.goal = goal

    goals_change = 'goals' not in st.session_state #or st.session_state.goals != goals

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

        # Heuristic type
        heuristic_type = st.selectbox('Heuristic Type', ['None', 'Euclidean', 'Euclidean + Traffic Aware'], index=2)

        astar_search_button = st.button('Perform A* Search')

        # Perform A* search
        if astar_search_button and 'graph' in st.session_state and goal is not None:
            for key in st.session_state.goals:
                st.session_state.graph.nodes[key].delivery_urgency = 0
            st.session_state.goals = []
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

        num_drones = st.slider(label='Number of Drones', min_value=1, max_value=10, value=1)
        crossover_type = st.selectbox('Crossover Type', ['Uniform', 'One-Point', 'Heuristic'], index=0)
        mutation_rate = st.slider(label='Mutation Rate %', min_value=0, max_value=100, value=2, step=1)

        # Tell user source node
        ga_source = st.text_input(
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

        # Select destination node to add to goals
        #add_goal = st.selectbox(label='Add destination node:',
        #                        options=[key for key in st.session_state.graph.nodes
        #                                 if not key == 'Hub' and key not in st.session_state.goals])
        # Add selected destination node
        #add_goal_button = st.button('Add destination node')

        # Add destination node to goals
        #if add_goal_button:
        #    st.session_state.goals.append(add_goal)

        # Display goals on streamlit
        st.write(f'**Goals:** {st.session_state.goals}')

        gen_alg_button = st.button('Perform Genetic Algorithm')

        # Perform Genetic Algorithm
        if gen_alg_button:
            drones = ga.Drones('Drones', st.session_state.graph, st.session_state.graph.nodes['Hub'],
                               [st.session_state.graph.nodes[goal] for goal in st.session_state.goals],
                               num_drones, crossover_type, mutation_rate/100)
            paths, cost = drones.genetic_alg()
            for drone in paths:
                st.write(f'**Drone {drone.name} Path:** {[node.name for node in drone.locations]} **Cost:** {drone.cost}')
            st.write(f'**Total Cost:** {cost}')


    if astar_search_button:
        # Generate plot from graph
        plot = st.session_state.graph.plot_graph(st.session_state.goal, astar_exploration, astar_best_path)

        # Display plot on streamlit
        st.pyplot(plot)
    elif generate_goals_button or gen_alg_button:
        # Generate plot from graph
        plot = st.session_state.graph.plot_graph(st.session_state.goals)

        # Display plot on streamlit
        st.pyplot(plot)

    else:
        # Generate plot from graph
        plot = st.session_state.graph.plot_graph()

        # Display plot on streamlit
        st.pyplot(plot)




if __name__ == '__main__':
    main()
