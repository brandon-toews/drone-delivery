import graph as gp
import streamlit as st
import drone as dp
import pandas as pd
import numpy as np


def main():
    # Create UI sidebar
    sidebar = st.sidebar
    
    # Create tabs in sidebar
    tab1, tab2, tab3 = sidebar.tabs(['Graph', 'A*', 'Ant Colony'])

    # Initialize goal
    goal = None
    astar_exploration = None
    astar_best_path = None

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

    astar_goal_change = 'goal' not in st.session_state or st.session_state.goal != goal

    if create_graph_button or 'graph' not in st.session_state or sliders_changed:

        # Create graph
        graph = gp.Graph(graph_width, graph_height, num_nodes)

        # Save the graph and slider values in the session state
        st.session_state.graph = graph
        st.session_state.graph_width = graph_width
        st.session_state.graph_height = graph_height
        st.session_state.num_nodes = num_nodes

    if astar_goal_change:
        st.session_state.goal = goal
        if goal is not None:
            astar_euc_dist = gp.euc_dist(st.session_state.graph.nodes['Hub'].pos, st.session_state.graph.nodes[goal].pos)

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
        astar_euc_dist = gp.euc_dist(st.session_state.graph.nodes['Hub'].pos, st.session_state.graph.nodes[goal].pos)

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
            # Create drone object
            astar_drone = dp.Drone('A*',
                                   st.session_state.graph,
                                   st.session_state.graph.nodes['Hub'],
                                   st.session_state.graph.nodes[goal],
                                   heuristic_type)

            # Perform A* search
            path = astar_drone.astar_search()

            # Display path on streamlit
            st.write(f'**Path:** {gp.retrieve_path_names(path[0])}')
            # Display path cost on streamlit
            if path[1] is not None:
                st.write(f'**Path cost:** {path[1]}min')

            # Save exploration and best path
            astar_exploration = astar_drone.explored_nodes()
            astar_best_path = path[0]

    # Generate plot from graph
    plot = st.session_state.graph.plot_graph(goal, astar_exploration, astar_best_path)
         
    # Display plot on streamlit
    st.pyplot(plot)


if __name__ == '__main__':
    main()
