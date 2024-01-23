import graph as gp
import streamlit as st
import pandas as pd
import numpy as np

def main():
    # Create UI sidbar
    sidebar = st.sidebar
    
    # Create tabs in sidebar
    tab1, tab2, tab3 = sidebar.tabs(['Graph', 'A*', 'Ant Colony'])

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

    # Generate plot from graph
    plot = st.session_state.graph.plot_graph()
         
    # Display plot on streamlit
    st.pyplot(plot) 

    # Graph tab
    with tab2:
        # Tab header
        st.header('A* Search')

        if 'graph' in st.session_state:
            goal = st.selectbox(label='Select goal', options=[node.name for node in st.session_state.graph.nodes])

if __name__ == '__main__':
    main()