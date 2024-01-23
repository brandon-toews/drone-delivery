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

    # Create graph
    graph = gp.Graph(graph_width, graph_height, num_nodes)

    # Generate plot from graph
    plot = graph.plot_graph()

    # Display plot on streamlit
    st.pyplot(plot)

if __name__ == '__main__':
    main()