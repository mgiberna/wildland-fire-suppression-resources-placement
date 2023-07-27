'''
CPU model: Intel(R) Core(TM) i5-5350U CPU @ 1.80GHz
Thread count: 2 physical cores, 4 logical processors, using up to 4 threads
'''
#python3.9.9
#!pip install networkx #v3.1
import networkx as nx
#!pip install matplotlib #v3.7.2
import matplotlib.pyplot as plt
import random
import pickle


def generate_square_graph_with_rand_weights(n, north_weight_interval, south_weight_interval, east_weight_interval, west_weight_interval):
    G = nx.DiGraph()
    # add nodes
    for i in range(n):
        for j in range(n):
            node = (i, j)
            G.add_node(node)
    # add arcs
    for i in range(n):
        for j in range(n):
            node = (i, j)
            neighbors = [
                (i - 1, j),  # neighbor to the north
                (i + 1, j),  # neighbor to the south
                (i, j - 1),  # neighbor to the west
                (i, j + 1)   # neighbor to the east
            ]
            weights = [
                random.randint(north_weight_interval[0], north_weight_interval[1]),  # weight for the north direction
                random.randint(south_weight_interval[0], south_weight_interval[1]),  # weight for the south direction
                random.randint(west_weight_interval[0], west_weight_interval[1]),    # weight for the west direction
                random.randint(east_weight_interval[0], east_weight_interval[1])     # weight for the east direction
            ]
            for neighbor, weight in zip(neighbors, weights):
                if neighbor in G.nodes():
                    G.add_edge(node, neighbor, weight=weight)
    graph = nx.grid_2d_graph(n, n, create_using=nx.DiGraph())
    graph.add_edges_from(G.edges(data=True))
    return G


def generate_dataset(sizes, propagation_times):
    dataset = []
    for i in range(len(sizes)):
        ignition_node = (sizes[i]/2 - 1, sizes[i]/2 - 1)
        for j in range(len(propagation_times)):
            graph = generate_square_graph_with_rand_weights(sizes[i],
                                                            propagation_times[j][0],
                                                            propagation_times[j][1],
                                                            propagation_times[j][2],
                                                            propagation_times[j][3])
            instance = i*len(propagation_times) + j + 1
            env_dict = {"instance": instance, "size": sizes[i], "graph": graph, "ignition_node": ignition_node}
            dataset.append(env_dict)
    return dataset


if __name__ == '__main__':
    #PARAMETERS
    sizes = (6,10,20,30)
    propagation_times = (((7,9),(2,4),(4,6),(6,8)),
                         ((7,9),(1,3),(4,6),(6,8)),
                         ((7,9),(2,4),(3,5),(6,8)),
                         ((7,9),(1,3),(3,5),(6,8)),
                         ((7,9),(2,4),(4,6),(4,6)),
                         ((7,9),(1,3),(4,6),(4,6)),
                         ((7,9),(2,4),(3,5),(3,5)),
                         ((7,9),(1,3),(3,5),(3,5)))
    #generating the dataset
    dataset = generate_dataset(sizes, propagation_times)
    #dumping the dataset in a pickle
    with open('dataset.pickle', 'wb') as file:
        pickle.dump(dataset, file)
    '''
    G = dataset[0]["graph"]
    # Accessing the weight of a directed edge
    node1 = (0,0)
    node2 = (0,1)
    edge_weight = G[node1][node2]['weight']
    print(f"The weight of the directed edge ({node1}, {node2}) is: {edge_weight}")
    
    node1 = (0,1)
    node2 = (0,0)
    edge_weight = G[node1][node2]['weight']
    print(f"The weight of the directed edge ({node1}, {node2}) is: {edge_weight}")
    # Generate layout for visualization
    pos = nx.spring_layout(G)
    
    # Draw the graph
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=12, font_weight='bold')
    
    # Draw edge labels
    edge_labels = {}
    for u, v, data in G.edges(data=True):
        if (u, v) in edge_labels:
            edge_labels[(u, v)] += f"\n{data['weight']}"
        else:
            edge_labels[(u, v)] = str(data['weight'])
    
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10)
    
    
    # Show the plot
    plt.show()

    # Generate multiple directed square graphs
    graph1 = generate_square_graph(5, (1, 10), (11, 20), (21, 30), (31, 40))
    graph2 = generate_square_graph(6, (2, 12), (13, 22), (23, 32), (33, 42))
    graph3 = generate_square_graph(7, (3, 14), (15, 24), (25, 34), (35, 44))
    
    # Create a dictionary to store the graphs
    graphs = {
        'graph1': graph1,
        'graph2': graph2,
        'graph3': graph3
    }
    
    # Save the dictionary of graphs to a file
    with open('graphs_data.pickle', 'wb') as file:
        pickle.dump(graphs, file)
    
    # Load the saved graphs from the file
    with open('graphs_data.pickle', 'rb') as file:
        loaded_graphs = pickle.load(file)
    
    # Now you can work with the loaded graphs
    print(loaded_graphs['graph1'].nodes())
    print(loaded_graphs['graph2'].nodes())
    print(loaded_graphs['graph3'].nodes())
    
    graph = dataset[0]["graph"]
    
    # Set the positions of the nodes in a grid layout
    n = int(len(graph.nodes()) ** 0.5)  # Number of rows/columns in the grid
    pos = {node: (node % n, -(node // n)) for node in graph.nodes()}
    
    # Accessing the weight of a directed edge
    node1 = 0
    node2 = 1
    edge_weight = graph[node1][node2]['weight']
    print(f"The weight of the directed edge ({node1}, {node2}) is: {edge_weight}")
    
    node1 = 1
    node2 = 0
    edge_weight = graph[node1][node2]['weight']
    print(f"The weight of the directed edge ({node1}, {node2}) is: {edge_weight}")
    
    # Draw the graph
    nx.draw(graph, pos=pos, with_labels=True, node_color='skyblue', node_size=800, edge_color='gray', arrows=True)
    
    # Display the graph
    plt.show()
    '''