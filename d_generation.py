'''
Author: Marco Giberna
Email: gibimarco@gmail.com
CPU model: Intel(R) Core(TM) i5-5350U CPU @ 1.80GHz
Thread count: 2 physical cores, 4 logical processors, using up to 4 threads
python3.9.9
'''
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