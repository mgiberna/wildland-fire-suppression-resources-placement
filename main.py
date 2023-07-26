import gurobipy as gb #v10.0.2
import numpy as np
import networkx as nx
import time
import pickle

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    with open('dataset_optimised_6resources.pickle', 'rb') as file:
        dataset = pickle.load(file)
    for dict in dataset:
        print(dict)
