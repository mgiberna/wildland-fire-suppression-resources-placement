# wildland-fire-suppression-resources-placement
Final Project for the Master Course "Mathematical Optimisation". 
Implementation of a model of mixed-integer linear programming from Mendes, A. B., & e Alvelos, F. P. (2023). Iterated local search for the placement of wildland fire suppression resources. European Journal of Operational Research, 304(3), 887-900, https://doi.org/10.1016/j.ejor.2022.04.037. 

## Required Dependencies
Apart some very common libraries, in this project networkX (v3.1) has been used. To install it:
```
!pip install networkx #v3.1
```
Other libraries used:
- gurobipy #v10.0.2
- time
- pickle
- csv
- matplotlib
- random
- pickle
- numpy
- copy
- logging

## ILS directory
It contains all the necessary to run the Iterated Local Search algorithm for the placement of wildland fire suppression resources.
See the README.md file within the folder to get further information.

## d_generation.py
Script to generate a dataset following the specification introduced in the paper. 
It contains a function to generate a single square graph, given the dimension and weights' intervals, and a function to build up a dataset containing all the created graph and other metadata.

## placement_of_wildland_fire_suppression_resources_gurobi.py
Script that solve the presented optimisation problem by using gurobi.
Running the script solves two problem given a dataset, each for a different number of resources used.
It contains a function to optimize the model and write down the solution, given a dataset (from d_generation.py) and a setup (5 or 6 resources).
Furthermore, the fire_resource_placement_model function returns a gurobi model such as presented in the paper. It takes in input the setup and a single instance from the previous dataset, along with corresponding metadata.

## main.py
This script run all the previously presented steps sequentially, in order to get the dataset, therefore the gurobi and ILS results.

