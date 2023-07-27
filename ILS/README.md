# Iterated Local Search
This repository contains all the code required to run the Iterated Local Search algorithm for Fire Resources Placement, as introduced in the paper.

## ILS.py
Implementation of ILS algorithm as presented in the paper. 
You can execute its main() to run the ILS algorithm alone.
Useful functions:
- iteratedLocalSearch(FRP, show_plot=False) (given a FRPinstance object, return the best found solution after running the ILS algorithm)
- test_ILS(dataset, n_resources, delta=50, time_horizon=28, n_replications = 5) (given a dataset - from d_generation.py - and n_resources, it runs for each instance in dataset the ILS algorithm. Then it writes a CSV file containing the solutions)

## ILS_debug.py
The same as ILS.py, but fitted for debugging purposes.

## FRPinstance.py
This class is made for creating an object "FRPinstance" to run the ILS for Fire Resources Placement.

The object attributes are:
- size
- time_horizon
- ignition_node
- graph 
- updated_graph (copy of graph, but this will be updated by changing its weights during the ILS, while graph will not)
- n_resources
- resources (list containing the available resources)
- delta
- n_resources_time (list in which each element i represents how many resources are available from the instant i)
- solution (solution[i,j,k] represent whether at time k a resources has been placed on (i,j))
- fire_arrivals (instants at which each node is burnt)
- value (current)
- OFV_CH (value after custructive heuristic)
- OFV_LS (value after first local search)
- OFV_ILS (value after iterated local search)

While the following function are implemented:
- copy(self) (returns a copy of the FRPinstance)
- add_resource(self, i,j,k) (add a resource at instant k on node i,j)
- remove_resource(self, i,j,k) (remove a resource at instant k on node i,j)
- update_fire_propagation_times(self, i,j,k, after_removing=True) (update fire propagation times after removing (True) or placing (False) a resource on node i,j at instant k)
- update_fire_arrival_times(self) (updates fire arrivals instant by running dijkstra algorithm)
- update_current_value(self) (update value by evaluating the objective function)
- reset(self) (return a resetted copy of the FRPinstance)
- get_value(self)
- get_value(self)
- get_OFV_CH(self)
- get_OFV_LS(self)
- get_OFV_ILS(self)
- show(self) (shows the graph, enlighting the burned, unburned, ignition nodes and those where a resource have been placed)