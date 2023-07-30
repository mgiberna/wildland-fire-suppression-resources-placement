'''
Author: Marco Giberna
Email: gibimarco@gmail.com
CPU model: Intel(R) Core(TM) i5-5350U CPU @ 1.80GHz
python3.9.9
'''
import random
import numpy as np
import networkx as nx
import pickle
from FRPinstance import FRPInstance
import copy
import time
import csv
import logging
DEBUG = False


def find_first_non_zero(arr):
    for index, element in enumerate(arr):
        if element != 0:
            return index
    return None


def evaluate(FRP):
    eps = 1 / FRP.n_resources
    fire_arrivals = nx.single_source_shortest_path_length(FRP.updated_graph, FRP.ignition_node)
    burned_nodes_at_h = sum(1 for time in fire_arrivals.values() if time < FRP.time_horizon)
    #do not count nodes where resource placed as burnt
    resource_indexes = np.nonzero(FRP.solution)
    resource_set = {(node[0], node[1]) for node in zip(resource_indexes[0], resource_indexes[1])}
    not_burned_nodes = sum(1 for node in resource_set if fire_arrivals[node] < FRP.time_horizon)
    burned_nodes_at_h = burned_nodes_at_h - not_burned_nodes
    number_resources_used = np.count_nonzero(FRP.solution)
    f = burned_nodes_at_h + eps * number_resources_used
    return f


def constructRandomSolution(FRP):
    while (min(nx.single_source_shortest_path_length(FRP.updated_graph, FRP.ignition_node).values()) < FRP.time_horizon)\
            and (len(FRP.resources) > 0):
        first_available_res_time = find_first_non_zero(FRP.n_resources_time)
        FRP.update_fire_arrival_times()
        unburned_nodes = {node for node, time in FRP.fire_arrivals.items() if time >= first_available_res_time}
        sorted_unburned_nodes = sorted(unburned_nodes, key=lambda node: FRP.fire_arrivals[node])
        restricted_unburned_nodes = sorted_unburned_nodes[0:restrictedCandidateListSize]
        selected_random_node = restricted_unburned_nodes[random.randint(0, len(restricted_unburned_nodes) - 1)]
        # avoid placing resources where there is one already
        while np.any(FRP.solution[selected_random_node[0], selected_random_node[1], :] != 0):
            selected_random_node = restricted_unburned_nodes[random.randint(0, len(restricted_unburned_nodes) - 1)]
        FRP.solution[selected_random_node[0], selected_random_node[1], first_available_res_time] = FRP.resources[0]
        FRP.update_fire_propagation_times(selected_random_node[0], selected_random_node[1], first_available_res_time,
                                          after_removing=False)
        FRP.update_fire_arrival_times()
        FRP.resources = FRP.resources[1:len(FRP.resources)+1]
        FRP.n_resources_time[first_available_res_time] = FRP.n_resources_time[first_available_res_time] - 1
    FRP.update_current_value()
    return FRP, FRP.get_value()


def multiStartConstructiveHeuristic(FRP):
    fstar = float('inf')
    for i in range(numberRepetitionsCH):
        FRP = FRP.reset()
        FRP, fCRS = constructRandomSolution(FRP)
        if fCRS < fstar:
            FRP.update_current_value()
            FRPstar = copy.deepcopy(FRP)
            fstar = FRP.get_value()
    FRPstar.OFV_CH = fstar
    return FRPstar, FRPstar.get_value()


def generateNeighborhood(node0, FRP):
    #neighbours of node0
    neighbor_nodes = list(FRP.updated_graph.neighbors(node0))
    #neighbours of other resources
    for (i,j,k) in zip(np.nonzero(FRP.solution)[0],np.nonzero(FRP.solution)[1],np.nonzero(FRP.solution)[2]):
        neighbor_nodes.append(list(FRP.updated_graph.neighbors((i,j))))
    #rewrite list as a list of nodes
    neighbor_nodes = [node for sublist in neighbor_nodes for node in (sublist if isinstance(sublist, list) else [sublist])]
    #remove nodes where a resource is placed already
    for node in neighbor_nodes:
        if np.max(FRP.solution[node[0], node[1], :]) > 0:
            neighbor_nodes.remove(node)
    #remove burned nodes
    first_available_res_time = find_first_non_zero(FRP.n_resources_time)
    FRP.update_fire_arrival_times()
    burned_nodes = [node for node, time in FRP.fire_arrivals.items() if time < first_available_res_time]
    for node in neighbor_nodes:
        if node in burned_nodes:
            neighbor_nodes.remove(node)
    #sort by fire arrivals and keep the first maxNeighborhoodSize nodes
    sorted_neighbor_nodes = sorted(neighbor_nodes, key=lambda node: FRP.fire_arrivals[node])
    reduced_neighborhood = sorted_neighbor_nodes[:maxNeighbourhoodSize]
    unique_reduced_neighborhood = [item for item in reduced_neighborhood if reduced_neighborhood.count(item) == 1]
    return unique_reduced_neighborhood


def localSearch(FRP):
    improvement = True
    FRP.update_current_value()
    value = FRP.get_value()
    bestMovement = copy.deepcopy(FRP)
    while improvement:
        improvement = False
        #for each node with a resource
        for (i,j,k) in zip(np.nonzero(FRP.solution)[0],np.nonzero(FRP.solution)[1],np.nonzero(FRP.solution)[2]):
            FRP.update_fire_arrival_times()
            #remove the resource from node0
            FRP.remove_resource(i,j,k)
            #update the fire propagation time to the unburned adjacent nodes of node0
            FRP.update_fire_propagation_times(i,j,k, after_removing=True)
            #run dijkstra's algorithm to determine the fire arrival instants
            FRP.update_fire_arrival_times()
            #neighborhood <- generateNeighborhood(node0)
            neighborhood = generateNeighborhood((i,j), FRP)
            #for each node1 in neighborhood
            for node1 in neighborhood:
                unfeasible = False
                #check whether there is already a resource in node1
                if np.max(FRP.solution[node1[0], node1[1], :]) > 0:
                    logging.debug("ALREADY A RESOURCE")
                    continue
                logging.debug("LOCAL SEARCH ON GOING")
                #place the available resource at node1
                FRP.add_resource(node1[0],node1[1],k)
                #update the fire travel time to the adjacent unburned nodes of node1
                FRP.update_fire_propagation_times(node1[0],node1[1],k, after_removing=False)
                FRP.update_fire_arrival_times()
                #check solution feasibility (if fire time arrival is lower then when a resource in that node has been placed)
                for (l,m,n) in zip(np.nonzero(FRP.solution)[0], np.nonzero(FRP.solution)[1], np.nonzero(FRP.solution)[2]):
                    if FRP.fire_arrivals[(l,m)] <= n:
                        unfeasible = True
                if unfeasible:
                    # remove resource from node1
                    FRP.remove_resource(node1[0], node1[1], k)
                    # update the fire travel time to the adjacent unburned nodes
                    FRP.update_fire_propagation_times(node1[0], node1[1], k, after_removing=True)
                    FRP.update_fire_arrival_times()
                    logging.debug("UNFEASIBLE")
                    logging.debug("placed res: ")
                    logging.debug(np.count_nonzero(FRP.solution))
                    logging.debug("available res: ")
                    logging.debug(len(FRP.resources))
                    continue
                #if the solution is feasible and the best possible improvement then Save movement as the bestMovement
                FRP.update_current_value()
                value_new = FRP.get_value()
                if value_new < value:
                    value = value_new
                    bestMovement = copy.deepcopy(FRP)
                    improvement = True
                    logging.debug("NEW BESTMOVEMENT")
                    logging.debug(value)
                #remove resource from node1
                FRP.remove_resource(node1[0], node1[1], k)
                #update the fire travel time to the adjacent unburned nodes
                FRP.update_fire_propagation_times(node1[0], node1[1], k, after_removing=True)
                FRP.update_fire_arrival_times()
                if np.count_nonzero(FRP.solution) + len(FRP.resources) < 5:
                    logging.debug("LOST A RESOURCE")
                logging.debug("FEASIBLE")
                logging.debug("placed res: ")
                logging.debug(np.count_nonzero(FRP.solution))
                logging.debug("available res: ")
                logging.debug(len(FRP.resources))
            #restore the resource to node0
            FRP.add_resource(i,j,k)
            FRP.update_fire_propagation_times(i, j, k, after_removing=False)
            FRP.update_fire_arrival_times()
            logging.debug("RESTORED")
            logging.debug("placed res: ")
            logging.debug(np.count_nonzero(FRP.solution))
            logging.debug("available res: ")
            logging.debug(len(FRP.resources))
        #if there is improvement then execute the bestMovement
        if improvement:
            FRP = copy.deepcopy(bestMovement)
            logging.debug("IMPROVEMENT")
            logging.debug("placed res: ")
            logging.debug(np.count_nonzero(FRP.solution))
            logging.debug("available res: ")
            logging.debug(len(FRP.resources))
        else:
            logging.debug("NO IMPROVEMENT")
            logging.debug("placed res: ")
            logging.debug(np.count_nonzero(FRP.solution))
            logging.debug("available res: ")
            logging.debug(len(FRP.resources))
    FRP.update_current_value()
    return FRP, FRP.get_value()


def acceptanceCriterion(FRP, best_FRP):
    FRP.update_current_value()
    best_FRP.update_current_value()
    if FRP.get_value() < best_FRP.get_value():
        new_best_FRP = FRP
    else:
        new_best_FRP = best_FRP
    return copy.deepcopy(new_best_FRP)


def perturbation(best_FRP):
    FRP = copy.deepcopy(best_FRP)
    #if not available resources set prob2 to zero
    if len(FRP.resources) == 0:
        prob2 = 0
    else:
        prob2 = probAddingRes
    #rnd = rand()
    rnd = random.random()
    #first perturbation type with prob1
    if rnd < probRemovingRes: # and len(FRP.resources) < 1:
        #remove a resource from node with largest deployment instant and update resource availability
        nonzero_indices = np.nonzero(FRP.solution)
        third_indices = nonzero_indices[2]
        index_with_largest_third_index = np.argmax(third_indices)
        i, j, k = nonzero_indices[0][index_with_largest_third_index], nonzero_indices[1][index_with_largest_third_index], \
                  nonzero_indices[2][index_with_largest_third_index]
        FRP.remove_resource(i,j,k)
        #update the fire travel time to its adjacent nodes
        FRP.update_fire_propagation_times(i,j,k,after_removing=True)
        FRP.update_fire_arrival_times()
        logging.debug("REMOVED RES")
        logging.debug("placed res: ")
        logging.debug(np.count_nonzero(FRP.solution))
        logging.debug("available res: ")
        logging.debug(len(FRP.resources))
    else:
        if rnd < probRemovingRes + prob2:
            #find the least release time of an available resource
            first_available_res_time = find_first_non_zero(FRP.n_resources_time)
            #run dijkstra's algorithm to determine the fire arrival instants
            FRP.update_fire_arrival_times()
            #identify the set of candidate nodes to receive the resource (unburned and without resource)
            unburned_nodes = {node for node, time in FRP.fire_arrivals.items() if time >= first_available_res_time}
            nodes_with_resources = np.nonzero(FRP.solution)
            nodes_with_resources = zip(nodes_with_resources[0], nodes_with_resources[1])
            candidate_nodes_list = [node for node in unburned_nodes if node not in nodes_with_resources]
            #sort candidate nodes in ascending order by their fire arrival instants
            sorted_candidate_nodes_list = sorted(candidate_nodes_list, key=lambda node: FRP.fire_arrivals[node])
            #build a candidate list for receiving the resource
            if len(sorted_candidate_nodes_list) > restrictedCandidateListSize:
                restricted_sorted_candidate_nodes_list = sorted_candidate_nodes_list[:restrictedCandidateListSize]
            #randomly select a node from the candidate list
            selected_node = restricted_sorted_candidate_nodes_list[random.randint(0,
                                                                                  len(restricted_sorted_candidate_nodes_list)-1)]
            #assign a resource to the selected node
            #update resource availability
            FRP.add_resource(selected_node[0], selected_node[1], first_available_res_time)
            #update fire travel time to its adjacent nodes
            FRP.update_fire_propagation_times(selected_node[0], selected_node[1], first_available_res_time,
                                              after_removing=False)
            logging.debug("ADDED RES")
            logging.debug("placed res: ")
            logging.debug(np.count_nonzero(FRP.solution))
            logging.debug("available res: ")
            logging.debug(len(FRP.resources))
        else:
            mod = 0
            failure = 0
            while mod < maxNumberModifications and failure < maxNumberFailures:
                #randomly select a node with resource
                nodes_with_resources = np.nonzero(FRP.solution)
                rnd_index = random.randint(0,len(nodes_with_resources[0])-1)
                selected_random_node = nodes_with_resources[0][rnd_index], nodes_with_resources[1][rnd_index], \
                                       nodes_with_resources[2][rnd_index]
                time_resource_placed = selected_random_node[2]
                #remove the resource
                FRP.remove_resource(selected_random_node[0], selected_random_node[1], selected_random_node[2])
                #update the fire propagation time to its adjacent nodes
                FRP.update_fire_propagation_times(selected_random_node[0], selected_random_node[1],
                                                  selected_random_node[2], after_removing=True)
                #run dijkstra's algorithm to determine the fire arrival instants
                FRP.update_fire_arrival_times()
                #generate the neighbourhood (all unburned nodes in ascending order, then splitted)
                unburned_nodes = {node for node, time in FRP.fire_arrivals.items() if time >= time_resource_placed}
                #remove nodes where a resource is placed already
                for (i, j, k) in zip(np.nonzero(FRP.solution)[0], np.nonzero(FRP.solution)[1], np.nonzero(FRP.solution)[2]):
                    if (i, j) in unburned_nodes:
                        unburned_nodes.remove((i, j))
                sorted_unburned_nodes = sorted(unburned_nodes, key=lambda node: FRP.fire_arrivals[node])
                unique_sorted_unburned_nodes = [item for item in sorted_unburned_nodes
                                                if sorted_unburned_nodes.count(item) == 1]
                neighbourhood = unique_sorted_unburned_nodes[0:maxNeighbourhoodSize]
                #randomly select a node from the neighbourhood
                selected_neighbourhood_node = neighbourhood[random.randint(0,len(neighbourhood) - 1)]
                #place the resource at the selected node
                FRP.add_resource(selected_neighbourhood_node[0], selected_neighbourhood_node[1], time_resource_placed)
                #update the fire propagation time to its adjacent nodes
                FRP.update_fire_propagation_times(selected_neighbourhood_node[0], selected_neighbourhood_node[1],
                                                  time_resource_placed, after_removing=False)
                #run dijkstra's algorithm to determine the fire arrival instants
                FRP.update_fire_arrival_times()
                #assess the solution feasibility
                for (l,m,n) in zip(np.nonzero(FRP.solution)[0], np.nonzero(FRP.solution)[1], np.nonzero(FRP.solution)[2]):
                    if FRP.fire_arrivals[(l,m)] < n:
                        unfeasible = True
                        break
                    else:
                        unfeasible = False
                if not unfeasible:
                    mod = mod + 1
                    failure = 0
                    logging.debug("SUCCESSFUL MODIFICATION")
                    logging.debug("placed res: ")
                    logging.debug(np.count_nonzero(FRP.solution))
                    logging.debug("available res: ")
                    logging.debug(len(FRP.resources))
                else:
                    #undo the proposed movement
                    #remove the resource from new location
                    FRP.remove_resource(selected_neighbourhood_node[0], selected_neighbourhood_node[1],
                                        time_resource_placed)
                    FRP.update_fire_propagation_times(selected_neighbourhood_node[0], selected_neighbourhood_node[1],
                                                      time_resource_placed, after_removing=True)
                    #add the resource at the old spot
                    FRP.add_resource(selected_random_node[0], selected_random_node[1], time_resource_placed)
                    # update the fire propagation time to its adjacent nodes
                    FRP.update_fire_propagation_times(selected_random_node[0], selected_random_node[1],
                                                      time_resource_placed, after_removing=False)
                    #increment failures count
                    failure = failure + 1
                    logging.debug("FAILURE MODIFICATION")
                    logging.debug("placed res: ")
                    logging.debug(np.count_nonzero(FRP.solution))
                    logging.debug("available res: ")
                    logging.debug(len(FRP.resources))
    #run dijkstra algorithm
    FRP.update_fire_arrival_times()
    FRP.update_current_value()
    return copy.deepcopy(FRP), FRP.get_value()


def setGlobalParameters(FRP):
    global h
    h = 28
    global maxIterationsWithoutImprovement
    maxIterationsWithoutImprovement = 50
    global probRemovingRes
    probRemovingRes = 0.075
    global probAddingRes
    probAddingRes = 0.025
    global maxNumberModifications
    maxNumberModifications = random.randint(3, 5)
    global maxNumberFailures
    maxNumberFailures = 100
    global maxNumberPerturbations
    global numberRepetitionsCH
    global maxNeighbourhoodSize
    global restrictedCandidateListSize
    if FRP.size == 6:
        maxNumberPerturbations = 75
        numberRepetitionsCH = 500
        maxNeighbourhoodSize = 6
        restrictedCandidateListSize = 5
    elif FRP.size == 10:
        maxNumberPerturbations = 100
        numberRepetitionsCH = 500
        maxNeighbourhoodSize = 10
        restrictedCandidateListSize = 5
    elif FRP.size == 20:
        maxNumberPerturbations = 200
        numberRepetitionsCH = 1000
        maxNeighbourhoodSize = 20
        restrictedCandidateListSize = 6
    elif FRP.size == 30:
        maxNumberPerturbations = 250
        numberRepetitionsCH = 1000
        maxNeighbourhoodSize = 30
        restrictedCandidateListSize = 6
    else:
        print("Error while setting parameters!")


def iteratedLocalSearch(FRP, show_plot=False):
    setGlobalParameters(FRP)
    FRP, f_CH = multiStartConstructiveHeuristic(FRP)
    FRP, f_LS = localSearch(FRP)
    FRP.OFV_LS = f_LS
    best_FRP = copy.deepcopy(FRP)
    fstar = best_FRP.get_value()
    no_improvement_counter = 0
    while no_improvement_counter < maxIterationsWithoutImprovement:
        FRP, f_p = perturbation(best_FRP)
        logging.debug("PERTURBATION FINISHED")
        logging.debug("placed res: ")
        logging.debug(np.count_nonzero(FRP.solution))
        logging.debug("available res: ")
        logging.debug(len(FRP.resources))
        FRP, f_LS = localSearch(FRP)
        logging.debug("LOCAL SEARCH FINISHED")
        logging.debug("placed res: ")
        logging.debug(np.count_nonzero(FRP.solution))
        logging.debug("available res: ")
        logging.debug(len(FRP.resources))
        best_FRP = acceptanceCriterion(FRP, best_FRP)
        logging.debug("value:")
        logging.debug(best_FRP.get_value())
        logging.debug("res unused:")
        logging.debug(len(best_FRP.resources))
        if best_FRP.get_value() == fstar:
            no_improvement_counter = no_improvement_counter + 1
            logging.debug("NO IMPROVEMENT ILS")
        else:
            no_improvement_counter = 0
            fstar = best_FRP.get_value()
            logging.debug("IMPROVEMENT ILS")
    if show_plot:
        best_FRP.show()
    best_FRP.OFV_ILS = best_FRP.get_value()
    return copy.deepcopy(best_FRP)


def test_ILS(dataset, n_resources, delta=50, time_horizon=28, n_replications = 5, show_plot=False):
    if DEBUG:
        logging.basicConfig(level=logging.DEBUG, format='%(name)s - %(levelname)s - %(message)s')
    keys = ["instance", "best_OFV_CH", "best_OFV_LS", "best_OFV_ILS", "avg_OFV_CH", "avg_OFV_LS", "avg_OFV_ILS",
            "avg_runtime", "total_runtime"]
    default_value = 0
    solution = dict.fromkeys(keys, default_value)
    with open("../dataset_ILS_" + str(n_resources) + "resources.csv", "w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=solution.keys())
        writer.writeheader()
    for data in dataset:
        best_OFV_CH = float('inf')
        best_OFV_LS = float('inf')
        best_OFV_ILS = float('inf')
        cumulative_OFV_CH = 0
        cumulative_OFV_LS = 0
        cumulative_OFV_ILS = 0
        t_0 = time.time()
        for i in range(n_replications):
            FRP = FRPInstance(data["size"], data["ignition_node"], data["graph"], n_resources, delta=delta,
                              time_horizon=time_horizon)
            best_FRP = iteratedLocalSearch(FRP, show_plot=show_plot)
            if best_FRP.OFV_ILS < best_OFV_ILS:
                best_OFV_CH = best_FRP.OFV_CH
                best_OFV_LS = best_FRP.OFV_LS
                best_OFV_ILS = best_FRP.OFV_ILS
            cumulative_OFV_CH = cumulative_OFV_CH + best_FRP.OFV_CH
            cumulative_OFV_LS = cumulative_OFV_LS + best_FRP.OFV_LS
            cumulative_OFV_ILS = cumulative_OFV_ILS + best_FRP.OFV_ILS
        total_runtime = time.time() - t_0
        avg_OFV_CH = cumulative_OFV_CH / n_replications
        avg_OFV_LS = cumulative_OFV_LS / n_replications
        avg_OFV_ILS = cumulative_OFV_ILS / n_replications
        avg_runtime = total_runtime / n_replications
        solution["instance"] = data["instance"]
        solution["best_OFV_CH"] = best_OFV_CH
        solution["best_OFV_LS"] = best_OFV_LS
        solution["best_OFV_ILS"] = best_OFV_ILS
        solution["avg_OFV_CH"] = avg_OFV_CH
        solution["avg_OFV_LS"] = avg_OFV_LS
        solution["avg_OFV_ILS"] = avg_OFV_ILS
        solution["avg_runtime"] = avg_runtime
        solution["total_runtime"] = total_runtime
        print("Instance " + str(data["instance"]) + " finished... \nResults:")
        print(solution)
        with open("../dataset_ILS_" + str(n_resources) + "resources.csv", "a", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=solution.keys())
            writer.writerow(solution)


if __name__ == '__main__':
    with open('../dataset.pickle', 'rb') as file:
        dataset = pickle.load(file)
    test_ILS(dataset, n_resources=5, delta=50, time_horizon=28, n_replications=5)
    with open('../dataset.pickle', 'rb') as file:
        dataset = pickle.load(file)
    test_ILS(dataset, n_resources=6, delta=50, time_horizon=28, n_replications=5)