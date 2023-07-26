'''
CPU model: Intel(R) Core(TM) i5-5350U CPU @ 1.80GHz
'''
import random
import numpy as np
import networkx as nx
import pickle



def find_first_non_zero(arr):
    for index, element in enumerate(arr):
        if element != 0:
            return index
    return None


def evaluate(dict, s):
    eps = 1 / dict["n_resources"]
    updated_G = dict["updated_graph"]
    fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
    unburned_nodes_at_h = sum(1 for time in fire_arrivals.values() if time > h)
    number_resources_used = np.count_nonzero(s)
    f = unburned_nodes_at_h + eps * number_resources_used
    return f


def constructRandomSolution(dict):
    s = np.zeros((dict["size"], dict["size"], h))
    #s[i,j,k] solution represent whether at time k a resources has been placed on (i,j)
    updated_G = dict["graph"]
    dict["updated_graph"] = updated_G.copy()
    while (min(nx.single_source_shortest_path_length(updated_G, dict["ignition_node"]).values()) < h) and (len(dict["resources"]) > 0):
        first_available_res_time = find_first_non_zero(dict["n_resources_time"])
        fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
#        burned_nodes = {node for node, time in fire_arrivals.items() if time < first_available_res_time}
        unburned_nodes = {node for node, time in fire_arrivals.items() if time >= first_available_res_time}
#        sorted_burned_nodes = sorted(burned_nodes, key=lambda node: fire_arrivals[node])
        sorted_unburned_nodes = sorted(unburned_nodes, key=lambda node: fire_arrivals[node])
        restricted_unburned_nodes = sorted_unburned_nodes[0:restrictedCandidateListSize]
        selected_random_node = restricted_unburned_nodes[random.randint(0,len(restricted_unburned_nodes) - 1)]
        s[selected_random_node[0], selected_random_node[1], first_available_res_time] = dict["resources"][0]
        dict["resources"] = dict["resources"][1:len(dict["resources"])]
        dict["n_resources_time"][first_available_res_time] = dict["n_resources_time"][first_available_res_time] - 1
        for neighbor_node in updated_G.neighbors(selected_random_node):
            updated_G[selected_random_node][neighbor_node]['weight'] = updated_G[selected_random_node][neighbor_node]['weight'] + dict["delta"]
    return s, evaluate(dict, s)


def multiStartConstructiveHeuristic(dict):
    fstar = float('inf')
    for i in range(numberRepetitionsCH):
        s = constructRandomSolution(dict)
        if evaluate(dict, s) < fstar:
            sstar = s
            fstar = evaluate(dict, s)
    return sstar, fstar


def generateNeighborhood(node0, s, updated_G, n_resources_time):
    neighbor_nodes = list(updated_G.neighbors(node0))
    for (i,j,k) in zip(np.nonzero(s)[0],np.nonzero(s)[1],np.nonzero(s)[2]):
        neighbor_nodes.append(list(updated_G.neighbors((i,j))))
    if len(neighbor_nodes) > maxNeighbourhoodSize:
        #remove burned nodes
        first_available_res_time = find_first_non_zero(n_resources_time)
        fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
        burned_nodes = {node for node, time in fire_arrivals.items() if time < first_available_res_time}
        for node in neighbor_nodes:
            if node in burned_nodes:
                neighbor_nodes.remove(node)
        #sort by fire arrivals and keep the first maxNeighborhoodSize nodes
        sorted_neighbor_nodes = sorted(neighbor_nodes, key=lambda node: fire_arrivals[node])
        reduced_neighborhood = sorted_neighbor_nodes[:maxNeighbourhoodSize]
    return reduced_neighborhood


def localSearch(dict, s0):
    updated_G = dict["updated_graph"].copy()
    s = s0
    sstar = s
    improvement = True
    value = evaluate(dict, s)
    bestMovement = s
    while improvement:
        improvement = False
        #non_zero_indices = np.nonzero(s)
        for (i,j,k) in zip(np.nonzero(s)[0],np.nonzero(s)[1],np.nonzero(s)[2]):
            fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
            #remove the resource from node0
            dict["resources"].append(s[i,j,k])
            if dict["n_resources"] == 5:
                if dict["resources"][-1] >= 3:
                    dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                else:
                    dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
            elif dict["resources"] == 6:
                if dict["resources"][-1] >= 4:
                    dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                else:
                    dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
            s[i,j,k] = 0
            #update the fire propagation time to the unburned adjacent nodes of node0
            for neighbor_node in updated_G.neighbors((i,j)):
                if fire_arrivals[neighbor_node] > k:
                    updated_G[(i,j)][neighbor_node]['weight'] = updated_G[(i,j)][neighbor_node]['weight'] - dict["delta"]
            #run dijkstra's algorithm to determine the fire arrival instants
            fire_arrivals_updated = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
            #neighborhood <- generateNeighborhood(node0)
            neighborhood = generateNeighborhood((i,j), s, updated_G, dict["n_resources_time"])
            #for each node1 in neighborhood
            for node1 in neighborhood:
                unfeasible = False
                #place the available resource at node1
                s[node1[0], node1[1], k] = dict["resources"][-1]
                if dict["n_resources"] == 5:
                    if dict["resources"][-1] >= 3:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
                elif dict["n_resources"] == 6:
                    if dict["resources"][-1] >= 4:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
                dict["resources"][-1].pop()
                #update the fire travel time to the adjacent unburned nodes of node1
                for neighbor_node in updated_G.neighbors(node1):
                    if fire_arrivals_updated[neighbor_node] > k:
                        updated_G[node1][neighbor_node]['weight'] = updated_G[node1][neighbor_node]['weight'] + dict["delta"]
                fire_arrivals_upupdated = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
                #check solution feasibility (if fire time arrival is lower then when a resource in that node has been placed)
                for (l,m,n) in zip(np.nonzero(s)[0], np.nonzero(s)[1], np.nonzero(s)[2]):
                    if fire_arrivals_upupdated[(l,m)] <= n: #or k
                        unfeasible = True
                if unfeasible:
                    continue
                #if the solution is feasible and the best possible improvement then Save movement as the bestMovement
                value_new = evaluate(dict, s)
                if value_new > value:
                    value = value_new
                    bestMovement = s
                    improvement = True
                #remove resource from node1
                dict["resources"].append(s[node1[0], node1[1], k])
                if dict["n_resources"] == 5:
                    if dict["resources"][-1] >= 3:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
                elif dict["n_resources"] == 6:
                    if dict["resources"][-1] >= 4:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
                s[node1[0], node1[1], k] = 0
                #update the fire travel time to the adjacent unburned nodes
                for neighbor_node in updated_G.neighbors(node1):
                    if fire_arrivals_updated[neighbor_node] > k:
                        updated_G[node1][neighbor_node]['weight'] = updated_G[node1][neighbor_node]['weight'] - dict["delta"]
            #restore the resource to node0
            if dict["n_resources"] == 5:
                if dict["resources"][-1] >= 3:
                    dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                else:
                    dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
            elif dict["n_resources"] == 6:
                if dict["resources"][-1] >= 4:
                    dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                else:
                    dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
            s[i,j,k] = dict["resources"][-1]
            dict["resources"][-1].pop()
        #if there is improvement then execute the bestMovement
        if improvement:
            sstar = bestMovement
    return sstar, evaluate(dict, sstar)


def acceptanceCriterion(sstar, sstarp):
    fstar = evaluate(dict, sstar)
    fstarp = evaluate(dict, sstarp)
    if fstarp > fstar:
        sstar = sstarp
        fstar = fstarp
    return sstar, fstar


def perturbation(dict, s):
    updated_G = dict["updated_graph"].copy()
    #if not available resources set prob2 to zero
    if not np.nonzero(s) < dict["n_resources"]:
        prob2 = 0
    #rnd = rand()
    rnd = random.rand()
    #first perturbation type with prob1
    if rnd < probRemovingRes:
        #remove a resource from node with largest deployment instant and update resource availability
        nonzero_indices = np.nonzero(s)
        third_indices = nonzero_indices[2]
        index_with_largest_third_index = np.argmax(third_indices)
        i, j, k = nonzero_indices[index_with_largest_third_index]
        dict["resources"].append(s[i, j, k])
        if dict["n_resources"] == 5:
            if dict["resources"][-1] >= 3:
                dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
            else:
                dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
        elif dict["n_resources"] == 6:
            if dict["resources"][-1] >= 4:
                dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
            else:
                dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
        s[i, j, k] = 0
        #update the fire travel time to its adjacent nodes
        for neighbor_node in updated_G.neighbors((i,j)):
                updated_G[(i,j)][neighbor_node]['weight'] = updated_G[(i,j)][neighbor_node]['weight'] - dict["delta"]
    else:
        if rnd < probRemovingRes + probAddingRes:
            #find the least release time of an available resource
            first_available_res_time = find_first_non_zero(dict["n_resources_time"])
            #run dijkstra's algorithm to determine the fire arrival instants
            fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
            #identify the set of candidate nodes to receive the resource (unburned and without resource)
            unburned_nodes = {node for node, time in fire_arrivals.items() if time >= first_available_res_time}
            nodes_with_resources = np.nonzero(s)
            nodes_with_resources = zip(nodes_with_resources[0], nodes_with_resources[1])
            candidate_nodes_list = [node for node in unburned_nodes if node not in nodes_with_resources]
            #sort candidate nodes in ascending order by their fire arrival instants
            sorted_candidate_nodes_list = sorted(candidate_nodes_list, key=lambda node: fire_arrivals[node])
            #build a candidate list for receiving the resource
            if len(sorted_candidate_nodes_list) > restrictedCandidateListSize:
                restricted_sorted_candidate_nodes_list = sorted_candidate_nodes_list[:restrictedCandidateListSize]
            #randomly select a node from the candidate list
            selected_node = restricted_sorted_candidate_nodes_list[random.randint(0, len(restricted_sorted_candidate_nodes_list) - 1)]
            #assign a resource to the selected node
            s[selected_node[0], selected_node[1], first_available_res_time] = dict["resources"][-1]
            #update resource availability
            if dict["n_resources"] == 5:
                if dict["resources"][-1] >= 3:
                    dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                else:
                    dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
            elif dict["n_resources"] == 6:
                if dict["resources"][-1] >= 4:
                    dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                else:
                    dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
            dict["resources"].pop()
            #update fire travel time to its adjacent nodes
            for neighbor_node in updated_G.neighbors((selected_node[0],selected_node[1])):
                updated_G[(selected_node[0],selected_node[1])][neighbor_node]['weight'] = updated_G[(selected_node[0],selected_node[1])][neighbor_node]['weight'] + dict["delta"]
        else:
            mod = 0
            failure = 0
            while mod < maxNumberModifications and failure < maxNumberFailures:
                #randomly select a node with resource
                nodes_with_resources = np.nonzero(s)
                selected_random_node = nodes_with_resources[random.randint(0,len(nodes_with_resources)-1)]
                #remove the resource
                dict["resources"].append(s[selected_random_node[0], selected_random_node[1], selected_random_node[2]])
                time_resource_placed = selected_random_node[2]
                if dict["n_resources"] == 5:
                    if dict["resources"][-1] >= 3:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
                elif dict["n_resources"] == 6:
                    if dict["resources"][-1] >= 4:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
                s[selected_random_node[0], selected_random_node[1], selected_random_node[2]] = 0
                #update the fire propagation time to its adjacent nodes
                for neighbor_node in updated_G.neighbors((selected_random_node[0], selected_random_node[1])):
                    updated_G[(selected_random_node[0], selected_random_node[1])][neighbor_node]['weight'] = updated_G[(selected_random_node[0], selected_random_node[1])][neighbor_node]['weight'] - dict["delta"]
                #run dijkstra's algorithm to determine the fire arrival instants
                fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
                #generate the neighbourhood (all unburned nodes in ascending order, then splitted)
                unburned_nodes = {node for node, time in fire_arrivals.items() if time >= time_resource_placed}
                sorted_unburned_nodes = sorted(unburned_nodes, key=lambda node: fire_arrivals[node])
                neighbourhood = sorted_unburned_nodes[0:maxNeighbourhoodSize]
                #randomly select a node from the neighbourhood
                selcted_neighbourhood_node = neighbourhood[random.randint(0,len(neighbourhood) - 1)]
                #place the resource at the selected node
                s[selcted_neighbourhood_node[0], selcted_neighbourhood_node[1], time_resource_placed] = dict["resources"][-1]
                # update resource availability
                if dict["n_resources"] == 5:
                    if dict["resources"][-1] >= 3:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
                elif dict["n_resources"] == 6:
                    if dict["resources"][-1] >= 4:
                        dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                    else:
                        dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
                dict["resources"].pop()
                #update the fire propagation time to its adjacent nodes
                for neighbor_node in updated_G.neighbors((selcted_neighbourhood_node[0], selcted_neighbourhood_node[1])):
                    updated_G[(selcted_neighbourhood_node[0], selcted_neighbourhood_node[1])][neighbor_node]['weight'] = updated_G[(selcted_neighbourhood_node[0], selcted_neighbourhood_node[1])][neighbor_node]['weight'] + dict["delta"]
                #run dijkstra's algorithm to determine the fire arrival instants
                new_fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
                #assess the solution feasibility
                for (l,m,n) in zip(np.nonzero(s)[0], np.nonzero(s)[1], np.nonzero(s)[2]):
                    if new_fire_arrivals[(l,m)] <= n:
                        unfeasible = True
                    else:
                        unfeasible = False
                if not unfeasible:
                    mod = mod + 1
                    failure = 0
                else:
                    #undo the proposed movement
                    #remove the resource from new location
                    dict["resources"].append(s[selcted_neighbourhood_node[0], selcted_neighbourhood_node[1], selcted_neighbourhood_node[2]])
                    time_resource_placed = selcted_neighbourhood_node[2]
                    if dict["n_resources"] == 5:
                        if dict["resources"][-1] >= 3:
                            dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                        else:
                            dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
                    elif dict["n_resources"] == 6:
                        if dict["resources"][-1] >= 4:
                            dict["n_resources_time"][14] = dict["n_resources_time"][14] + 1
                        else:
                            dict["n_resources_time"][9] = dict["n_resources_time"][9] + 1
                    s[selcted_neighbourhood_node[0], selcted_neighbourhood_node[1], selcted_neighbourhood_node[2]] = 0
                    for neighbor_node in updated_G.neighbors((selcted_neighbourhood_node[0], selcted_neighbourhood_node[1])):
                        updated_G[(selcted_neighbourhood_node[0], selcted_neighbourhood_node[1])][neighbor_node]['weight'] = updated_G[(selcted_neighbourhood_node[0], selcted_neighbourhood_node[1])][neighbor_node]['weight'] - dict["delta"]
                    #add the resource at the old spot
                    s[selected_random_node[0], selected_random_node[1], time_resource_placed] = dict["resources"][-1]
                    # update resource availability
                    if dict["n_resources"] == 5:
                        if dict["resources"][-1] >= 3:
                            dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                        else:
                            dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
                    elif dict["n_resources"] == 6:
                        if dict["resources"][-1] >= 4:
                            dict["n_resources_time"][14] = dict["n_resources_time"][14] - 1
                        else:
                            dict["n_resources_time"][9] = dict["n_resources_time"][9] - 1
                    dict["resources"].pop()
                    # update the fire propagation time to its adjacent nodes
                    for neighbor_node in updated_G.neighbors((selected_random_node[0], selected_random_node[1])):
                        updated_G[(selected_random_node[0], selected_random_node[1])][neighbor_node]['weight'] = updated_G[(selected_random_node[0], selected_random_node[1])][neighbor_node]['weight'] + dict["delta"]
                    #increment failures count
                    failure = failure + 1
    #run dijkstra algorithm
    final_fire_arrivals = nx.single_source_shortest_path_length(updated_G, dict["ignition_node"])
    return s, evaluate(dict,s)


def setGlobalParameters(dict):
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
    if dict["size"] == 6:
        maxNumberPerturbations = 75
        numberRepetitionsCH = 500
        maxNeighbourhoodSize = 6
        restrictedCandidateListSize = 5
    elif dict["size"] == 10:
        maxNumberPerturbations = 100
        numberRepetitionsCH = 500
        maxNeighbourhoodSize = 10
        restrictedCandidateListSize = 5
    elif dict["dict"] == 20:
        maxNumberPerturbations = 200
        numberRepetitionsCH = 1000
        maxNeighbourhoodSize = 20
        restrictedCandidateListSize = 6
    elif dict["dict"] == 30:
        maxNumberPerturbations = 250
        numberRepetitionsCH = 1000
        maxNeighbourhoodSize = 30
        restrictedCandidateListSize = 6
    else:
        print("Error while setting parameters!")


def iteratedLocalSearch(dict):
    G = dict["graph"]
    grid_graph = nx.grid_2d_graph(dict["size"], dict["size"], create_using=nx.DiGraph())
    grid_graph.add_edges_from(G.edges())
    dict["graph"] = grid_graph.copy(as_view=False)
    dict["updated_graph"] = grid_graph.copy(as_view=False)
    dict["n_resources"] = 5
    dict["resources"] = [i for i in range(1, dict["n_resources"] + 1)]
    dict["delta"] = 50
    dict["n_resources_time"] = [0 for _ in range(h)]
    dict["n_resources_time"][14] = 3
    dict["n_resources_time"][9] = len(dict["resources"]) - 3
    setGlobalParameters(dict)
    (s0, f0) = multiStartConstructiveHeuristic(dict)
    (sstar, fstar) = localSearch(s0)
    noimprovement_counter = 0
    while noimprovement_counter < maxIterationsWithoutImprovement: #TODO stoppingCriterionIsNotMet (maximum number of iteration without improvement)
        (sp, fp) = perturbation(sstar)
        (sstarp, fstarp) = localSearch(sp)
        (sstar, fstar) = acceptanceCriterion(sstar, sstarp)
        if sstar == sstarp:
            noimprovement_counter = 0
        else:
            noimprovement_counter = noimprovement_counter + 1
    return sstar, fstar


if __name__ == '__main__':
    with open('dataset.pickle', 'rb') as file:
        dataset = pickle.load(file)

    dict = dataset[0]
    sstar, fstar = iteratedLocalSearch(dict)
    print(sstar, fstar)