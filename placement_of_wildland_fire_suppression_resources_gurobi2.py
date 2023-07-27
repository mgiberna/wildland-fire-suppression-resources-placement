'''
CPU model: Intel(R) Core(TM) i5-5350U CPU @ 1.80GHz
Thread count: 2 physical cores, 4 logical processors, using up to 4 threads
'''
import gurobipy as gb #v10.0.2
import networkx as nx
import time
import pickle
import csv


def fire_resource_placement_model(dict, time_horizon, setup1):
    G = dict["graph"]
    # fire resources placement model
    FRP = gb.Model()

    # SETS AND PARAMETERS
    # N set of nodes
    N = G.nodes()
    # A set of arcs
    A = G.edges()
    # R set of resources
    R1 = (1,2,3,4,5)
    R2 = (1,2,3,4,5,6)
    # h target instant
    h = time_horizon
    # K set of time instants
    K = range(0,h)
    # ign ignition node
    ign = dict["ignition_node"]
    # n number of nodes
    n = G.number_of_nodes()
    # c_ij fire spread time between the (center of) node i and the (center of) node j in that direction
    c = gb.tupledict()
    i = 0
    for source in N:
        j = 0
        for target in N:
            shortest_path = nx.shortest_path(G, source=source, target=target, weight='weight')
            total_weight = sum(G[u][v]['weight'] for u, v in zip(shortest_path[:-1], shortest_path[1:]))
            c[source, target] = total_weight
            j += 1
        i += 1
    # cmax maximum fire spread time between any two nodes
    cmax = max(c.values())
    # a_k number of resources that become available at instant k
    a1 = tuple([0]*h)
    a1 = a1[:9] + (2,) + a1[10:14] + (3,) + a1[15:]
    a2 = tuple([0]*h)
    a2 = a2[:9] + (3,) + a2[10:14] + (3,) + a2[15:]
    # delta delay, expressed in time units, of the fire arrival to an unburned adjacent node to the one that received a resource
    delta = 50
    # eps weight of the total number of resources in the objective function
    eps1 = 1/len(R1)
    eps2 = 1/len(R2)
    # set whether considering setup1 with 5 resources or setup2 with 6 resources
    # setup1 = True # False -> setup2
    if setup1:
        R = R1
        a = a1
        eps = eps1
    else: #setup2
        R = R2
        a = a2
        eps = eps2

    # DECISION VARIABLES
    # x_ij the number of shortest paths that include arc i,j
    X = FRP.addVars([(i, j) for i, j in A], vtype=gb.GRB.INTEGER, name="X")
    # t_i length of a shortest path between the root and each node i
    T = FRP.addVars([i for i in N], vtype=gb.GRB.INTEGER, name="T")
    # s_ij slack variable that is zero whenever arc ij belongs to a shortest path
    S = FRP.addVars([(i, j) for i, j in A], vtype=gb.GRB.INTEGER, name="S")
    # q_ij a binary variable that equal 1 if arc ij belongs to a shortest path, and 0 otherwise
    Q = FRP.addVars([(i, j) for i, j in A], vtype=gb.GRB.BINARY, name="Q")
    # y^k_i a binary variable that equals 1 if node i is burned at instant k
    Y = FRP.addVars([(i, k) for i in N for k in K], vtype=gb.GRB.BINARY, name="Y")
    # o_k number of resources available but not used at instant k and therefore available at instant k + 1
    O = FRP.addVars([k for k in K], vtype=gb.GRB.INTEGER, name="O")
    # z^kr_i a binary variable that equals 1 if node i receives resource r at instant k (bk), and 0 otherwise
    Z = FRP.addVars([(i,k,r) for i in N for k in K for r in R], vtype=gb.GRB.BINARY, name="Z")

    # CONSTRAINTS
    # force that n − 1 paths departure from the ignition node ign (starting node)
    #FRP.addConstr((X[ign, :].sum() == n - 1), name = 'Constraint2')
    FRP.addConstr((gb.quicksum(X[(i, j)] for (i,j) in A if i == ign) == n - 1), name='Constraint2')
    # guarantees that one path reaches each node in the network
    FRP.addConstrs((-gb.quicksum(X[(i,j)] for (l,j) in A if l == i) + gb.quicksum(X[(j,i)] for (j,l) in A if l == i)  == 1 for i in N if i != ign), name = 'Constraint3')
    # the fire arrival time of ignition node is zero
    FRP.addConstr((T[ign] == 0), name='Constraint4')
    # activates the binary variable qi j if arc (i, j) belongs to a shortest path
    FRP.addConstrs((X[(i,j)] <= (n - 1)*Q[(i,j)] for (i,j) in A), name='Constraint5')
    # a resource can be assigned at most once to any node
    FRP.addConstrs((gb.quicksum(Z[i,k,r] for i in N for k in K) <= 1 for r in R), name='Constraint6')
    # each node can receive at most one resource throughout the planning period
    FRP.addConstrs((gb.quicksum(Z[i,k,r] for r in R for k in K) <= 1 for i in N), name='Constraint7')
    # allows assigning resources at instant 1, based on the number of resources that were released (a1) (note time instant 1 is at index 0)
    FRP.addConstr((gb.quicksum(Z[i,1,r] for i in N for r in R) + O[0] == a[0]), name='Constraint8')
    # controls the number of available resources by balancing the number of unassigned resources at the end of each time period
    FRP.addConstrs((gb.quicksum(Z[i,k,r] for i in N for r in R) + O[k] == a[k-1] + O[k-1] for k in K[1:]), name='Constraint9')
    # checked whether node i is burned at instant bk
    FRP.addConstrs((Z[i,k,r] <= 1 + (T[i] - k)/k for i in N for k in K for r in R), name='Constraint10')
    # calculates the fire arrival instant at node j, having node i as origin. In case a resource has been assigned to node i,
    # a delay in the fire arrival at node j is guaranteed by parameter delta
    FRP.addConstrs((T[j] - T[i] + S[(i,j)] == c[i,j] + delta*gb.quicksum(Z[i,k,r] for k in K for r in R ) for (i,j) in A), name='Constraint11')
    # forces a slack variable sij to be zero whenever arc (i, j) belongs to a shortest path (i.e., when qij is one)
    FRP.addConstrs((S[(i,j)] <= ((n - 1)*cmax + (len(R) - 1)*delta)*(1 - Q[(i,j)]) for (i,j) in A), name='Constraint12')
    # define if node i is burned at instant k
    FRP.addConstrs((Y[(i,k)] >= (k - T[i] + 1)/k - gb.quicksum(Z[i,l,r] for r in R for l in range(1,k+1)) for k in K for i in N if i != ign), name='Constraint13')
    FRP.addConstrs((Y[(i,k)] <= 1 + (k*(1 - gb.quicksum(Z[i,l,r] for r in R for l in range(1,k+1))) - T[i])/((n - 1)*cmax + (len(R) - 1)*delta) for k in K for i in N), name='Constraint14')
    # variables domain
    FRP.addConstrs((O[k] >= 0 for k in K), name="Constraint15")
    FRP.addConstrs((T[i] >= 0 for i in N if i != ign), name="Constraint16")
    FRP.addConstrs((X[(i,j)] >= 0 for (i,j) in A), name="Constraint17a")
    FRP.addConstrs((S[(i,j)] >= 0 for (i,j) in A), name="Constraint17b")
    # Q, Y, Z binary

    # OBJECTIVE FUNCTION
    FRP.setObjective(gb.quicksum(Y[i,h] for i in N) + eps*gb.quicksum(Z[i,k,r] for i in N for k in K for r in R), gb.GRB.MINIMIZE)

    return FRP


def optimize_and_write_model(dataset, setup):
    time_horizon = 28
    if setup:
        n_resources = "5"
    else:
        n_resources = "6"
    # SOLVE
    reduced_dataset = [dict for dict in dataset if dict["instance"] <= 24]
    dataset = reduced_dataset
    for dict in dataset:
        if dict["instance"] <= 17 and setup==True:
            continue
        print("Creating and Optimizing Model for Instance #" + str(dict["instance"]) + "...")
        t_0 = time.time()
        FRP = fire_resource_placement_model(dict, time_horizon, setup1=setup)
        FRP.setParam('TimeLimit', 7200)
        FRP.optimize()
        t = time.time() - t_0
        if FRP.status == gb.GRB.OPTIMAL:
            dataset[dict["instance"] - 1]["OFV"] = str(FRP.objVal)
            dataset[dict["instance"] - 1]["LB"] = str(FRP.ObjBound)
            # dataset[dict["instance" - 1]]["OLB"] = FRP.getAttr('RootLowerBnd')
            names_to_retrieve = (f"Y[{i},{time_horizon}]" for i in dataset[dict["instance"] - 1]["graph"].nodes())
            Y = [FRP.getVarByName(name) for name in names_to_retrieve]
            dataset[dict["instance"] - 1]["BurnedNodes"] = str(gb.quicksum(Y[i].X for i in range(len(Y))))
            dataset[dict["instance"] - 1]["Solution"] = Y
            dataset[dict["instance"] - 1]["NodeCount"] = str(FRP.NodeCount)
            dataset[dict["instance"] - 1]["SolCount"] = str(FRP.SolCount)
            dataset[dict["instance"] - 1]["Runime"] = str(FRP.Runtime)
            dataset[dict["instance"] - 1]["TotalTime"] = t
        else:
            dataset[dict["instance"] - 1]["OFV"] = ""
            dataset[dict["instance"] - 1]["LB"] = str(FRP.ObjBound)
            # dataset[dict["instance"]]["OLB"] = FRP.getAttr('RootLowerBnd')
            dataset[dict["instance"] - 1]["BurnedNodes"] = ""
            dataset[dict["instance"] - 1]["Solution"] = ""
            dataset[dict["instance"] - 1]["NodeCount"] = str(FRP.NodeCount)
            dataset[dict["instance"] - 1]["SolCount"] = str(FRP.SolCount)
            dataset[dict["instance"] - 1]["Runtime"] = str(FRP.Runtime)
            dataset[dict["instance"] - 1]["TotalTime"] = t

        if dict["instance"] == 1:
            with open("dataset_optimised_" + str(n_resources) + "resources.csv", "w", newline="") as fp:
                writer = csv.DictWriter(fp, fieldnames=dataset[dict["instance"] - 1].keys())
                writer.writeheader()
                writer.writerow(dataset[dict["instance"] - 1])
        else:
            with open("dataset_optimised_" + str(n_resources) + "resources.csv", "a", newline="") as fp:
                writer = csv.DictWriter(fp, fieldnames=dataset[dict["instance"] - 1].keys())
                writer.writerow(dataset[dict["instance"] - 1])
        FRP.write("./Results/Models/FRP" + str(n_resources) + "resources_" + str(dict["instance"]) + "instance.mps")
        if FRP.status == gb.GRB.OPTIMAL:
            FRP.write("./Results/Models/FRP" + str(n_resources) + "resources_" + str(dict["instance"]) + "instance.sol")

    print(dataset)
    with open('dataset_optimised_' + str(n_resources) + 'resources.pickle', 'wb') as file:
        pickle.dump(dataset, file)


if __name__ == '__main__':
    with open('dataset.pickle', 'rb') as file:
        dataset = pickle.load(file)
    # Set whether to use setup1 (true - 5 resources) or setup2 (false - 6 resources)
    setup = True
    optimize_and_write_model(dataset, setup) # 5 resources

    with open('dataset.pickle', 'rb') as file:
        dataset = pickle.load(file)
    # Set whether to use setup1 (true - 5 resources) or setup2 (false - 6 resources)
    setup = False
    # SOLVE
    optimize_and_write_model(dataset, setup)  # 6 resources