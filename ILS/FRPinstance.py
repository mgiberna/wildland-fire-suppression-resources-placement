import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
first_deployment_instant = 15
second_deployment_instant = 10

class FRPInstance:
    def __init__(self, size, ignition_node, graph, n_resources, delta=50, time_horizon=28):
        self.size = size
        self.time_horizon = time_horizon
        self.ignition_node = ignition_node
        #self.graph = nx.grid_2d_graph(size, size, create_using=nx.DiGraph())
        #self.graph.add_edges_from(graph.edges(data=True))
        self.graph = graph
        self.updated_graph = self.graph.copy(as_view=False)
        self.n_resources = n_resources
        self.resources = [i for i in range(1, n_resources + 1)]
        self.delta = delta
        self.n_resources_time = [0 for _ in range(time_horizon)]
        self.n_resources_time[second_deployment_instant] = 3
        self.n_resources_time[first_deployment_instant] = len(self.resources) - 3
        self.solution = np.zeros((size, size, time_horizon)) #FRP.solution[i,j,k] solution represent whether at time k a resources has been placed on (i,j)
        self.fire_arrivals = nx.single_source_dijkstra_path_length(self.updated_graph, self.ignition_node, weight='weight')
        self.value = ""
        self.OFV_CH = ""
        self.OFV_LS = ""
        self.OFV_ILS = ""

    def __repr__(self):
        return f"(graph = {self.graph}, n_resources = {self.n_resources}, available resources = {self.resources}, " \
               f"available resources for time = {self.n_resources_time}, delta = {self.delta}, size = {self.size}, " \
               f"ignition_node = {self.ignition_node}, solution = {self.solution}), fire_arrivals = {self.fire_arrivals}, "\
               f"value = {self.value}, OFV_CH = {self.OFV_CH}, OFV_LS = {self.OFV_LS}, OFV_ILS = {self.OFV_ILS}"

    def copy(self):
        new_FRP = FRPInstance(self.size, self.ignition_node, self.graph, self.n_resources, self.delta, self.time_horizon)
        new_FRP.updated_graph = self.updated_graph
        new_FRP.resources = self.resources
        new_FRP.n_resources_time = self.n_resources_time
        new_FRP.solution = self.solution
        new_FRP.value = self.value
        new_FRP.OFV_CH = self.OFV_CH
        new_FRP.OFV_LS = self.OFV_LS
        new_FRP.OFV_ILS = self.OFV_ILS
        return new_FRP

    def add_resource(self, i,j,k):
        self.solution[i, j, k] = self.resources[-1]
        if self.n_resources == 5:
            if self.resources[-1] >= 3:
                self.n_resources_time[second_deployment_instant] = self.n_resources_time[second_deployment_instant] - 1
            else:
                self.n_resources_time[first_deployment_instant] = self.n_resources_time[first_deployment_instant] - 1
        elif self.n_resources == 6:
            if self.resources[-1] >= 4:
                self.n_resources_time[14] = self.n_resources_time[14] - 1
            else:
                self.n_resources_time[first_deployment_instant] = self.n_resources_time[first_deployment_instant] - 1
        self.resources.pop()

    def remove_resource(self, i,j,k):
        self.resources.append(self.solution[i, j, k])
        if self.n_resources == 5:
            if self.resources[-1] >= 3:
                self.n_resources_time[second_deployment_instant] = self.n_resources_time[second_deployment_instant] + 1
            else:
                self.n_resources_time[first_deployment_instant] = self.n_resources_time[first_deployment_instant] + 1
        elif self.n_resources == 6:
            if self.resources[-1] >= 4:
                self.n_resources_time[second_deployment_instant] = self.n_resources_time[second_deployment_instant] + 1
            else:
                self.n_resources_time[first_deployment_instant] = self.n_resources_time[first_deployment_instant] + 1
        self.solution[i, j, k] = 0

    def update_fire_propagation_times(self, i,j,k, after_removing=True):
        if after_removing:
            for neighbor_node in self.updated_graph.neighbors((i, j)):
                self.updated_graph[(i, j)][neighbor_node]['weight'] = self.updated_graph[(i, j)][neighbor_node]['weight'] - self.delta
        else:
            for neighbor_node in self.updated_graph.neighbors((i, j)):
                self.updated_graph[(i, j)][neighbor_node]['weight'] = self.updated_graph[(i, j)][neighbor_node]['weight'] + self.delta

    def update_fire_arrival_times(self):
        self.fire_arrivals = nx.single_source_dijkstra_path_length(self.updated_graph, self.ignition_node, weight='weight')

    def update_current_value(self):
        eps = 1 / self.n_resources
        self.update_fire_arrival_times()
        burned_nodes_at_h = sum(1 for time in self.fire_arrivals.values() if time <= self.time_horizon)
        # do not count nodes where resource placed as burnt
        resource_indexes = np.nonzero(self.solution)
        resource_set = {(node[0], node[1]) for node in zip(resource_indexes[0], resource_indexes[1])}
        not_burned_nodes = sum(1 for node in resource_set if self.fire_arrivals[node] <= self.time_horizon)
        burned_nodes_at_h = burned_nodes_at_h - not_burned_nodes
        number_resources_used = np.count_nonzero(self.solution)
        self.value = burned_nodes_at_h + eps * number_resources_used

    def reset(self):
        new_FRP = FRPInstance(self.size, self.ignition_node, self.graph, self.n_resources, self.delta, self.time_horizon)
        return new_FRP

    def get_value(self):
        self.update_current_value()
        return self.value

    def get_OFV_CH(self):
        return self.OFV_CH

    def get_OFV_LS(self):
        return self.OFV_LS

    def get_OFV_ILS(self):
        return self.OFV_ILS

    def show(self):
        indexes = np.nonzero(self.solution)
        resource_set = {(node[0],node[1]) for node in zip(indexes[0], indexes[1])}
        burned_set = {node for node in self.graph.nodes() if self.fire_arrivals[node] < self.time_horizon}
        resource_color = 'blue'
        burned_color = 'red'
        default_color = 'gray'
        ignition_color = 'yellow'
        node_colors = []
        for node in self.graph.nodes():
            if node in resource_set:
                node_colors.append(resource_color)
            elif node == self.ignition_node:
                node_colors.append(ignition_color)
            elif node in burned_set:
                node_colors.append(burned_color)
            else:
                node_colors.append(default_color)
        pos = {(x, y): (x, y) for x in range(self.size) for y in range(self.size)}
        plt.figure(figsize=(8, 6))
        nx.draw(self.updated_graph, pos, with_labels=True, node_size=800, node_color=node_colors)
        nx.draw_networkx_labels(self.updated_graph, pos, labels=self.fire_arrivals, font_size=12, font_color='red', font_weight='bold')
        plt.title("Graph with Fire Arrival Times")
        plt.axis('equal')
        text_x, text_y = -0.2 * self.size, 0.95 * self.size
        text = "OFV = " + str(self.get_value())
        plt.text(text_x, text_y, text, fontsize=10, color='blue', ha='center')
        text_x, text_y = -0.2 * self.size, 0.90 * self.size
        text = str(len(self.resources)) + " resources unused"
        plt.text(text_x, text_y, text, fontsize=10, color='blue', ha='center')
        text_x, text_y = -0.2 * self.size, 0.85 * self.size
        text = "Burned Nodes = " + str(sum(1 for time in self.fire_arrivals.values() if time < self.time_horizon))
        plt.text(text_x, text_y, text, fontsize=10, color='blue', ha='center')
        plt.show()
        #plt.show(block=True)

