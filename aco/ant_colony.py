#!/usr/bin/env python

import numpy as np

class AntColony:
    ''' Class used for handling the behaviour of the whole ant colony '''

    class Ant:
        ''' Class used for handling the ant's individual behaviour '''

        def __init__(self, start_node_pos, final_node_pos):
            self.start_pos = start_node_pos
            self.actual_node = start_node_pos
            self.final_node = final_node_pos
            self.visited_nodes = []
            self.final_node_reached = False
            self.remember_visited_node(start_node_pos)

        def move_ant(self, node_to_visit):
            ''' Moves ant to the selected node '''
            self.actual_node = node_to_visit
            self.remember_visited_node(node_to_visit)

        def remember_visited_node(self, node_pos):
            ''' Appends the visited node to the list of visited nodes '''
            self.visited_nodes.append(node_pos)

        def get_visited_nodes(self):
            ''' Returns the list of visited nodes '''
            return self.visited_nodes

        def is_final_node_reached(self):
            ''' Checks if the ant has reached the final destination '''
            if self.actual_node == self.final_node:
                self.final_node_reached = True

        def enable_start_new_path(self):
            ''' Enables a new path search setting the final_node_reached variable to false '''
            self.final_node_reached = False

        def setup_ant(self):
            ''' Clears the list of visited nodes, stores the first one, and selects the first one as initial '''
            self.visited_nodes[1:] = []
            self.actual_node = self.start_pos

    def __init__(self, in_map, no_ants, iterations, evaporation_factor, pheromone_adding_constant):
        self.map = in_map
        self.no_ants = no_ants
        self.iterations = iterations
        self.evaporation_factor = evaporation_factor
        self.pheromone_adding_constant = pheromone_adding_constant
        self.paths = []
        self.ants = self.create_ants()
        self.best_result = []

    def create_ants(self):
        ''' Creates a list containing the total number of ants specified in the initial node '''
        return [self.Ant(self.map.initial_node, self.map.final_node) for _ in range(self.no_ants)]

    def select_next_node(self, actual_node):
        ''' Randomly selects the next node to visit '''
        total_sum = sum(edge['Pheromone'] for edge in actual_node.edges)
        for edge in actual_node.edges:
            edge['Probability'] = edge['Pheromone'] / total_sum
        chosen_edge = np.random.choice(actual_node.edges, p=[edge['Probability'] for edge in actual_node.edges])
        for edge in actual_node.edges:
            edge['Probability'] = 0.0
        return chosen_edge['FinalNode']

    def pheromone_update(self):
        ''' Updates the pheromone level of each of the trails and sorts the paths by length '''
        self.sort_paths()
        for path in self.paths:
            for j, element in enumerate(path):
                for edge in self.map.nodes_array[element[0]][element[1]].edges:
                    if (j + 1) < len(path) and edge['FinalNode'] == path[j + 1]:
                        edge['Pheromone'] = (1.0 - self.evaporation_factor) * edge['Pheromone'] + self.pheromone_adding_constant / float(len(path))
                    else:
                        edge['Pheromone'] = (1.0 - self.evaporation_factor) * edge['Pheromone']

    def empty_paths(self):
        ''' Empty the list of paths '''
        self.paths.clear()

    def sort_paths(self):
        ''' Sorts the paths '''
        self.paths.sort(key=len)

    def add_to_path_results(self, in_path):
        ''' Appends the path to the results path list '''
        self.paths.append(in_path)

    def get_coincidence_indices(self, lst, element):
        ''' Gets the indices of the coincidences of elements in the path '''
        result = []
        offset = -1
        while True:
            try:
                offset = lst.index(element, offset + 1)
            except ValueError:
                return result
            result.append(offset)

    def delete_loops(self, in_path):
        ''' Checks if there is a loop in the resulting path and deletes it '''
        res_path = list(in_path)
        for element in res_path:
            coincidences = self.get_coincidence_indices(res_path, element)
            coincidences.reverse()
            for i, coincidence in enumerate(coincidences):
                if i != len(coincidences) - 1:
                    res_path[coincidences[i + 1]:coincidence] = []
        return res_path

    def calculate_path(self):
        ''' Carries out the process to get the best path '''
        for i in range(self.iterations):
            for ant in self.ants:
                ant.setup_ant()
                while not ant.final_node_reached:
                    node_to_visit = self.select_next_node(self.map.nodes_array[int(ant.actual_node[0])][int(ant.actual_node[1])])
                    ant.move_ant(node_to_visit)
                    ant.is_final_node_reached()
                self.add_to_path_results(self.delete_loops(ant.get_visited_nodes()))
                ant.enable_start_new_path()
            self.pheromone_update()
            self.best_result = self.paths[0]
            self.empty_paths()
            print(f'Iteration: {i}, length of the path: {len(self.best_result)}')
        return self.best_result
