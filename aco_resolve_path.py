import numpy as np
import sys
import argparse
import matplotlib.pyplot as plt
from map_class import Map
from ant_colony import AntColony

def arguments_parsing():
    ''' Function used for handling the command line argument options '''
    parser = argparse.ArgumentParser()
    parser.add_argument('ants', help='the number of ants that made up the colony', type=int)
    parser.add_argument('iterations', help='the number of iterations to be performed by the algorithm', type=int)
    parser.add_argument('map', help='the map to calculate the path from', type=str)
    parser.add_argument('p', help='controls the amount of pheromone that is evaporated, range[0-1], precision 0.05', type=float, choices=np.around(np.arange(0.0, 1.05, 0.05), decimals=2))
    parser.add_argument('Q', help='controls the amount of pheromone that is added', type=float)
    parser.add_argument('-d', '--display', default=0, action='count', help='display the map and the resulting path')
    args = parser.parse_args()
    return args.ants, args.iterations, args.map, args.p, args.Q, args.display

def smooth_path(path):
    ''' Function to smooth the path using simple averaging method '''
    smoothed_path = [path[0]]
    for i in range(1, len(path) - 1):
        prev_node = path[i - 1]
        next_node = path[i + 1]
        new_node = ((prev_node[0] + next_node[0]) // 2, (prev_node[1] + next_node[1]) // 2)
        smoothed_path.append(new_node)
    smoothed_path.append(path[-1])
    return smoothed_path

if __name__ == '__main__':
    ants, iterations, map_path, p, Q, display = arguments_parsing()
    # Get the map
    map_obj = Map(map_path)
    colony = AntColony(map_obj, ants, iterations, p, Q)
    path = colony.calculate_path()
    print("Original Path:", path)
    if display > 0:
        map_obj.represent_path(path)
    
    smoothed_path = smooth_path(path)
    print("Smoothed Path:", smoothed_path)
    
    if display > 0:
        plt.plot([node[1] for node in path], [node[0] for node in path], 'b-', label='Original Path')
        plt.plot([node[1] for node in smoothed_path], [node[0] for node in smoothed_path], 'r-', label='Smoothed Path')
        map_obj.represent_map()
        plt.legend()
        plt.show()
