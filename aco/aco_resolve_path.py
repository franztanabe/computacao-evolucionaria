#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse
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


def getSubOptimal(full_map, antRadius, posAnt, final_node):
    map_occupancygrid = full_map
    
    imax = map_occupancygrid.shape[0]
    jmax = map_occupancygrid.shape[1]
    
    up_point_view = ( 0 if posAnt[0] - antRadius < 0 else posAnt[0] - antRadius,
                      0 if posAnt[1] - antRadius < 0 else posAnt[1] - antRadius)
    down_point_view = ( imax-1 if posAnt[0] + antRadius >= imax else posAnt[0] + antRadius,
                        jmax-1 if posAnt[1] + antRadius >= jmax else posAnt[1] + antRadius)

    print(up_point_view)
    print(down_point_view)
    print(posAnt)
    
    new_world = [[0 for _ in range(map_occupancygrid.shape[1])] for _ in range(map_occupancygrid.shape[0])]
    
    print(map_occupancygrid.shape)
    for i in range(up_point_view[0],down_point_view[0]+1):
        if(i>= map_occupancygrid.shape[0]): 
            continue
        for j in range(up_point_view[1],down_point_view[1]+1):
            if(j>= map_occupancygrid.shape[1]): 
                continue
            if(i == down_point_view[0] or j == down_point_view[1] ):
                new_world[i][j] =  0 if posAnt[0]+1 > i or posAnt[1]+1 > j else 1
            else:
                new_world[i][j] = map_occupancygrid[i][j] 
        
    
    newOccupancy = np.copy(new_world)
    
    
    plt.plot(posAnt[1], posAnt[0], 'ro', markersize=10)
    plt.plot(down_point_view[1], down_point_view[0], 'bo', markersize=10)
    plt.imshow(newOccupancy, cmap='gray', interpolation='nearest')
    plt.show()
    plt.close()

    ## TODO adptar ponto sub otimo com base no no parametro final de entrada 
    return Map( newOccupancy,
                (posAnt[0],posAnt[1]),
                # getFinalPoint(newOccupancy, final_node)
                (down_point_view[0],down_point_view[1]) 
               )
    

def _read_map(map_name):
    ''' Reads data from an input map txt file'''
    map_planning = np.loadtxt('./maps/' + map_name, dtype=str)
    return map_planning

def _map_2_occupancy_map(in_map):
    ''' Takes the matrix and converts it into a float array '''
    map_arr = np.copy(in_map)
    map_arr[map_arr == 'O'] = 0
    map_arr[map_arr == 'E'] = 1
    map_arr[map_arr == 'S'] = 1
    map_arr[map_arr == 'F'] = 1
    return map_arr.astype(int)


def represent_path(path, occupancy_map, initial_node, final_node):
    ''' Represents the path in the map '''
    x = []
    y = []
    for p in path:
        x.append(p[1])
        y.append(p[0])
    plt.plot(x, y)
    
    plt.plot(initial_node[1], initial_node[0], 'ro', markersize=10)
    plt.plot(final_node[1], final_node[0], 'bo', markersize=10)
    plt.imshow(occupancy_map, cmap='gray', interpolation='nearest')
    plt.show()
    plt.close()


if __name__ == '__main__':
    ants, iterations, map_path, p, Q, display = arguments_parsing()
    # Get the map
    rad_vision = 5
    in_map = _read_map(map_path)
    occupancy_map = _map_2_occupancy_map(in_map)
    initial_node = (int(np.where(in_map == 'S')[0]), int(np.where(in_map == 'S')[1]))
    final_node = (int(np.where(in_map == 'F')[0]), int(np.where(in_map == 'F')[1]))
    
    # map_obj = Map(map_path)
    it = 0
    pos_ant = initial_node
    # pos_ant = (12,13)
    real_path = [initial_node]
    while(it<10):
        #print(f'Iteracao {it}')
        new_map = getSubOptimal(occupancy_map, 5, pos_ant, final_node)
        
        colony = AntColony(new_map, ants, iterations, p, Q)
        path = colony.calculate_path()
        
        print(path)
        
        for point in path:
            if(point[0] > ( (rad_vision-1) + pos_ant[0]) or point[1]> ((rad_vision-1) + pos_ant[1]-1 )):
                break
            real_path.append(point)
        
        pos_converted = real_path[-1]
        
        if(pos_converted == final_node):
            break
        
        pos_ant = pos_converted
        
        
        represent_path(real_path, occupancy_map, initial_node, final_node)
        it += 1
    
    
    represent_path(real_path, occupancy_map, initial_node, final_node)
    