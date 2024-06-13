#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse
from map_class import Map
from ant_colony import AntColony
import time

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
    down_point_view = ( imax-1 if posAnt[0] + antRadius >= imax else posAnt[0] + antRadius+1,
                        jmax-1 if posAnt[1] + antRadius >= jmax else posAnt[1] + antRadius+1)

    if(down_point_view[0]>= map_occupancygrid.shape[0]):
        if(down_point_view[1]>= map_occupancygrid.shape[1]):
            down_point_view = (map_occupancygrid.shape[0]-1,map_occupancygrid.shape[1]-1)
        else:
            down_point_view = (map_occupancygrid.shape[0]-1,map_occupancygrid.shape[1])
    if(down_point_view[1]>= map_occupancygrid.shape[1]):
            down_point_view = (map_occupancygrid.shape[0],map_occupancygrid.shape[1]-1)
    
    new_world = [[0 for _ in range(map_occupancygrid.shape[1])] for _ in range(map_occupancygrid.shape[0])]
    
    for i in range(up_point_view[0],down_point_view[0]+1):
        for j in range(up_point_view[1],down_point_view[1]+1):
            if(i == down_point_view[0] or j == down_point_view[1] ):
                t_i = map_occupancygrid.shape[0]-1 if i >= map_occupancygrid.shape[0] else i
                t_j = map_occupancygrid.shape[1]-1 if j >= map_occupancygrid.shape[1] else j 
                
                max_i  = down_point_view[0] if posAnt[0]+ (antRadius-3) >= down_point_view[0] else posAnt[0]+ (antRadius-3)
                max_j  = down_point_view[1] if posAnt[1]+ (antRadius-3) >= down_point_view[1] else posAnt[1]+ (antRadius-3)
                if(max_i > i or max_j > j):
                    new_world[max_i][max_j]  = map_occupancygrid[max_i][max_j]
                else:
                    new_world[t_i][t_j] =  0 if posAnt[0]+1 > i or posAnt[1]+1 > j else 1
            else:
                new_world[i][j] = map_occupancygrid[i][j] 
    
    if(down_point_view[0] <= 98 or down_point_view[1] <= 98 ):    
        new_world[down_point_view[0]][down_point_view[1]] = 1
        new_world[down_point_view[0]-1][down_point_view[1]] = 1
        new_world[down_point_view[0]-2][down_point_view[1]] = 1
        new_world[down_point_view[0]][down_point_view[1]-1] = 1
        new_world[down_point_view[0]][down_point_view[1]-2] = 1
    
    newOccupancy = np.copy(new_world)

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
    #plt.show()
    plt.pause(0.1)
    #plt.close("Figure 1")
    
def represent_path_static(path, occupancy_map, initial_node, final_node):
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
    plt.pause(30)
    #plt.close("Figure 1")


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
    
    try:
        start_time = time.time()

        # Get the map
        rad_vision = 5
        in_map = _read_map(map_path)
        occupancy_map = _map_2_occupancy_map(in_map)
        initial_node = (int(np.where(in_map == 'S')[0]), int(np.where(in_map == 'S')[1]))
        final_node = (int(np.where(in_map == 'F')[0]), int(np.where(in_map == 'F')[1]))
        
        it = 0
        pos_ant = initial_node
        real_path = [initial_node]
        while(it<100):
            new_map = getSubOptimal(occupancy_map, 5, pos_ant, final_node)
            
            colony = AntColony(new_map, ants, iterations, p, Q)
            path = colony.calculate_path()
            
            # ot_path = smooth_path(path)
            
            for point in path:
                if(point[0] > ( (rad_vision-1) + pos_ant[0]) or point[1]> ((rad_vision-1) + pos_ant[1]-1 )):
                    break
                real_path.append(point)
            
            pos_converted = real_path[-1]
            
            if(pos_converted == final_node):
                break
            
            pos_ant = pos_converted
            print(pos_ant)
            
            represent_path(real_path, occupancy_map, initial_node, final_node)
            it += 1
    except:
        print("nao convergiu")
    
    print(real_path)
    print(f"tamanho -> {len(real_path)}")
    
    print("--- %s seconds ---" % (time.time() - start_time))
    
    represent_path_static(real_path, occupancy_map, initial_node, final_node)
    