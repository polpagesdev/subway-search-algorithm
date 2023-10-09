# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = 'Pol Pages'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2019- 2020
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy
import unittest
import random


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    path_list = []
    i = path.last

    for j in map.connections[i].keys():
        temp_path = copy.deepcopy(path)
        temp_path.add_route(j)
        path_list.append(temp_path)

    return path_list


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    aux_path_list = []
    for path in path_list:
        aux_path_list.append(path)

    for path in path_list:
        tail = path.route[:-1]
        if path.last in tail:
            aux_path_list.remove(path)

    path_list = aux_path_list

    return path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    list_of_path = expand_paths + list_of_path

    return list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    final = list_of_path[0].last

    while final != destination_id:
        list_head = list_of_path[0]
        head_expanded = expand(list_head, map)
        head_expanded = remove_cycles(head_expanded)
        list_of_path_tail = list_of_path[1:]
        list_of_path = insert_depth_first_search(head_expanded, list_of_path_tail)
        final = list_of_path[0].last

    return list_of_path[0]


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    list_of_path = list_of_path + expand_paths

    return list_of_path


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    final = list_of_path[0].last

    while final != destination_id:
        list_head = list_of_path[0]
        head_expanded = expand(list_head, map)
        head_expanded = remove_cycles(head_expanded)
        list_of_path_tail = list_of_path[1:]
        list_of_path = insert_breadth_first_search(head_expanded, list_of_path_tail)
        final = list_of_path[0].last

    return list_of_path[0]


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    type_preference = int(type_preference)
    cost = 0

    if type_preference == 0:
        for path in expand_paths:
            path.update_g(1)

    elif type_preference == 1:
        for path in expand_paths:
            cost = map.connections[path.penultimate][path.last]
            path.update_g(cost)

    elif type_preference == 2:
        for path in expand_paths:
            actual_pos = [map.stations[path.last]['x'], map.stations[path.last]['y']]
            past_pos = [map.stations[path.penultimate]['x'], map.stations[path.penultimate]['y']]
            distance = euclidean_dist(actual_pos, past_pos)
            if distance == 0:
                cost = 0
                # cost = map.connections[path.penultimate][path.last]
            else:
                line_velocity = map.velocity[map.stations[path.last]['line']]
                real_time = map.connections[path.penultimate][path.last]
                cost = line_velocity * real_time
            path.update_g(cost)

    elif type_preference == 3:
        for path in expand_paths:
            if map.stations[path.penultimate]['line'] == map.stations[path.last]['line']:
                path.update_g(0)
            else:
                path.update_g(1)

    else:
        print("ERROR: TYPE PREFERENCE UNDEFINED")

    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    list_of_path_aux = expand_paths + list_of_path

    list_of_path_aux.sort(key=lambda path: path.g)

    return list_of_path_aux


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    final = list_of_path[0].last
    cost_dct = {}

    while final != destination_id:
        list_head = list_of_path[0]
        head_expanded = expand(list_head, map)
        head_expanded = remove_cycles(head_expanded)
        head_expanded = calculate_cost(head_expanded, map, type_preference)
        head_expanded, list_of_path, cost_dct = remove_redundant_paths(head_expanded, list_of_path, cost_dct)
        if len(list_of_path) == 1:
            list_of_path_tail = []
        else:
            list_of_path_tail = list_of_path[1:]
        list_of_path = insert_cost(head_expanded, list_of_path_tail)
        final = list_of_path[0].last

    return list_of_path[0]


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    destination_pos = [map.stations[destination_id]['x'], map.stations[destination_id]['y']]

    if type_preference == 0:
        for path in expand_paths:
            if path.last == destination_id:
                path.update_h(0)
            else:
                path.update_h(1)

    elif type_preference == 1:
        for path in expand_paths:
            path_pos = [map.stations[path.last]['x'], map.stations[path.last]['y']]
            dist = euclidean_dist(path_pos, destination_pos)
            line_velocity = map.velocity[3]
            heuristics = dist / line_velocity
            path.update_h(heuristics)

    elif type_preference == 2:
        for path in expand_paths:
            path_pos = [map.stations[path.last]['x'], map.stations[path.last]['y']]
            dist = euclidean_dist(path_pos, destination_pos)
            path.update_h(dist)

    elif type_preference == 3:
        for path in expand_paths:
            if map.stations[path.last]['line'] == map.stations[destination_id]['line']:
                path.update_h(0)
            else:
                path.update_h(1)

    else:
        print("ERROR: TYPE PREFERENCE UNDEFINED")

    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g in this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
    """
    expand_paths_list = []
    for path in expand_paths:
        expand_paths_list.append(path)

    for path in expand_paths:
        path_cost = path.g
        if visited_stations_cost.get(path.last, 'NULL') != 'NULL':
            if path_cost < visited_stations_cost[path.last]:
                for redundant_path in list_of_path:
                    if path.last in redundant_path.route:
                        list_of_path.remove(redundant_path)
                visited_stations_cost.update({path.last: path_cost})
            else:
                expand_paths_list.remove(path)
        else:
            visited_stations_cost.update({path.last: path_cost})

    expand_paths = expand_paths_list

    return expand_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    list_of_path_aux = expand_paths + list_of_path

    list_of_path_aux.sort(key=lambda path: path.f)

    return list_of_path_aux


def coord2station(coord, map):
    """
        From coordinates, it searches the closest station.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """
    possible_origins = []
    dist_list = []
    dist_list_sorted = []
    station_coord = []
    i = 1
    while i <= len(map.stations):
        station_coord.clear()
        station_coord.append(map.stations[i]['x'])
        station_coord.append(map.stations[i]['y'])
        min_dist = euclidean_dist(coord, station_coord)
        dist_list.append(min_dist)
        dist_list_sorted.append(min_dist)
        i += 1

    dist_list_sorted.sort()

    min_dist = dist_list_sorted[0]

    i = 1
    for elem in dist_list:
        if min_dist == elem:
            possible_origins.append(i)
        i += 1

    return possible_origins


def Astar(origin_coor, dest_coor, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (list): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    possible_origins = coord2station(origin_coor, map)
    origin = possible_origins[0]
    possible_destinations = coord2station(dest_coor, map)
    destination = possible_destinations[0]
    cost_dct = {}
    list_of_path = [Path(origin)]
    final = list_of_path[0].last

    while final != destination:
        list_head = list_of_path[0]
        head_expanded = expand(list_head, map)
        head_expanded = remove_cycles(head_expanded)
        head_expanded = calculate_heuristics(head_expanded, map, destination, type_preference)
        head_expanded = calculate_cost(head_expanded, map, type_preference)
        head_expanded = update_f(head_expanded)
        head_expanded, list_of_path, cost_dct = remove_redundant_paths(head_expanded, list_of_path, cost_dct)
        if len(list_of_path) == 1:
            list_of_path_tail = []
        else:
            list_of_path_tail = list_of_path[1:]
        list_of_path = insert_cost_f(head_expanded, list_of_path_tail)
        final = list_of_path[0].last

    return list_of_path[0]


def Astar_improved(origin_coor, dest_coor, map, type_preference):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (list): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    possible_origins = coord2station(origin_coor, map)
    possible_destinations = coord2station(dest_coor, map)
    destination = possible_destinations[0]
    cost_dct = {}
    list_of_path = []
    for path in possible_origins:
        list_of_path.append(Path(path))
    final = list_of_path[0].last

    while final != destination:
        list_head = list_of_path[0]
        head_expanded = expand(list_head, map)
        head_expanded = remove_cycles(head_expanded)
        head_expanded = calculate_heuristics(head_expanded, map, destination, type_preference)
        head_expanded = calculate_cost(head_expanded, map, type_preference)
        head_expanded = update_f(head_expanded)
        head_expanded, list_of_path, cost_dct = remove_redundant_paths(head_expanded, list_of_path, cost_dct)
        if len(list_of_path) == 1:
            list_of_path_tail = []
        else:
            list_of_path_tail = list_of_path[1:]
        list_of_path = insert_cost_f(head_expanded, list_of_path_tail)
        final = list_of_path[0].last

    return list_of_path[0]