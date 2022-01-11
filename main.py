#import required libraries
import json
import math
from queue import PriorityQueue
import time


#function for getting path
def get_path(parent, target):
    vertexes = target
    path_array =[]
    while vertexes is not None:
        path_array.append(vertexes)
        vertexes = parent[vertexes]
    return path_array[::-1]

#function for displaying path
def display_path(path_list):
    for i in range(len(path_list)):
        if i == len(path_list) - 1:
            print(path_list[i])
            break
        print(path_list[i]+" -> ", end="")
        
#function for calculating cost
def cal_cost(path):
    path_cost = 0
    for index in range(len(path)-1):
        path_cost += costs[(path[index]+','+path[index+1])] #accessing Cost json 
    return path_cost

def cal_dist(path):
    path_dist = 0
    for index in range(len(path) - 1):
        path_dist += dists[(path[index] + "," + path[index + 1])]
    return path_dist


def dijkstra(source, target):
    print("--- Running Dijkstra Algorithm ---")
     
    vertexes = 264346 #total number of vertexes
    
    visited_node = set()
    distance = {source: 0}
    parent_vertex = {source: None}

    priorityQueue = PriorityQueue()
    priorityQueue.put((0, source))
    
    while not priorityQueue.empty():
        (_, current) = priorityQueue.get()
        
        if current not in visited_node:
            visited_node.add(current)
        
        if current == target:
            break
            
        for neighbour in graph.get(str(current)):
            old_cost = distance.get(neighbour, float('inf'))
            new_cost = distance[current] + dists[str(current)+','+str(neighbour)]
            
            if new_cost < old_cost:
                priorityQueue.put((new_cost, neighbour))
                distance[neighbour] = new_cost
                parent_vertex[neighbour] = current
            
    if target not in parent_vertex.keys():
        return None
    else:
        Distance = distance[target]
        return parent_vertex,Distance


def dijkstra_algo():
    #Check if user input valid parameteres
    while True:
        Source = str(input("Select source node: "))
        if Source not in graph.keys():
            print("Invalid node entered. Enter a number btwn 1-264346.")
        else:
            break
        
    while True:
        Target = str(input("Select target node: "))
        if Target not in graph.keys():
            print("Invalid node entered. Enter a number btwn 1-264346.")
        else:
            break  
    
    vertexes = 264346
    
    start_time = time.time()
    parent,Distance = dijkstra(Source, Target)
    
    if parent != None:
        path = get_path(parent, Target)
        #print("Shortest Path: " + " -> ".join(path) + "\n")
        print('Shortest path: ' + "\n")
        display_path(path)
        print('Shortest distance: %s' %str(Distance))
        print('Total energy cost: ', cal_cost(path))
        print("Time taken:\t%s seconds" % (time.time() - start_time))
        print("--- End of Shortest Path Search - Dijkstra Algorithm ---")
    else:
        print(f"No valid path between node {Source} and node {Target}.")
        print("--- End of Shortest Path Search - Dijkstra Algorithm ---")


def UCS(source, target,energy_constrain = 287932):
    print("--- Running UCS - w/ Energy Constrain ---")
    
    priorityQueue = PriorityQueue()
    priorityQueue.put((0, 0, source, [source]))
    visited_node = []
    
    total_cost =0
    total_dist =0
    min_cost = 10000000
    min_dist = 10000000
    min_path = []

    
    while not priorityQueue.empty():
        dist_from_current, cost_from_current, current, path = priorityQueue.get()
        
        if current not in visited_node:
            visited_node.append(current)
        
            if current == target:
                if cal_cost(path) > energy_constrain:
                    visited_node.remove(current)
                else:
                    if min_dist >= cal_dist(path):
                        min_path = list(path)
                        min_dist = dist_from_current
                    visited_node.remove(current)
            else: 
                neighbours = graph.get(str(current))

                for neighbour in neighbours:
                    neighbour_dist = dists[current+','+neighbour]
                    neighbour_cost = costs[current+','+neighbour]
                    neighbour_path = list(path)
                    neighbour_path.append(neighbour)

                    if cost_from_current + neighbour_cost <= energy_constrain:
                        priorityQueue.put((neighbour_dist + dist_from_current, neighbour_cost+cost_from_current, neighbour, neighbour_path))
    return min_path


def UCS_algo():
    #Check if user input valid parameteres
    while True:
        Source = str(input("Select source node: "))
        if Source not in graph.keys():
            print("Invalid node entered. Enter a number btwn 1-264346.")
        else:
            break
        
    while True:
        Target = str(input("Select target node: "))
        if Target not in graph.keys():
            print("Invalid node entered. Enter a number btwn 1-264346.")
        else:
            break  
    
    while True:
        Constrain = int(input("Select energy constrain: "))
        break

    start_time = time.time()
    path = UCS(Source, Target,Constrain)
    
    if path != None:
        print('Shortest path: ' + "\n")
        display_path(path)
        print('Shortest distance: ',cal_dist(path))
        print('Total energy cost: ', cal_cost(path))
        print("Time taken:\t%s seconds" % (time.time() - start_time))
        print("--- End of Uniform Cost Search - w/ Energy Constrain ---")
    else:
        print(f"No valid path between node {Source} and node {Target}.")
        print("--- End of Uniform Cost Search - w/ Energy Constrain ---")


def aStar(source, target, energy_constrain):
    print('--- Running aStar - w/ Energy Constrain --- ')
    
    priorityQueue = PriorityQueue()
    priorityQueue.put((0,source,[source]))
    visited_node = []
    
    min_dist = 100000000
    min_path = []
    target_x, target_y = coords[target]
    
    while not priorityQueue.empty():
        total_path_cost, current, path = priorityQueue.get()
        dist_to_current = cal_dist(path)
        cost_to_current = cal_cost(path)
        
        if current not in visited_node:
            visited_node.append(current)
            if current == target:
                if cal_cost(path) > energy_constrain:
                    visited_node.remove(current)
                else:
                    if min_dist >= cal_dist(path):
                        min_path = list(path)
                        min_dist = cal_dist(path)
                    visited_node.remove(current)
            else:
                neighbours = graph[current]
                
                for neighbour in neighbours:
                    neighbour_dist = dists[current +',' + neighbour]
                    neighbour_cost = costs[current+ ','+ neighbour]
                    neighbour_path = list(path)
                    neighbour_path.append(neighbour)
                    
                    dist_to_neighbour = dist_to_current + neighbour_dist
                    cost_to_neighbour = cost_to_current + neighbour_cost
                    
                    neighbour_x, neighbour_y = coords[neighbour]
                    
                    euclid_dist = math.sqrt((neighbour_x - target_x)**2 + (neighbour_y - target_y)**2)
                    
                    if cost_to_neighbour <= energy_constrain:
                        priorityQueue.put((dist_to_neighbour+euclid_dist, neighbour, neighbour_path))
    return min_path


def aStar_algo():
    #Check if user input valid parameteres
    while True:
        Source = str(input("Select source node: "))
        if Source not in graph.keys():
            print("Invalid node entered. Enter a number btwn 1-264346.")
        else:
            break
        
    while True:
        Target = str(input("Select target node: "))
        if Target not in graph.keys():
            print("Invalid node entered. Enter a number btwn 1-264346.")
        else:
            break  
    
    while True:
        Constrain = int(input("Select energy constrain: "))
        break
    
    start_time = time.time()
    path = aStar(Source, Target,Constrain)
    
    if path != None:
        print('Shortest path: ' + "\n")
        display_path(path)
        print('Shortest distance: ',cal_dist(path))
        print('Total energy cost: ', cal_cost(path))
        print("Time taken:\t%s seconds" % (time.time() - start_time))
        print("--- End of aStar Search - w/ Energy Constrain ---")
    else:
        print(f"No valid path between node {Source} and node {Target}.")
        print("--- End of aStar Search - w/ Energy Constrain ---")


if __name__ == "__main__":
    with open('Coord.json') as coord:
        coords = json.load(coord)

    with open('Cost.json') as cost:
        costs =json.load(cost)

    with open('Dist.json') as dist:
        dists = json.load(dist)

    with open('G.json') as G:
        graph = json.load(G)

    while True:
        print('Select an Algorithm \n')
        print('1. Dijkstra Algorithm - Shortest Path w/o Energy Constrain')
        print('2. Uniform Cost Search - with Energy Constrain')
        print('3. aStar Search - with Energy Constrain')
        print('4. Exit')
        select = int(input("Select a choice: "))
        
        if select == 1:
            dijkstra_algo()
        elif select == 2:
            UCS_algo()
        elif select == 3:
            aStar_algo()
        else:
            break

    coord.close()
    cost.close()
    dist.close()
    G.close()