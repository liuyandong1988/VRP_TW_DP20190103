#!/usr/bin/env
# -*- coding: UTF-8 -*-
import base.datamapping as dm 
import base.instance_problem as inst
import base.algorithm as alg
import copy   
import random
import numpy as np
import Initial_CIM
import Initial_RSCIM

def printNetworkID(network):
    id_list = []
    for node in network:
        id_list.append(node.id)
    print ('Network id:', id_list) 

def generatePopuRSCIM(size, instance):
    population = []
    for i in range(size):
        rscim_instance = copy.deepcopy(instance)
        init_instance = Initial_RSCIM.RSCIM(rscim_instance)
        individual = init_instance.run()
        population.append(individual)
    return population
     
def check_route_feasible(route, network):
    '''
    return a feasible route
    '''
    route = route [1:-1]
#     printNetworkID(route)
#     input('prompt')
    network_id = []
    remove_node = []
    for node in network:
        network_id.append(node.id)
    
    for node in route:
        if node.id not in network_id:
            # not the same customer node
            pass
        else:
            remove_node.append(node)
        
    for node in remove_node:
        route.remove(node)
    return route

def initialPopuCrossover(popu_MPCIM, popu_RSCIM, size, instance):  
    init_population = [] 
    node_network = instance.network
    vehicle_capacity = instance.fleet.fleet[0].capacity
    distance_matrix = instance.distance_matrix
    for i in range(size):
        crossover_paths = []
        current_network = []
        depot = node_network[0] 
        remain_network = node_network[1:]  # delete the depot node
        indivi_mpcim = copy.deepcopy(random.choice(popu_MPCIM))
        indivi_rscim = copy.deepcopy(random.choice(popu_RSCIM))
        vehicles_mpcim = indivi_mpcim.solution.fleet.fleet
        vehicles_rscim = indivi_rscim.solution.fleet.fleet
#         print ('!!',len(vehicles_mpcim),len(vehicles_rscim))
        routes_num = len(indivi_rscim.solution.fleet.fleet)
        mpcim_mark = 1
        rscim_mark = 0
        
        while (len(crossover_paths) < routes_num) :
            if vehicles_mpcim and mpcim_mark == 1:
                vehicle = random.choice(vehicles_mpcim)
                route = vehicle.route
#                 printNetworkID(route)
#                 printNetworkID(current_network)
                route = check_route_feasible(route, current_network)
                if route != []:
                    remain_network =  [item for item in remain_network if item not in route] 
                    current_network.extend(route)
                    route.insert(0,depot)
                    route.append(depot)
                    crossover_paths.append(route)
#                     print('111')
#                 print ('mpcim Route......')
#                 printNetworkID(route)
#                 print ('Current network......')
#                 printNetworkID(current_network)
#                 printNetworkID(route)
                vehicles_mpcim.remove(vehicle)
                mpcim_mark = 0
                rscim_mark = 1
                continue
            if vehicles_rscim and rscim_mark == 1: 
                vehicle = random.choice(vehicles_rscim)
                route = vehicle.route
#                 printNetworkID(route)
#                 printNetworkID(current_network)
                route = check_route_feasible(route, current_network)
#                 printNetworkID(route)
                if route != []:
                    remain_network = [item for item in remain_network if item not in route]  
                    current_network.extend(route)
                    route.insert(0,depot)
                    route.append(depot)
                    crossover_paths.append(route)
#                     print('222')
#                 print ('rscim Route......')
#                 printNetworkID(route)
#                 print ('Current network......')
#                 printNetworkID(current_network)
        
                vehicles_rscim.remove(vehicle)
                mpcim_mark = 1
                rscim_mark = 0
                continue
#         print ('Allocate the node number:%s'%len(current_network))
#         printNetworkID(current_network)
#         print ('Remain the node number:%s'%len(remain_network))
#         printNetworkID(remain_network)
        
        # put the remain nodes into crossover_paths
        all_paths = {}
        for index, path in enumerate(crossover_paths):
            if remain_network == []:
                all_paths[index] = path
                continue
                
#             printNetworkID(path)
#             input('prompt')
            path_demand = [node.demand for node in path]
            current_load = sum(path_demand)
#             print (current_load)
#             input('prompt')
            for node in remain_network:
                if (node.demand + current_load) <= vehicle_capacity:
                    current_load  += node.demand
                    if len(path) == 1:
                        path.append(node)
                    else:
                        path = insertNode(path, node, distance_matrix, depot)
                else:
#                     printNetworkID(path)
                    all_paths[index] = path
                    remain_network = [item for item in remain_network if item not in path]
                    break
#             printNetworkID(path)
            all_paths[index] = path
            remain_network = [item for item in remain_network if item not in path]
#         # print the result
        print ('-'*30)
        for ind, pa in all_paths.items():
            pa = pa[1:-1]
            for node in pa:
                node.visited = False
            all_paths[ind] = pa 
#             printNetworkID(pa)
#         input('prompt')
        print('*'*10)
        new_instance = copy.deepcopy(instance)
        fleet = new_instance.fleet
        # allocate the paths to the vehicles
        vehicle_id = 0
        for index, path in all_paths.items():
#             printNetworkID(path)
            
            for node in path:
                if not node.visited:
                    try:
                        # add node and update the capacity
#                         print (fleet.fleet[vehicle_id].load)
                        fleet.fleet[vehicle_id].add_node(node)
                    except ValueError:
                        continue
#             printNetworkID(fleet.fleet[vehicle_id].route)
            vehicle_id += 1
        for vehicle in fleet:
            vehicle.route.insert_node(0, depot)
            vehicle.route.append_node(depot)
            path = []
            for i in vehicle.route.route:
                path.append(i.id)
            print ( 'The vehicle %s route:%s' %(vehicle.id, path))
        init_instance_solution = alg.Solution(new_instance, [depot.id]) 
#         print (init_instance_solution.solution.fleet.fleet[0].route[0].id,[depot.id])
        print ('Total distance:', init_instance_solution.eval())
#         input('prompt')
        init_population.append(init_instance_solution)
    return init_population
            
def insertNode(path, node, distance_matrix, depot):  
    '''
    Based on the distance insert node
    '''
    #         print ('Node:', node.id)
    #         print ('Path add')  
    #         printNetworkID(path)
    change_distance = 0
    cost_value = []
    for i,j in zip(path[:-1], path[1:]):
        change_distance = distance_matrix[depot.id-1][node.id-1] + distance_matrix[node.id-1][depot.id-1] + distance_matrix[i.id-1][j.id-1] -\
              (distance_matrix[i.id-1][node.id-1] - distance_matrix[node.id-1][j.id-1])              
    cost_value.append(change_distance)
    path.insert(cost_value.index(min(cost_value))+1, node)
    #         printNetworkID(path)
    return path       

if __name__ == '__main__':
    #the data file
    problem_path = 'A-n32-k5.vrp'
    raw_data = dm.Importer()
    raw_data.import_data(problem_path)
    data = dm.DataMapper(raw_data)
    instance = inst.ProblemInstance(data, raw_data.distance_matrix) 
    cim_instance = copy.deepcopy(instance)
    # Multi-parameter CIM beta [0.2~1.4]
    population_MPCIM = Initial_CIM.mpCIM(instance)
    print ('Use the MPCIM gets population size: %s'%len(population_MPCIM))
    # RSCIM gets population
    pop_rscim_size = 7
    population_RSCIM = generatePopuRSCIM(pop_rscim_size, instance)
    print ('Use the RSCIM gets population size: %s' %len(population_RSCIM))
    # crossover mechanism with MPCIM and RSCIM get N population (N = 50)
    total_popu_size = 50
    total_population = []
    total_population.extend(population_MPCIM)
    total_population.extend(population_RSCIM)
    remain_popu_size = total_popu_size - len(population_MPCIM) - len(population_RSCIM)
    print ('Generate the size of population:%s'%remain_popu_size)
    crossover_population =  initialPopuCrossover(population_MPCIM, population_RSCIM, remain_popu_size, instance)
    total_population.extend( crossover_population  ) 
    print ('Total population:%s'%len(total_population))  
     
 

        
 