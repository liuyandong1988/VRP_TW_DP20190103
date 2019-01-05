#!/usr/bin/env
# -*- coding: UTF-8 -*-
import base.datamapping as dm 
import base.instance_problem as inst
import base.algorithm as alg
import copy   
import random
import numpy as np


def printNetworkID(network):
    id_list = []
    for node in network:
        id_list.append(node.id)
    print ('Network id:', id_list) 

class CIM(object):
    '''
    The cheapest insertion method (CIM)
    '''
    def __init__(self, instance, beta = 1):
        self.instance = instance
        self.distance_matrix = instance.distance_matrix
        self.beta = beta
        
    def run(self):
        network = copy.deepcopy(self.instance.network)
        self.depot = network.get_node(1)
        network.remove_node(self.depot)
        #print network id
#         printNetworkID(network)
        random.shuffle(network.network)
        # insert the node based on the distance
        fleet = self.instance.fleet
        vehicle_capacity = fleet.fleet[0].capacity
        remain_network = network.network 
        path = []
        all_paths = []
        current_load = 0
        # plan the path
        while remain_network:
            for node in remain_network:
#                 print ('remain---')
#                 printNetworkID(remain_network)
                if (node.demand + current_load) <= vehicle_capacity:
                    current_load += node.demand
                    if len(path) <= 1 :
                        path.append(node)
                    else:
                        path = self.insertNode(path, node)
#             print ('path')
#             printNetworkID(path)
            all_paths.append(path)
            remain_network = [item for item in remain_network if item not in path]
            path = []
            current_load = 0
#             print ('remain****')
#             printNetworkID(remain_network)
#         print ('******************')
#         for path in all_paths:
#             printNetworkID(path)

        # allocate the paths to the vehicles
        for path in all_paths:
            for node in path:
                if not node.visited:
                    for vehicle in fleet:
                        try:
                            # add node and update the capacity
                            vehicle.add_node(node)
                            break
                        except ValueError:
                            continue
        for vehicle in fleet:
            vehicle.route.insert_node(0, self.depot)
            vehicle.route.append_node(self.depot)
            path = []
            for i in vehicle.route.route:
                path.append(i.id)
#             print ( 'The vehicle %s route:%s' %(vehicle.id, path))
        self.instance = alg.Solution(self.instance, [self.depot.id]) 
#         print ('Total distance:', self.instance.eval())
        return self.instance
        
#         return self.instance 
    
    def insertNode(self, path, node):  
        '''
        Based on the distance insert node
        '''
#         print ('Node:', node.id)
#         print ('Path add')  
#         printNetworkID(path)
        change_distance = 0
        cost_value = []
        for i,j in zip(path[:-1], path[1:]):
            change_distance = self.distance_matrix[self.depot.id-1][node.id-1] + self.distance_matrix[node.id-1][self.depot.id-1] + self.distance_matrix[i.id-1][j.id-1] -\
                              self.beta*(self.distance_matrix[i.id-1][node.id-1] - self.distance_matrix[node.id-1][j.id-1])              
            cost_value.append(change_distance)
        path.insert(cost_value.index(min(cost_value))+1, node)
#         printNetworkID(path)
        return path
    
def mpCIM(instance):
    low = 0.2
    high = 1.4
    population = []
    parameter = np.arange(low, high + 0.2, 0.2)
#     print (parameter)
    for param in parameter:
        mpCIM_instance = copy.deepcopy(instance)
        init_instance = CIM(mpCIM_instance, param)
        individual = init_instance.run()
#         print ('-'*30)
        population.append(individual)
    return population

if __name__ == '__main__':
    #the data file
    problem_path = 'A-n32-k5.vrp'
    raw_data = dm.Importer()
    raw_data.import_data(problem_path)
    data = dm.DataMapper(raw_data)
    instance = inst.ProblemInstance(data, raw_data.distance_matrix) 
    cim_instance = copy.deepcopy(instance)
    init_instance = CIM(cim_instance)
    individual = init_instance.run()
    # Multi-parameter CIM beta [0.2~1.4]
    population = mpCIM(instance)
