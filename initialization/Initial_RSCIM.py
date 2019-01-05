#!/usr/bin/env
# -*- coding: UTF-8 -*-
import base.datamapping as dm 
import base.instance_problem as inst
import base.algorithm as alg
import copy   
import random
import math


def printNetworkID(network):
    id_list = []
    for node in network:
        id_list.append(node.id)
    print ('Network id:', id_list) 

class RSCIM(object):
    '''
    The random seed  cheapest insertion method (RSCIM)
    Have a wide search range
    '''
    def __init__(self, instance, beta = 1):
        self.instance = instance
        self.distance_matrix = instance.distance_matrix
        self.total_demand = self.calDemand()
        self.vehicle_capacity = self.instance.fleet.fleet[0].capacity 
        self.routes_num = math.ceil(self.total_demand/self.vehicle_capacity)
#         print (self.routes_num)
#         input('prompt')
        self.beta = beta
        
    def run(self):
        network = copy.deepcopy(self.instance.network)
        self.depot = network.get_node(1)
        network.remove_node(self.depot)
        #print network id
#         printNetworkID(network)
        random.shuffle(network.network)
        # find the seed customers nodes
        all_paths = {}
        for i in range(self.routes_num):
            all_paths[i] = [ network.network[i] ]
#         print (all_paths)
#         input('prompt')
        # insert the node based on the distance
        seed_network = network.network[:self.routes_num]
        remain_network = network.network[self.routes_num:]
        for index, path in all_paths.items():
            current_load = path[0].demand
            for node in remain_network:
                if (node.demand + current_load) <= self.vehicle_capacity:
                    current_load  += node.demand
                    if len(path) == 1:
                        path.append(node)
                    else:
                        path = self.insertNode(path, node)
                else:
                    all_paths[index] = path
                    remain_network = [item for item in remain_network if item not in path]
                    break
#         # print the result
#         print (all_paths)
#         for ind, pa in all_paths.items():
#             printNetworkID(pa)
#         input('prompt')

        fleet = self.instance.fleet
        # allocate the paths to the vehicles
        for index, path in all_paths.items():
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
    
    def calDemand(self):
        '''
        Calculate the total demand of all customers
        '''
        total_demand = 0
        network = self.instance.network
        for node in network:
#             print (node.demand)
            total_demand += node.demand
        return total_demand
    

if __name__ == '__main__':
    #the data file
    problem_path = 'A-n32-k5.vrp'
    raw_data = dm.Importer()
    raw_data.import_data(problem_path)
    data = dm.DataMapper(raw_data)
    instance = inst.ProblemInstance(data, raw_data.distance_matrix) 
    rscim_instance = copy.deepcopy(instance)
    init_instance = RSCIM(rscim_instance)
    individual = init_instance.run()
