import base.datamapping as dm 
import base.instance_problem as inst
import base.algorithm as alg
import copy
import initialization.Initial_CIM as Initial_CIM
import initialization.Initial_MPCIM_RSCIM_crossover as initial_crossover
import random, math
import base.TwoOptimal as TwoOptimal 
from numpy import sort

def printNetworkID(network):
    id_list = []
    for node in network:
        id_list.append(node.id)
    print ('Network id:', id_list) 

def printFleets(fleets):
    print ('*Print the solution...')
    length = 0
    for vehicle in fleets:
        length += len(vehicle.route)
        printNetworkID(vehicle.route)
    print ('CUSTOMERS:', length)
    

def inputData(file_name):
    raw_data = dm.Importer()
    raw_data.import_data(file_name)
    data = dm.DataMapper(raw_data)
    instance = inst.ProblemInstance(data, raw_data.distance_matrix) 
    return instance, raw_data.node_coordinates_list

def popInitialization(instance, pop_size):
    population = []
    # Multi-parameter CIM beta [0.2~1.4]
    cim_instance = copy.deepcopy(instance)
    population_MPCIM = Initial_CIM.mpCIM(cim_instance)
    # RSCIM gets population
    rscim_instance = copy.deepcopy(instance)
    pop_rscim_size = 7
    population_RSCIM = initial_crossover.generatePopuRSCIM(pop_rscim_size, rscim_instance)
    population.extend(population_MPCIM)
    population.extend(population_RSCIM)
    # crossover mechanism with MPCIM and RSCIM get N population
    remain_pop_size = pop_size - len(population_MPCIM) - len(population_RSCIM)
    crossover_population =  initial_crossover.initialPopuCrossover(population_MPCIM, population_RSCIM, remain_pop_size, instance)
    population.extend(crossover_population) 
    print ('Total population:%s' %len(population)) 
    return population 
    
class GA(object):
    '''
    Use GA optimizes the solution
    '''   
    def __init__(self, population, iteration, instance, coordination):
        self.parent_size = len(population)
        self.population1 = copy.deepcopy(population)
        self.population2 = copy.deepcopy(population)
        self.iteration = iteration
        self.instance = instance  # instance without allocating the task
        self.distance_matrix = instance.distance_matrix
        self.coordination = coordination
#         best_individual = self.reproducing(population)
#         self.recombination(best_individual)
#         self.selection()
#         self.mutation7(self.best_individual)
#         self.recombination(best_individual)
#         self.localImproved(best_individual)
        
    def reproducing(self, population):
        '''
        choose the best individual in the population.
        '''
        for individual in population:
            individual.value = individual.eval()
#             printFleets(individual.solution.fleet)
        population.sort(key = lambda chromosome: chromosome.value)
#         for individual in population:
#             print ('*', individual.value)
        best_individual = population[0]
        return best_individual

    def insertNode(self, path, node):  
        '''
        Based on the distance insert node
        '''
        change_distance = 0
        cost_value = []
        for i,j in zip(path[:-1], path[1:]):
            change_distance = self.distance_matrix[i.id-1][node.id-1] + self.distance_matrix[node.id-1][j.id-1] \
                                - self.distance_matrix[i.id-1][j.id-1] 
            cost_value.append(change_distance)
#         printNetworkID(path)         
        path.insert(cost_value.index(min(cost_value))+1, node)
#         printNetworkID(path)
#         input('prompt')
        return path   
    
    def generateInstance(self, all_paths, isolated_customers, instance):
        '''
        Generate a new instance by inserting the isolated customers into the route
        '''
        paths_tmp = {}
        for index, path in enumerate(all_paths):
#             printNetworkID(path)
            path_demand = [node.demand for node in path]
            current_load = sum(path_demand)
#             print (current_load)
            for node in isolated_customers:
                if (node.demand + current_load) <= self.vehicle_capacity:
                    current_load  += node.demand
                    if len(path) == 1:
                        path.append(node)
                    else:
                        path = self.insertNode(path, node)
                else:
#                     printNetworkID(path)
                    all_paths[index] = path
                    isolated_customers = [item for item in isolated_customers if item not in path]
                    break
#             printNetworkID(path)
            paths_tmp[index] = path
        fleet = instance.fleet
        # allocate the paths to the vehicles
        vehicle_id = 0
        for index, path in paths_tmp.items():
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
            vehicle.route.insert_node(0, self.depot)
            vehicle.route.append_node(self.depot)
#             path = []
#             for i in vehicle.route.route:
#                 path.append(i.id)
#             print ( 'The vehicle %s route:%s' %(vehicle.id, path))
        instance_solution = alg.Solution(instance, [self.depot.id]) 
        return instance_solution 
    
    
    
    def mutation1(self,solution):
        '''
        choose a route randomly and choose customer in the route randomly
        reinsert the same route randomly
        '''
        fleets = solution.solution.fleet
        route = [[],[],[]]
        # exlude the one customer route
        while len(route) == 3:
            route_index = random.randint(0,len(fleets)-1)
            route = fleets.fleet[route_index].route
        # remove the customer node randomly
        customer = random.sample(route[1:-1], 1)[0]
#         printNetworkID(route)
        route.pop_node_id(customer.id)
#         printNetworkID(route)
#         print (customer.id)
        # reinsert the customer node randomly
        insert_index = random.randint(1,len(route)-1) 
        route.insert_node(insert_index, customer)
#         printNetworkID(route)
#         printFleets(fleets)
        return solution 
    
    def mutation2(self, solution):
        '''
        choose a route randomly and choose customer in the route randomly
        reinsert the another feasible route randomly 
        
        '''
        fleets = solution.solution.fleet
        self.vehicle_capacity = fleets[0].capacity
        route = [[],[],[]]
        # exlude the one customer route
        while len(route) == 3:
            route_index = random.randint(0,len(fleets)-1)
            route = fleets.fleet[route_index].route
        # remove the customer node randomly
        customer = random.sample(route[1:-1], 1)[0]
#         printNetworkID(route)
#         print ('Load before pop:',fleets.fleet[route_index].load)
        route.pop_node_id(customer.id)
        fleets.fleet[route_index].update_load()
#         print ('Load after pop:',fleets.fleet[route_index].load)
#         printNetworkID(route)
#         print (customer.id)
        # reinsert the customer node randomly in another vehicle
        for indx, veh in enumerate(fleets):
            path = veh.route
            if indx == route_index:
                continue
            else:
                if (veh.load + customer.demand) > self.vehicle_capacity :
                    continue
                else:
                    insert_index = random.randint(1,len(path)-1) 
                    path.insert_node(insert_index, customer)
#                     print ('Befor add',veh.load)
#                     print ('Customer demand:', customer.demand)
                    print ('Add customer route:', end=' ')
                    printNetworkID(path)
                    veh.update_load()
#                     print ('After add:',veh.load)
                    break
#         printFleets(fleets)
#         input('123')
        return solution
    
    def mutation3(self, solution):
        '''
        randomly choose two routes and exchange the customer
        '''
        mark = 1
        times = 0
        fleets = solution.solution.fleet
        self.vehicle_capacity = fleets[0].capacity
        route_index = random.sample(range(0, len(fleets)), 2)
        route_index.sort()
        route_index1, route_index2 = route_index[0], route_index[1]
        # exchange customer node ensure the feasible solution
        while mark:
            times += 1
            if times == 10: # generate the feasible solution times
                return solution
            else:
                vehicle1 = fleets.fleet[route_index1]
                vehicle2 = fleets.fleet[route_index2]
                customer1 = random.sample(vehicle1.route[1:-1], 1)[0]
                customer2 = random.sample(vehicle2.route[1:-1], 1)[0]
                customer1_index = vehicle1.route.route.index(customer1) 
                customer2_index = vehicle2.route.route.index(customer2) 
                if (vehicle1.load - customer1.demand + customer2.demand) <= self.vehicle_capacity and \
                 (vehicle2.load - customer2.demand + customer1.demand) <= self.vehicle_capacity:
                    mark = 0
                else:
                    pass
        # vehicle1 exchange the customer
#         print ('Exchange customers : %s & %s'%(customer1.id, customer2.id))
#         print ('Demand: %s & %s'%(customer1.demand, customer2.demand ) )
#         print ('The first route:',end='')
#         printNetworkID(vehicle1.route)
#         print('Vehicle1 load before exchange:', vehicle1.load)
        vehicle1.route.pop_node_id(customer1.id)
        vehicle1.route.insert_node(customer1_index, customer2)
        vehicle1.update_load()
#         printNetworkID(vehicle1.route)
#         print('Vehicle1 load after exchange:', vehicle1.load)
#         # vehicle2 exchange the customer
#         print ('*'*30)
#         print ('The second route:',end='')
#         printNetworkID(vehicle2.route)
#         print('Vehicle2 load before exchange:', vehicle2.load)
        vehicle2.route.pop_node_id(customer2.id)
        vehicle2.route.insert_node(customer2_index, customer1)
        vehicle2.update_load()
#         printNetworkID(vehicle2.route)
#         print('Vehicle2 load after exchange:', vehicle2.load)
#         input('123')
        return solution
             
    def mutation4(self, solution):    
        '''
        Part routes insert the another route
        '''
        fleets = solution.solution.fleet
        self.vehicle_capacity = fleets[0].capacity
        path_length = [len(vehicle.route) for vehicle in fleets]
        max_path_vehicle = fleets.fleet[path_length.index(max(path_length))] 
        max_path = max_path_vehicle.route
        min_path_vehicle = fleets.fleet[path_length.index(min(path_length))] 
        min_path = min_path_vehicle.route 
        mark = 1
        times = 0
        print ('Exchange vehicle')
        printNetworkID(max_path)
        printNetworkID(min_path)
        # judge the solution is feasible ?
        while mark:
            times += 1
            if times == 10:
                return solution
            else:
                part_path_index = random.sample(range(1, len(max_path)-1), 2)
                part_path_index.sort()
                part_path = max_path[part_path_index[0]:part_path_index[1]+1]
                print ('Part path:', end = '')
                printNetworkID(part_path)
                part_part_demand = [node.demand for node in part_path]
                if (min_path_vehicle.load + sum(part_part_demand)) <= self.vehicle_capacity:
                    mark = 0
                else:
                    pass
        # delete the part path
        print ('The load before delete the part path:',max_path_vehicle.load)
        for node in part_path:
            max_path.pop_node_id(node.id)
        max_path_vehicle.update_load()
        print ('The load after delete the part path:',max_path_vehicle.load)
        print ('Remain route:', end = '')
        printNetworkID(max_path_vehicle.route)
        # insert the part path
        print ('The load before insert the part path:',min_path_vehicle.load)
        printNetworkID(min_path_vehicle.route)
        insert_index = random.randint(1, len(min_path)-1)
        for node in part_path[::-1]:
            min_path.insert_node(insert_index, node)
            min_path_vehicle.update_load()    
        print ('The load after insert the part path:',min_path_vehicle.load)
        print ('Insert route:', end = '')
        printNetworkID(min_path_vehicle.route)
#         input('123')
        return solution
   
    def mutation5(self, solution): 
        '''
        randomly choose a route and choose a customer to reinsert randomly
        '''
        fleets = solution.solution.fleet
        self.vehicle_capacity = fleets[0].capacity
        route = [[],[],[]]
        # exlude the one customer route
        while len(route) == 3:
            route_index = random.randint(0,len(fleets)-1)
            route = fleets.fleet[route_index].route
        # reinsert the customer node randomly
        customer = random.sample(route[1:-1], 1)[0]
#         print ('Distance 1:', solution.eval())
        route.pop_node_id(customer.id)
        fleets.fleet[route_index].update_load()
#         print ('Distance 2:', solution.eval())
        # reinsert the customer by distance
        self.insertNode(route, customer)
#         print ('Distance 3:', solution.eval())
#         input('123')
        return solution
        
    def mutation6(self, solution):
        '''
        choose a route randomly and choose customer in the route randomly
        reinsert the another feasible route by  best distance 
        
        '''
        fleets = solution.solution.fleet
        self.vehicle_capacity = fleets[0].capacity
        route = [[],[],[]]
        # exlude the one customer route
        while len(route) == 3:
            route_index = random.randint(0,len(fleets)-1)
            route = fleets.fleet[route_index].route
        # remove the customer node randomly
        customer = random.sample(route[1:-1], 1)[0]
#         printNetworkID(route)
#         print ('Load before pop:',fleets.fleet[route_index].load)
        route.pop_node_id(customer.id)
        fleets.fleet[route_index].update_load()
#         print ('Load after pop:',fleets.fleet[route_index].load)
#         printNetworkID(route)
#         print (customer.id)
        # reinsert the customer node randomly in another vehicle
        for indx, veh in enumerate(fleets):
            path = veh.route
            if indx == route_index:
                continue
            else:
                if (veh.load + customer.demand) > self.vehicle_capacity :
                    continue
                else:
                    self.insertNode(path, customer)
                    print ('Before add',veh.load)
                    print ('Customer demand:', customer.demand)
                    print ('Add customer route:', end=' ')
                    printNetworkID(path)
                    veh.update_load()
#                     print ('After add:',veh.load)
                    break
#         printFleets(fleets)
#         input('123')   
        return solution     
        
    def mutation7(self, solution):
        '''
        randomly choose two routes;
        All exchange customer;
        choose the best exchange (distance);
        '''
        fleets = solution.solution.fleet
        self.vehicle_capacity = fleets[0].capacity
        route_index = random.sample(range(0, len(fleets)), 2)
        route_index.sort()
        route_index1, route_index2 = route_index[0], route_index[1]
        vehicle1 = fleets.fleet[route_index1]
        vehicle2 = fleets.fleet[route_index2]
        feasible_couple = {}
        # exchange customer node ensure the feasible solution
        for index1, customer1 in enumerate(vehicle1.route):
            for index2, customer2 in enumerate(vehicle2.route):
                if index1 == 0 or index1 == (len(vehicle1.route) - 1) or index2 == 0 or index2 == (len(vehicle2.route) - 1):
                    continue
                if (vehicle1.load - customer1.demand + customer2.demand) <= self.vehicle_capacity and \
                 (vehicle2.load - customer2.demand + customer1.demand) <= self.vehicle_capacity:
                    # this is an exchange couple 
                    distance = self.distance_matrix[vehicle1.route[index1-1].id-1][customer1.id-1] +\
                               self.distance_matrix[customer1.id-1][vehicle1.route[index1+1].id-1] +\
                               self.distance_matrix[vehicle2.route[index2-1].id-1][customer2.id-1] +\
                               self.distance_matrix[customer2.id-1][vehicle2.route[index2+1].id-1] -\
                               (self.distance_matrix[vehicle1.route[index1-1].id-1][customer2.id-1] +\
                                self.distance_matrix[customer2.id-1][vehicle1.route[index1+1].id-1] +\
                                self.distance_matrix[vehicle2.route[index2-1].id-1][customer1.id-1] +\
                                self.distance_matrix[customer1.id-1][vehicle2.route[index2+1].id-1]) 
                    feasible_couple[distance] = (customer1, customer2, index1, index2)
        feasible_couple_sort = sorted(feasible_couple.items(), key=lambda kv: kv[0], reverse = True)
#         print ('feasible_couple_sort',feasible_couple_sort)
        if feasible_couple_sort == []: 
            return solution
        else:
            exchange_couple = feasible_couple_sort[0]
        if exchange_couple[0] <= 0:
            return solution
        node1, node2, node1_index, node2_index = exchange_couple[1][0],exchange_couple[1][1],exchange_couple[1][2],exchange_couple[1][3]
        # vehicle1 exchange the customer
#         print ('Exchange customers : %s & %s'%(node1.id, node2.id))
#         print ('Distance before exchange:', solution.eval())
#         print ('Demand: %s & %s'%(node1.demand, node2.demand ) )
#         print ('The first route:',end='')
#         printNetworkID(vehicle1.route)
#         print('Vehicle1 load before exchange:', vehicle1.load)
        vehicle1.route.pop_node_id(node1.id)
        vehicle1.route.insert_node(node1_index, node2)
        vehicle1.update_load()
#         printNetworkID(vehicle1.route)
#         print('Vehicle1 load after exchange:', vehicle1.load)
#         # vehicle2 exchange the customer
#         print ('*'*30)
#         print ('The second route:',end='')
#         printNetworkID(vehicle2.route)
#         print('Vehicle2 load before exchange:', vehicle2.load)
        vehicle2.route.pop_node_id(node2.id)
        vehicle2.route.insert_node(node2_index, node1)
        vehicle2.update_load()
#         printNetworkID(vehicle2.route)
#         print('Vehicle2 load after exchange:', vehicle2.load)
#         print ('Distance after exchange:', solution.eval())
#         input('123')
        return solution   

    
    def recombination(self, solution):
        '''
        (1) remove 1/2~1/10 customers from route, [isolated customers];
        (2) insert the isolated_customers into routes, FSCIM;
        '''
        fleet = solution.solution.fleet
        isolated_customers = []
        all_paths_isolated = []
        # remove customer node from the routes
        for vehicle in fleet:
            self.depot = vehicle.route.route[0]
            self.vehicle_capacity = vehicle.capacity
#             printNetworkID(vehicle.route.route)
            for node in vehicle.route.route[1:-1]:
                node.visited = False
            route_node_num = len(vehicle.route.route) - 2 # exclude the depot
            if route_node_num > 1:
                remove_num = math.floor(random.uniform(0.1, 0.5) * route_node_num) # choose the isolated node numbers
                if remove_num  == 0:
                    all_paths_isolated.append(vehicle.route.route[1:-1])
                    continue
#                 print ('Romove node number:%s'%remove_num)
                remove_index = random.sample(range(1, route_node_num+1), remove_num)
                remove_node_id = [vehicle.route.route[i].id for i in remove_index]
#                 print ('Delete the customer nodes index and id : %s & %s '%(remove_index, remove_node_id))
                # delete the customer node from the vehicle route
                isolated_customers_unit = [vehicle.route.route[node_index] for node_index in remove_index]
                isolated_customers.extend(isolated_customers_unit)
                for node_id in remove_node_id:
                    vehicle.route.pop_node_id(node_id)
#                 printNetworkID(vehicle.route.route)
            all_paths_isolated.append(vehicle.route.route[1:-1])
#         print (all_paths_isolated)
#         print ('***All isolated customers***')
#         printNetworkID(isolated_customers)
#         print('*'*30)
        random.shuffle(isolated_customers)
#         print ('-'*30)
#         # print the route removing the isolated customers
#         for path in all_paths_isolated:
#             printNetworkID(path)
#         print ('-'*30)
        new_instance_solution = self.generateInstance( all_paths_isolated, isolated_customers, copy.deepcopy(self.instance))
        return new_instance_solution

    def twoOpt(self, route):
        '''
        Local improvement 2-optimal
        '''
        copy_route = copy.deepcopy(route)
        path = []
        for node in route:
            path.append(node.id) 
        path = path[:-1] # remove the depot, a circle route 
        go = True
        while go:
            go, path = TwoOptimal.twoOptimal(path, len(path), self.coordination)
        tmp_path = path [path.index(1):] + path[:path.index(1)+1] # start from the depot and end to the depot
        for index,node_id in enumerate(tmp_path):
            for node in copy_route:
                if node.id == node_id:
                    route.route[index] = node
                    break    
                
    def localImproved(self, instance):
        '''
        For each route( Local Improvement ), use the 2-opt. 
        '''
        fleet = instance.solution.fleet
#         printFleets(fleet)
#         print ('Total:', instance.eval())
        for vehicle in fleet :
            self.twoOpt(vehicle.route)  
#         print ('-'*10)
#         printFleets(fleet)
#         print ('Total:', instance.eval())
        
    def fitness(self, pop, instance):
        fitness_value = (4 * self.parent_size + 1 ) - (pop.index(instance) + 1)
#         print ('Fitness:', fitness_value)
        instance.fitness = fitness_value 
        
    def selection(self,pop, bounds):
        """the Roulette wheel"""

        r = random.uniform(0, bounds)
#         print ('random r:',r)
#         input('prompt')
        for individual in pop:
#             lenn = 0
#             for veh in individual.solution.fleet: 
#                 lenn += len(veh.route)
#             print (lenn)
#             printFleets(individual.solution.fleet)
            r -= individual.fitness
            if r <= 0:
                return individual
        raise Exception("Error!", bounds)
    
    def runPop1(self, population):
        '''
        Get offspring (2*N) by recombination
        '''
        offspring_size = 2*self.parent_size
        offspring = []
        parent = []
        parent.extend(population)
        parent.extend(copy.deepcopy(population))
#         i=0
        for individual in parent:
#             i+=1
#             print ('*',i)
            child = self.recombination(individual)
#             printFleets(child.solution.fleet)
#             lenn = 0
#             for veh in child.solution.fleet:
#                 lenn += len(veh.route) 
#             print(lenn)
            offspring.append(child)
            if len(offspring) == (offspring_size - 1):
                break
            
        return offspring    
    
    def run(self):
        method = 1 # 1: diversification 2: intensification 
        times = 0
        offspring1 = []
        offspring2 = []
        best_value = float('inf')
        current_value = 0
        current_individual = None
        if method == 1:
            print ('Only the population1: diversification!')
            best_individual = copy.deepcopy(self.reproducing(self.population1))
            best_value = best_individual.value
            parent = self.population1
            while (times != self.iteration):
                print (times)
                # generate the offspring (N= 2*parent)
#                 print('Best value:%s'%best_value)
#                 printFleets(best_individual.solution.fleet)
                offspring1.append(best_individual)
                offspring1.extend(self.runPop1(parent))
                # get the best individual
                current_individual = copy.deepcopy(self.reproducing(offspring1))
                current_value = current_individual.value 
                if current_value < best_value:
                    best_individual = current_individual
                    best_value = current_value
#                     print ('-'*10)
#                     print('Best value:%s'%best_value)
#                     printFleets(best_individual.solution.fleet)
                else:
                    # the result value no change
                    times += 1
                # new parent
                new_generation = []
                new_generation.append(best_individual)
                bounds = 0 # the roulette wheel selection bound
                for instance in offspring1:
#                     printFleets(instance.solution.fleet)
                    self.fitness(offspring1, instance)
                    bounds += instance.fitness
#                     print ('Bounds:', bounds)
                while( len(new_generation) != self.parent_size ):
                    get_one = copy.deepcopy(self.selection(offspring1, bounds))
                    new_generation.append(get_one)
                parent = copy.deepcopy(new_generation)
#                 parent = new_generation
                offspring1 = [] 
            print ('The final result by population1.')
            print ('The best distance:%s'%best_individual.value)
            for veh in best_individual.solution.fleet:
                printNetworkID(veh.route) 
                     

        

if __name__ == '__main__':
    # input the data
    file_name = 'A-n32-k5.vrp' 
    instance, coordination = inputData(file_name)
    # initialization
    pop_size = 50
    population = popInitialization(instance, pop_size)
    # GA iteration and solution
    iteration_stop = 50
    gaSolver = GA(population, iteration_stop, instance, coordination)
    gaSolver.run()
    # show the best solution 
    pass
    