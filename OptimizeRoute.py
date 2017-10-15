# From here https://developers.google.com/optimization/routing/tsp/vehicle_routing

import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import googlemaps

from haversine import haversine

def distance(lat1, long1, lat2, long2):
    # Distance in km, get direct line distance and multiply by 1.5 as an estimation for road network
    return haversine((lat1, long1),(lat2, long2),miles = False) * 1.5 
        
class CreateDistanceCallback(object):
  """Create callback to calculate distances between points."""

  def __init__(self, locations):
    """Initialize distance array."""
    size = len(locations)
    self.matrix = {}

    self.calls = 0
    
    for from_node in range(size):
      self.matrix[from_node] = {}
      for to_node in range(size):
        x1 = locations[from_node][1]
        y1 = locations[from_node][2]
        x2 = locations[to_node][1]
        y2 = locations[to_node][2]
        self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
      self.calls += 1
      return self.matrix[from_node][to_node]

# Demand callback
class CreateDemandCallback(object):
  """Create callback to get demands at each location."""

  def __init__(self, locations):
    self.matrix = locations 

  def Demand(self, from_node, to_node):
    return self.matrix[from_node][3] #4th column of locations contains the demand

# Service time (proportional to demand) callback.
class CreateServiceTimeCallback(object):
  """Create callback to get time windows at each location."""

  def __init__(self, locations, time_per_demand_unit):
    self.matrix = locations
    self.time_per_demand_unit = time_per_demand_unit

  def ServiceTime(self, from_node, to_node):
    return self.matrix[from_node][3] * self.time_per_demand_unit

# Create the travel time callback (equals distance divided by speed).
class CreateTravelTimeCallback(object):
  """Create callback to get travel times between locations."""

  def __init__(self, dist_callback, speed):
    self.dist_callback = dist_callback
    self.speed = speed

  def TravelTime(self, from_node, to_node):
    travel_time = self.dist_callback(from_node, to_node) / self.speed * 60 #speed is in km/h
    return travel_time

# Create total_time callback (equals service time plus travel time).
class CreateTotalTimeCallback(object):
  """Create callback to get total times between locations."""

  def __init__(self, service_time_callback, travel_time_callback):
    self.service_time_callback = service_time_callback
    self.travel_time_callback = travel_time_callback

  def TotalTime(self, from_node, to_node):
    service_time = self.service_time_callback(from_node, to_node)
    travel_time = self.travel_time_callback(from_node, to_node)
    return service_time + travel_time

def main():
    # Create the data.
    locations = create_data_array()

#    # Distance matrix
#    origins = []
#    destinations = []
#
#    for location in locations:
#        origins.append(str(location[1])+' '+str(location[2]))
#        destinations.append(str(location[1])+' '+str(location[2]))
#        
#    gmaps = googlemaps.Client(key='AIzaSyBDStBcrgTyPKJGuxTRw4uqR2kkCBH-RIQ')
#    gmaps.timeout = 10000
#    matrix = gmaps.distance_matrix(origins, destinations, mode="driving", units='metric', avoid='tolls')
    
    num_locations = len(locations)
    depot = 0    # The depot is the start and end point of each route.
    num_vehicles = 5

    # Create routing model.
    if num_locations > 0:
      
        # The number of nodes of the VRP is num_locations.
        # Nodes are indexed from 0 to num_locations - 1. By default the start of
        # a route is node 0.
        # There is also the option to begin and end at the specified locations, check https://developers.google.com/optimization/routing/tsp/vehicle_routing    
        routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

        # Setting first solution heuristic: the
        # method for finding a first solution to the problem.
        search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
        # The 'PATH_CHEAPEST_ARC' method does the following:
        # Starting from a route "start" node, connect it to the node which produces the
        # cheapest route segment, then extend the route by iterating on the last
        # node added to the route.
        
        #    search_parameters.first_solution_strategy = (
        #        routing_enums_pb2.FirstSolutionStrategy.GLOBAL_CHEAPEST_ARC)
        # GlobalCheapestArc = iteratively connect two nodes which produce the cheapest route segment
        
        # Put a callback to the distance function here. The callback takes two
        # arguments (the from and to node indices) and returns the distance between
        # these nodes.
        dist_between_locations = CreateDistanceCallback(locations)
        dist_callback = dist_between_locations.Distance
        
        # Minimize distance
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)        
    
        # Put a callback to the demands.
        demands_at_locations = CreateDemandCallback(locations)
        demands_callback = demands_at_locations.Demand
    
        # Adding capacity dimension constraints.
        slack_max = 0  # How much the capacity can increase beyond demand?
        vehicle_capacity = 20
        fix_start_cumul_to_zero = True
        capacity  = "Capacity"
        
        routing.AddDimension(demands_callback, 
                             slack_max, 
                             vehicle_capacity,
                             fix_start_cumul_to_zero, 
                             capacity)
    
        # Add time dimension.
        # Time is measured in minutes
        time_per_demand_unit = 30
        horizon = 8 * 60 # 8 hours * 60 minutes
        time = "Time"
        speed = 30 #km/h
    
        service_times = CreateServiceTimeCallback(locations, time_per_demand_unit)
        service_time_callback = service_times.ServiceTime
    
        travel_times = CreateTravelTimeCallback(dist_callback, speed)
        travel_time_callback = travel_times.TravelTime
    
        total_times = CreateTotalTimeCallback(service_time_callback, travel_time_callback)
        total_time_callback = total_times.TotalTime
        
        # Minimize time. You can also have a different cost evaluator for each vehicle (https://groups.google.com/forum/#!msg/or-tools-discuss/LCuQSC-JcXU/7_A-7Az-738J)
        #routing.SetArcCostEvaluatorOfAllVehicles(travel_time_callback)
        #routing.SetArcCostEvaluatorOfVehicle()
        
        # total_time_callback — Returns the service time plus travel time to the next location.
        # horizon — An upper bound for the accumulated time over each vehicle's route. This sets a global time window of [0, horizon] for all locations. To set the individual time windows at each location, you need to set ranges on the cumulative variable for time, as shown in Add time window constraints.
        # fix_start_cumul_to_zero — Since the value is True, the cumulative variable for time is set to 0 at the start of each vehicle's route.
        # time — The name of the dimension, which you can use to access data or variables stored in it.
        routing.AddDimension(total_time_callback,  
                             horizon, # max waiting time at a location
                             horizon,
                             fix_start_cumul_to_zero,
                             time)
        
        # Add time window constraints.
        time_dimension = routing.GetDimensionOrDie(time)
        for location in range(1, num_locations):
          start = locations[location][4]
          end = locations[location][5]
          time_dimension.CumulVar(location).SetRange(start, end)          
        
        # Solve, displays a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            size = len(locations)
            # Display solution.
            # Solution cost.
            print("Total distance of all routes: " + str(assignment.ObjectiveValue()) + "\n")
    
            # Inspect solution.
            capacity_dimension = routing.GetDimensionOrDie(capacity);
            time_dimension = routing.GetDimensionOrDie(time);
    
            InitVehicleDetailsIndb(1)
            
            for vehicle_nbr in range(num_vehicles):
                
                index = routing.Start(vehicle_nbr)
                node_index = routing.IndexToNode(index)          
                
                plan_output = 'Route {0}:'.format(vehicle_nbr)
                
                route_distance = 0
                route_demand = 0
                position_in_route = 0
                distance_for_this_node = 0
                
                # node_index is the index in locations list
                # index = node_index except for the depot which is duplicated for each vehicle
                while not routing.IsEnd(index):                
                    
                    position_in_route+=1                
                    route_demand += locations[node_index][3]
                    
                    #print(index)
                    #print(node_index)
                    #print(locations[node_index][6])
                    load_var = capacity_dimension.CumulVar(index) #cumulcative value of dimension capacity when arriving at this node
                    time_var = time_dimension.CumulVar(index) #cumulcative value of time capacity when arriving at this node
                    
    #                plan_output += \
    #                    "\n {node_name} Load({load}) Time({tmin}, {tmax}) -> ".format(
    #                        node_name=locations[node_index][0],
    #                        load=assignment.Value(load_var),
    #                        tmin=str(assignment.Min(time_var)),
    #                        tmax=str(assignment.Max(time_var)))
                    
                    plan_output += \
                        "\n {node_name} Arr. load({load}), Arr. time({time}), Travel distance({distance}) -> ".format(
                            node_name=locations[node_index][0],
                            load=assignment.Value(load_var),
                            time=str(assignment.Value(time_var)),
                            distance=round(distance_for_this_node,1))
                        
                    if position_in_route>1: #position_in_route=1 is always the start which is common for all routes
                        UpdateRouteDetailsIndb(vehicle_nbr, position_in_route-1, assignment.Min(time_var), assignment.Max(time_var), '', locations[node_index][6])
                                        
                    #Now move to next node                
                    index = assignment.Value(routing.NextVar(index))
                    next_node_index = routing.IndexToNode(index) # store it in a var to get the distance
                    distance_for_this_node = dist_callback(node_index, next_node_index)
                    route_distance += round(distance_for_this_node,1)
                    node_index =next_node_index
    
                # End of route
                load_var = capacity_dimension.CumulVar(index) # cumulcative value of dimension capacity when arriving at this node
                time_var = time_dimension.CumulVar(index) # cumulcative value of time capacity when arriving at this node
                    
                plan_output += \
                        "\n {node_name} Arr. load({load}), Arr. time({time}), Travel distance({distance})".format(
                            node_name=locations[node_index][0],
                            load=assignment.Value(load_var),
                            time=str(assignment.Value(time_var)),
                            distance=round(distance_for_this_node,1))
                        
                route_duration = assignment.Value(time_var)                                                
                
                print(plan_output)            
                print("Demand met by vehicle " + str(vehicle_nbr) + ": " + str(route_demand))
                print("Distance of route " + str(vehicle_nbr) + ": " + str(route_distance) + " km")
                print("Duration of route " + str(vehicle_nbr) + ": " + str(route_duration) + " minutes")
                print("\n")
            
                InsertVehicleDetailsIndb(1, vehicle_nbr, route_distance, route_demand, route_duration, plan_output)
                
                print("Calls: " + str(dist_between_locations.calls))
        else:
            print('No solution found.')
          
    else:
        print('Specify an instance greater than 0.')

def GetConnection():
    import pyodbc    
    #creating connection Object which will contain SQL Server Connection    
    server = 'rnd-101'
    database = 'Minos2'
    username = 'sa'
    password = '77java&&'
    driver= '{ODBC Driver 13 for SQL Server}'
    connection = pyodbc.connect('DRIVER='+driver+';PORT=1433;SERVER='+server+';PORT=1443;DATABASE='+database+';UID='+username+';PWD='+ password)
    return connection

def InitVehicleDetailsIndb(ORID):
    connection = GetConnection()
    cursor = connection.cursor()    
    
    SQLCommand = "DELETE FROM tblOptimizedRouteVehicleInfo WHERE ORID=?"
    Values = [ORID]
    cursor.execute(SQLCommand,Values)
    
    connection.commit()

    #closing connection    
    connection.close()   

def InsertVehicleDetailsIndb(ORID, vehicleid, distance, demandmet, duration, routeinfo):    
    connection = GetConnection()
    cursor = connection.cursor()    
    
    SQLCommand = "INSERT INTO tblOptimizedRouteVehicleInfo (ORID, VehicleID, Distance, DemandMet, Duration, RouteInfo) VALUES (?, ?, ?, ?, ?, ?)"
    Values = [ORID, vehicleid, distance, demandmet, duration, routeinfo]
    cursor.execute(SQLCommand,Values)
    
    connection.commit()

    #closing connection    
    connection.close()   

def UpdateRouteDetailsIndb(vehicleid, position, arrival, departure, routeinfo, ORDetailID):    
    connection = GetConnection()
    cursor = connection.cursor()
    
    SQLCommand = "UPDATE tblOptimizedRouteDetails SET vehicleid=?, position=?, arrival=? ,departure=?, routeinfo=? WHERE ORDetailID=?"
    Values = [vehicleid, position, arrival, departure, routeinfo, ORDetailID]
    cursor.execute(SQLCommand,Values)
    
    connection.commit()

    #closing connection    
    connection.close()    

def create_data_array():
    connection = GetConnection()
    cursor = connection.cursor()

    #cursor.execute("SELECT CUS.adrid, CUS.CentroidX, CUS.CentroidY, CUS.leename, CUS.cuscode, CUS.adrstreet, CUS.adrzipcode, CUS.cusid, CUS.adrnumber, CUS.adrcity, CUS.adrdistrict , CUS.tziros, cus.slmid2 FROM vwCustomers AS CUS WHERE (CUS.slmid IN (4) OR CUS.slmid2 IN (4) OR CUS.slmid3 IN (4))")
    cursor.execute("SELECT ORDetailID, cusid, name, adrid, long, lat, demand, starttime, endtime FROM tblOptimizedRouteDetails WHERE ORID=1")
    locations=[]
    
    # tw_duration is the width of the time windows.
    #tw_duration = 2150
  
    for row in cursor.fetchall():
        #print(row.leename)        
        locations.append([row.name, row.lat, row.long, row.demand, row.starttime, row.endtime, row.ORDetailID]) #name, latitude, longitude, demand, start_time, end_time, ORDetailID

    #closing connection    
    connection.close()    

#  locations = [[82, 76], [96, 44], [50, 5], [49, 8], [13, 7], [29, 89], [58, 30], [84, 39],
#               [14, 24], [12, 39], [3, 82], [5, 10], [98, 52], [84, 25], [61, 59], [1, 65],
#               [88, 51], [91, 2], [19, 32], [93, 3], [50, 93], [98, 14], [5, 42], [42, 9],
#               [61, 62], [9, 97], [80, 55], [57, 69], [23, 15], [20, 70], [85, 60], [98, 5]]

#  demands = [0, 19, 21, 6, 19, 7, 12, 16, 6, 16, 8, 14, 21, 16, 3, 22, 18,
#             19, 1, 24, 8, 12, 4, 8, 24, 24, 2, 20, 15, 2, 14, 9]
  
    return locations
if __name__ == '__main__':
  main()

