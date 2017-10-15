# From here https://developers.google.com/optimization/routing/tsp/vehicle_routing - Google OR Tools    

import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def distance(lat1, long1, lat2, long2):
    # Note: The formula used in this function is not exact, as it assumes
    # the Earth is a perfect sphere.

    # Mean radius of Earth in miles
    radius_earth = 3959

    # Convert latitude and longitude to
    # spherical coordinates in radians.
    degrees_to_radians = math.pi/180.0
    phi1 = lat1 * degrees_to_radians
    phi2 = lat2 * degrees_to_radians
    lambda1 = long1 * degrees_to_radians
    lambda2 = long2 * degrees_to_radians
    dphi = phi2 - phi1
    dlambda = lambda2 - lambda1

    a = haversine(dphi) + math.cos(phi1) * math.cos(phi2) * haversine(dlambda)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius_earth * c
    return d * 1000

def haversine(angle):
    h = math.sin(angle / 2) ** 2
    return h

#def distance(x1, y1, x2, y2):
#    # Manhattan distance
#    dist = (abs(x1 - x2) + abs(y1 - y2))
#    return dist
    
class CreateDistanceCallback(object):
  """Create callback to calculate distances between points."""

  def __init__(self, locations):
    """Initialize distance array."""
    size = len(locations)
    self.matrix = {}

    for from_node in range(size):
      self.matrix[from_node] = {}
      for to_node in range(size):
        x1 = locations[from_node][1]
        y1 = locations[from_node][2]
        x2 = locations[to_node][1]
        y2 = locations[to_node][2]
        self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
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
    travel_time = self.dist_callback(from_node, to_node) / self.speed
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
  
  num_locations = len(locations)
  depot = 0    # The depot is the start and end point of each route.
  num_vehicles = 5

  # Create routing model.
  if num_locations > 0:
      
    # The number of nodes of the VRP is num_locations.
    # Nodes are indexed from 0 to num_locations - 1. By default the start of
    # a route is node 0.
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
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
    
    # Put a callback to the demands.
    demands_at_locations = CreateDemandCallback(locations)
    demands_callback = demands_at_locations.Demand

    # Adding capacity dimension constraints.
    slack_max = 0
    vehicle_capacity = 5
    fix_start_cumul_to_zero = True
    capacity  = "Capacity"
    
    routing.AddDimension(demands_callback, slack_max, vehicle_capacity,
                         fix_start_cumul_to_zero, capacity)

    # Add time dimension.
    time_per_demand_unit = 3
    horizon = 24 * 3600
    time = "Time"
    speed = 1

    service_times = CreateServiceTimeCallback(locations, time_per_demand_unit)
    service_time_callback = service_times.ServiceTime

    travel_times = CreateTravelTimeCallback(dist_callback, speed)
    travel_time_callback = travel_times.TravelTime

    total_times = CreateTotalTimeCallback(service_time_callback, travel_time_callback)
    total_time_callback = total_times.TotalTime

    routing.AddDimension(total_time_callback,  # total time function callback
                         horizon,
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

        for vehicle_nbr in range(num_vehicles):
            index = routing.Start(vehicle_nbr)
            plan_output = 'Route {0}:'.format(vehicle_nbr)
            
            route_dist = 0
            
            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                load_var = capacity_dimension.CumulVar(index)
                time_var = time_dimension.CumulVar(index)
                plan_output += \
                    " {node_name} Load({load}) Time({tmin}, {tmax}) -> ".format(
                        node_name=locations[node_index][0],
                        load=assignment.Value(load_var),
                        tmin=str(assignment.Min(time_var)),
                        tmax=str(assignment.Max(time_var)))
                    
                # Add the distance to the next node.
                # TODO
                
                index = assignment.Value(routing.NextVar(index))

                node_index = routing.IndexToNode(index)
                load_var = capacity_dimension.CumulVar(index)
                time_var = time_dimension.CumulVar(index)
                plan_output += \
                  " {node_name} Load({load}) Time({tmin}, {tmax})".format(
                      node_name=locations[node_index][0],
                      load=assignment.Value(load_var),
                      tmin=str(assignment.Min(time_var)),
                      tmax=str(assignment.Max(time_var)))
            
            #TODO
            #node_index_next = assignment.Value(routing.NextVar(index))
            #route_dist += dist_callback(node_index, node_index_next)            
            #print("Distance of route " + str(vehicle_nbr) + ": " + str(route_dist))
            
#        print("Demand met by vehicle " + str(vehicle_nbr) + ": " + str(route_demand) + "\n")
            print(plan_output)
            print("\n")
        
#        index_next = assignment.Value(routing.NextVar(index))
#        route = ''
#        route_dist = 0
#        route_demand = 0

#        while not routing.IsEnd(index_next):
#          node_index = routing.IndexToNode(index)
#          
#          node_index_next = routing.IndexToNode(index_next)
#          #route += str(node_index) + " -> "
#          route += locations[node_index][0] + " -> " + str(dist_callback(node_index, node_index_next)) + " -> "
#          # Add the distance to the next node.
#          route_dist += dist_callback(node_index, node_index_next)
#          # Add demand.
#          route_demand += locations[node_index_next][3]
#          index = index_next
#          index_next = assignment.Value(routing.NextVar(index))
#
#        node_index = routing.IndexToNode(index)
#        node_index_next = routing.IndexToNode(index_next)
#        route += locations[node_index][0] + " -> " + str(dist_callback(node_index, node_index_next)) + " -> "+ locations[node_index_next][0]
#        route_dist += dist_callback(node_index, node_index_next)
#        print("Route for vehicle " + str(vehicle_nbr) + ":\n\n" + route + "\n")
#        print("Distance of route " + str(vehicle_nbr) + ": " + str(route_dist))
#        print("Demand met by vehicle " + str(vehicle_nbr) + ": " + str(route_demand) + "\n")
        
    else:
      print('No solution found.')
      
  else:
    print('Specify an instance greater than 0.')

def create_data_array():

    import pyodbc    
    #creating connection Object which will contain SQL Server Connection    
    server = 'rnd-101'
    database = 'Minos2'
    username = 'sa'
    password = '77java&&'
    driver= '{ODBC Driver 13 for SQL Server}'
    connection = pyodbc.connect('DRIVER='+driver+';PORT=1433;SERVER='+server+';PORT=1443;DATABASE='+database+';UID='+username+';PWD='+ password)

    cursor = connection.cursor()

    cursor.execute("SELECT CUS.adrid, CUS.CentroidX, CUS.CentroidY, CUS.leename, CUS.cuscode, CUS.adrstreet, CUS.adrzipcode, CUS.cusid, CUS.adrnumber, CUS.adrcity, CUS.adrdistrict , CUS.tziros, cus.slmid2 FROM vwCustomers AS CUS WHERE (CUS.slmid IN (4) OR CUS.slmid2 IN (4) OR CUS.slmid3 IN (4))")

    locations=[]
    
    # tw_duration is the width of the time windows.
    tw_duration = 2150
  
    for row in cursor.fetchall():
        #print(row.leename)        
        locations.append([row.leename, row.CentroidX, row.CentroidY, 1, 0, 24 * 3600]) #name, latitude, longitude, demand, start_time, end_time

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

