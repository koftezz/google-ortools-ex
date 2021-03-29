import yaml
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
    data = {
        'time_matrix': [
            [0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7],
            [6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14],
            [9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9],
            [8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16],
            [7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14],
            [3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8],
            [6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5],
            [2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10],
            [3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6],
            [2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5],
            [6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4],
            [6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10],
            [4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8],
            [4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6],
            [5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2],
            [9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9],
            [7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0],
        ], 'time_windows': [
            (0, 5),  # depot
            (7, 12),  # 1
            (10, 15),  # 2
            (16, 18),  # 3
            (10, 13),  # 4
            (0, 5),  # 5
            (5, 10),  # 6
            (0, 4),  # 7
            (5, 10),  # 8
            (0, 3),  # 9
            (10, 16),  # 10
            (10, 15),  # 11
            (0, 5),  # 12
            (5, 10),  # 13
            (7, 8),  # 14
            (10, 15),  # 15
            (11, 15),  # 16
        ], 'demands': [0, 1, 1, 2, 3, 2, 3, 3, 1, 1, 2, 1, 2, 3, 3, 1, 1],
        'vehicle_capacities': [10, 10, 15, 15], 'num_vehicles': 4, 'depot': 0}
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_load = 0
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        # plan_output2 = 'Time for vehicle {}:\n'.format(vehicle_id)
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            time_var = time_dimension.CumulVar(index)
            plan_output += 'Node: {0} & Load({1})'.format(node_index,
                                                          route_load)
            plan_output += ' & Time({1},{2}) ->\n'.format(
                manager.IndexToNode(index), solution.Min(time_var),
                solution.Max(time_var))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
        plan_output += 'Node: {0} & Load({1})'.format(
            manager.IndexToNode(index),
            route_load)
        time_var = time_dimension.CumulVar(index)
        plan_output += ' & Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                   solution.Min(time_var),
                                                   solution.Max(time_var))
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_load += route_load
        total_time += solution.Min(time_var)
    print('Total load of all routes: {}'.format(total_load))
    print('Total time of all routes: {}min'.format(total_time))


def _demand_callback(manager, data):
    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    return demand_callback


def _time_callback(manager, data):
    def time_callback(from_index, to_index):
        """
        Return the total time between the two nodes.
        """
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    return time_callback


def _time_window_constraints(routing, manager, data, time_callback_index):
    time = 'Time'
    routing.AddDimension(
        time_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    depot_idx = data['depot']
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][depot_idx][0],
            data['time_windows'][depot_idx][1])


def _capacity_constraints(routing, data, capacity_callback_index):
    routing.AddDimensionWithVehicleCapacity(
        capacity_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        name='Capacity')


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    data['num_vehicles'] = len(data['vehicle_capacities'])

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'],
                                           data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Add capacity for each route
    capacity_callback_index = routing.RegisterUnaryTransitCallback(
        _demand_callback(manager, data))
    _capacity_constraints(routing, data, capacity_callback_index)

    # Add time window
    time_callback_index = routing.RegisterTransitCallback(
        _time_callback(manager, data))
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)
    _time_window_constraints(routing, manager, data, time_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()

    # first solution strategy -
    # The first solution strategy is the method the
    # solver uses to find an initial solution.
    # https://developers.google.com/optimization/routing/routing_options#first_sol_options
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # local search options
    # options for local search strategies (also called metaheuristics)
    # https://developers.google.com/optimization/routing/routing_options#local_search_options
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)

    # search limits
    # terminate the solver after it reaches a specified limit,
    # such as the maximum length of time, or number of solutions found.
    # https://developers.google.com/optimization/routing/routing_options#search-limits
    search_parameters.time_limit.FromSeconds(100)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution exists!")


if __name__ == '__main__':
    main()
