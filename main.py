import json
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Open the JSON file
with open('assignment_cvrp.json', 'r') as file:
    # Load the JSON data from the file
    data_json = json.load(file)

# Create data for model
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = [[math.ceil(value) for value in row] for row in data_json['distance']]
    data["time_matrix"] = data_json['durations']
    data["volume"] = aggregate('volume_matrix')
    data['weight'] = aggregate('weight_matrix', 1000)
    data["volume_capacities"] = data_json['max_volume']
    data["weight_capacities"] = data_json['max_weight']
    data['per_km_cost'] = [math.ceil(value) for value in data_json['perKmCostPerVehicle']]
    data['fixed_cost'] = data_json['fixedCostPerVehicle']
    data["num_vehicles"] = len(data_json['max_volume'])
    data["depot"] = 0
    data['order_loc_ids'] = data_json['loc_ids']
    return data

def aggregate(name, multiplier=1):
    temp_list = []
    for i in range(len(data_json['loc_ids'])):
        temp = 0
        for j in range(len(data_json['location_matrix'])):
            if data_json['loc_ids'][i] == data_json['location_matrix'][j]:
                temp += math.ceil(data_json[name][j] * multiplier)
        temp_list.append(math.ceil(temp))

    return temp_list


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_volume = 0
    total_weight = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_volume = 0
        route_weight = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_volume += data["volume"][node_index]
            route_weight += data["weight"][node_index]
            plan_output += f" loc{node_index} weight({route_weight}) volume({route_volume}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += f" loc{manager.IndexToNode(index)} weight({route_weight}) volume({route_volume})\n"
        plan_output += f"Cost of the route: {route_distance}\n"
        plan_output += f"Volume of the route: {route_volume}\n"
        plan_output += f"Weight of the route: {route_weight}\n"
        print(plan_output)
        total_distance += route_distance
        total_volume += route_volume
        total_weight += route_weight
    print(f"Total cost of all routes: {total_distance}")
    print(f"Total volume of all routes: {total_volume}")
    print(f"Total weight of all routes: {total_weight}")


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a distance callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set distance callback for all vehicles.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Create and register a vehicle cost callback.
    def vehicle_cost_callback(from_index, to_index, vehicle_id):
        """Returns the cost based on distance and vehicle type."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        distance = data["distance_matrix"][from_node][to_node]
        return int(data['per_km_cost'][vehicle_id] * distance)

    # Register vehicle cost callbacks and set vehicle costs.
    for vehicle_id in range(data["num_vehicles"]):
        vehicle_cost_callback_index = routing.RegisterTransitCallback(
            lambda from_index, to_index, vehicle_id=vehicle_id:
            vehicle_cost_callback(from_index, to_index, vehicle_id)
        )
        routing.SetArcCostEvaluatorOfVehicle(vehicle_cost_callback_index, vehicle_id)
        vehicle_fixed_cost = math.ceil(data["fixed_cost"][vehicle_id])
        routing.SetFixedCostOfVehicle(vehicle_fixed_cost, vehicle_id)

    # Add Weight Capacity constraint.
    def weight_callback(from_index):
        """Returns the weight demand at the node."""
        from_node = manager.IndexToNode(from_index)
        return data["weight"][from_node]

    weight_callback_index = routing.RegisterUnaryTransitCallback(weight_callback)
    routing.AddDimensionWithVehicleCapacity(
        weight_callback_index,
        0,  # null capacity slack
        data["weight_capacities"],  # vehicle weight capacities
        True,  # start cumul to zero
        "Wt_Capacity",
    )

    # Add Volume Capacity constraint.
    def volume_callback(from_index):
        """Returns the volume demand at the node."""
        from_node = manager.IndexToNode(from_index)
        return data["volume"][from_node]

    volume_callback_index = routing.RegisterUnaryTransitCallback(volume_callback)
    routing.AddDimensionWithVehicleCapacity(
        volume_callback_index,
        0,  # null capacity slack
        data["volume_capacities"],  # vehicle volume capacities
        True,  # start cumul to zero
        "Volume_Capacity",
    )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(10)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
        save_solution_as_json(data, manager, routing, solution)

# Function to save the solution as a JSON file
def save_solution_as_json(data, manager, routing, solution, output_file='solution.json'):
    """Saves the solution to a JSON file."""
    solution_dict = {
        "objective_value": solution.ObjectiveValue(),
        "routes": [],
        "total_distance": 0,
        "total_volume": 0,
        "total_weight": 0
    }

    total_distance = 0
    total_volume = 0
    total_weight = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route = {
            "vehicle_id": vehicle_id,
            "route": [],
            "route_distance": 0,
            "route_volume": 0,
            "route_weight": 0
        }
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route["route"].append(f"loc{node_index}")
            route["route_volume"] += data["volume"][node_index]
            route["route_weight"] += data["weight"][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route["route_distance"] += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        route["route"].append(f"loc{manager.IndexToNode(index)}")
        solution_dict["routes"].append(route)

        total_distance += route["route_distance"]
        total_volume += route["route_volume"]
        total_weight += route["route_weight"]

    solution_dict["total_distance"] = total_distance
    solution_dict["total_volume"] = total_volume
    solution_dict["total_weight"] = total_weight

    # Save the solution to a JSON file
    with open(output_file, 'w') as f:
        json.dump(solution_dict, f, indent=4)
    print(f"Solution saved to {output_file}")

if __name__ == "__main__":
    main()
