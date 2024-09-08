# Capacitated Vehicle Routing Problem (CVRP) Solver

This project implements a Capacitated Vehicle Routing Problem (CVRP) solver using [Google OR-Tools](https://developers.google.com/optimization). The goal is to optimize delivery routes for a fleet of vehicles while respecting their weight and volume capacities.

## Table of Contents
- [Problem Description](#problem-description)
- [Features](#features)
- [Data](#data)
- [Setup](#setup)
- [Usage](#usage)
- [Results](#results)
- [License](#license)

## Problem Description

The Capacitated Vehicle Routing Problem (CVRP) involves determining the optimal set of routes for a fleet of vehicles to service a set of customers. Each vehicle has:
- A maximum weight and volume capacity.
- A cost per kilometer for traveling.
- A fixed cost for being assigned to a route.

The goal is to minimize the total cost, which includes the distance traveled and the fixed vehicle costs, while ensuring that each vehicle respects its capacity limits for both weight and volume.

## Features

- Load data from JSON files.
- Constraints for vehicle weight and volume capacities.
- Minimize total delivery costs (fixed vehicle costs + per-kilometer costs).
- Supports custom distance matrices, travel times, and cost matrices.
- Uses OR-Tools to find the optimal or near-optimal solution.
  
## Data

The input data is provided in the `assignment_cvrp.json` file, which contains:
- `distance_matrix`: A matrix of distances between locations.
- `volume_matrix`: The volume of goods for each location.
- `weight_matrix`: The weight of goods for each location.
- `location_matrix`: The list of locations.
- `max_volume`: The maximum volume capacity for each vehicle.
- `max_weight`: The maximum weight capacity for each vehicle.
- `perKmCostPerVehicle`: The cost per kilometer for each vehicle.
- `fixedCostPerVehicle`: The fixed cost associated with each vehicle.
- `loc_ids`: The IDs of each location.
