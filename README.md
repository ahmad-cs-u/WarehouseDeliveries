# WarehouseDeliveries
A warehouse delivery scheduler application written in C++ that uses various data structures to schedule deliveries based on priority.

# Overview
This Warehouse Management System is a C++ application designed to manage warehouse networks, delivery scheduling, and route optimization. The system provides functionalities for loading warehouse data, scheduling deliveries based on priority, processing deliveries using Dijkstra's algorithm for optimal routing, and performing graph traversals (BFS/DFS) and minimum spanning tree computations.


# Features

## Core Functionalities

### Warehouse Network Management

  Load warehouse data from files

  Manage connections between warehouses with distance, time, and cost metrics

### Delivery Scheduling

  Priority-based delivery scheduling using AVL trees and max-heaps

  Three route optimization options: distance, time, or cost

## Graph Algorithms

  Dijkstra's algorithm for optimal path finding

  Prim's algorithm for Minimum Spanning Tree (MST) computation

  Breadth-First Search (BFS) and Depth-First Search (DFS) traversals (were a requirement)

## Data Structures Used

  AVL Trees for storing deliveries by priority

  Max-Heap for processing high-priority deliveries first

  Custom Stack and Queue implementations for DFS and BFS

  Adjacency Matrix for warehouse connections

  Path nodes for route representation

## How to Use

Load Data Files

  warehouses.txt: Contains warehouse IDs and cities

  warehouse_connections.txt: Contains connection data between warehouses

  deliveries.txt: Contains delivery information with priorities

## Main Menu Options

  Load Data: Loads all data files

  Schedule Delivery: Schedules a specific delivery by ID

  Process Deliveries: Processes all scheduled deliveries in priority order

  Compute MST: Calculates minimum spanning tree from a starting warehouse

  BFS/DFS Traversal: Performs graph traversal from a starting warehouse

## File Formats
warehouses.txt

    warehouse_id,city
    WH001,New York
    ...

warehouse_connections.txt

    from_warehouse,to_warehouse,distance,time,cost
    WH001,WH002,2445,40,500
    ...

deliveries.txt
    
    delivery_id,origin_id,destination_id,priority,route_priority
    DL001,WH001,WH005,5,cost
    ...


## Notes

  The system uses numeric priorities (1-5) with 1 being lowest and 5 highest

  Route priorities can be "distance", "time", or "cost"

  All data must be loaded before performing any operations

  The program includes error handling for invalid inputs and file operations

## License

This project is open-source and available for educational purposes.

  
