# Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star

This repository contains implementations of various discrete path planning algorithms, including BFS, DFS, Dijkstra, and A*.

## Table of Contents

- [BFS (Breadth-First Search)](#bfs-breadth-first-search)
- [DFS (Depth-First Search)](#dfs-depth-first-search)
- [Dijkstra's Algorithm](#dijkstras-algorithm)
  - [Avoid-East Cost](#avoid-east-cost)
  - [Avoid-West Cost](#avoid-west-cost)
- [A* Algorithm](#a-algorithm)
  - [Manhattan Distance Heuristic](#manhattan-distance-heuristic)
  - [Euclidean Distance Heuristic](#euclidean-distance-heuristic)

## BFS (Breadth-First Search)
Breadth-First Search is a simple algorithm to traverse through nodes in a graph. It visits nodes level by level.
![Breadth_First_Search](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/4f3d6d52-b37c-43fc-a034-db05b57aa479)
The Breadth First Search algorithm implemented gives above result. The program uses a Queue data structure that stores children nodes of the current node and then pops the queue in priority of FIFO to explore these nodes. The BFS casts a wider net of searched nodes than DFS algorithm but the resulting path is still optimum. The initial node is chosen as (1,1) adn the goal node is chosen as (34, 16) in a `mediumMaze`. The results are,
- Nodes visited: 224
- Cost of Path: 68

## DFS (Depth-First Search)
Depth-First Search is an algorithm to traverse a graph or tree. It visits nodes by diving deeper into the graph before backtracking.
![Depth_First_Search](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/ee51ed20-3537-478b-bb55-a15aa124dddf)
The Depth First Search (DFS) algorithm implemented gives above result. The initial node is chosen as (1,1) and the goal node is chosen as (34, 16) in a `mediumMaze`. The program for DFS uses recursion with following results,
- Nodes visited: 140
- Cost of Path: 130

## Dijkstra's Algorithm
Dijkstra's algorithm is used to find the shortest path from a start node to all other nodes in a weighted graph.

### Avoid-East Cost
This cost function prioritizes paths that avoid moving in the eastward direction.
![Dijkstra_westCost](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/07412c47-e681-4fe7-8a56-6285d7566a86)
![Dijkstra_westcost_west_goal](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/82f99b53-702f-4ced-905c-bbc0ee896e86)

### Avoid-West Cost
This cost function prioritizes paths that avoid moving in the westward direction.
![Dijkstra_eastCost](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/7d37c016-3286-4085-b8cd-60917990ddfd)
![Dijkstra_eastCost_westgoal](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/6bcf20be-0c46-434e-a604-75be73d3da73)

## A* Algorithm
A* is a pathfinding and graph traversal algorithm, which is used to find the shortest path from a start node to a target node.

### Manhattan Distance Heuristic
Manhattan distance is the distance between two points measured along axes at right angles. It is computed as the sum of the absolute differences of their coordinates.
![aStar_Manhatten_small](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/b014eeb2-6137-418f-87f5-6e1abb1c0106)
![aStar_Manhatten](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/9514fdad-250c-4381-a45a-180a836643b9)
![aStar_Manhatten_bigMaze](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/e3b68c46-525a-4698-af89-0ed9ef65850f)

### Euclidean Distance Heuristic
Euclidean distance represents the shortest distance between two points in a plane. It is computed using the Pythagorean theorem.
![aStar_Eucildean_small](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/82fa2bce-cad6-431a-967c-265dcd7aebde)
![aStar_Euclidean](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/0eea47b9-40dd-4839-bce7-8ab2a7c7a6bb)
![aStar_Euclidean_bigMaze](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/7df0c219-078d-43af-af1e-63ae00d95fed)


---

Feel free to contribute or raise issues if you find any discrepancies in the implementations.


