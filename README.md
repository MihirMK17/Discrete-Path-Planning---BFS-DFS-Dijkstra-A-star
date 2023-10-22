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


## DFS (Depth-First Search)

Depth-First Search is an algorithm to traverse a graph or tree. It visits nodes by diving deeper into the graph before backtracking.
![Depth_First_Search](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/ee51ed20-3537-478b-bb55-a15aa124dddf)


## Dijkstra's Algorithm

Dijkstra's algorithm is used to find the shortest path from a start node to all other nodes in a weighted graph.

### Avoid-East Cost

This cost function prioritizes paths that avoid moving in the eastward direction.
![Dijkstra_westcost_west_goal](https://github.com/MihirMK17/Discrete-Path-Planning-BFS-DFS-Dijkstra-A-star/assets/123691876/2dfa3b6d-9c55-4dbc-acd0-cc9fdc166a17)


### Avoid-West Cost

This cost function prioritizes paths that avoid moving in the westward direction.

## A* Algorithm

A* is a pathfinding and graph traversal algorithm, which is used to find the shortest path from a start node to a target node.

### Manhattan Distance Heuristic

Manhattan distance is the distance between two points measured along axes at right angles. It is computed as the sum of the absolute differences of their coordinates.

### Euclidean Distance Heuristic

Euclidean distance represents the shortest distance between two points in a plane. It is computed using the Pythagorean theorem.

---

Feel free to contribute or raise issues if you find any discrepancies in the implementations.


