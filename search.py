#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: sonia martinez
"""

# Please do not distribute or publish solutions to this
# exercise. You are free to use these problems for educational purposes.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

from mazemods import maze
from mazemods import makeMaze
from mazemods import collisionCheck
from mazemods import makePath
from mazemods import getPathFromActions
from mazemods import getCostOfActions
from mazemods import stayWestCost
from mazemods import stayEastCost
from collections import deque
from collections import OrderedDict


nodesVisitedDFS = set()
solutionDFS = {}
xInitial = (1, 1)


def isInBounds(xI, n, m):
    (x, y) = xI
    return 0 <= x <= n - 1 and 0 <= y <= m - 1


def depthFirstSearch(xI, xG, n, m, O):
    """
      Search the deepest nodes in the search tree first.

      Your search algorithm needs to return a list of actions
      leading to a valid path to the goal.
      Make sure to implement a graph search algorithm.
      Your algorithm also needs to return the cost of the path.
      Use the getCostOfActions function to do this.
      Finally, the algorithm should return the number of visited
      nodes in your search.

      """
    "*** YOUR CODE HERE ***"
    global solutionDFS
    controls = [(-1, 0), (1, 0), (0, 1), (0, -1)]
    global nodesVisitedDFS
    global xInitial
    actions = []
    current = xI
    nodesVisitedDFS.add(current)

    if current == xG:

        while current is not xInitial:
            parent = solutionDFS[current]
            actions.append((current[0]-parent[0], current[1]-parent[1]))
            current = parent
        actions.reverse()
        return actions, getCostOfActions(xInitial, actions, O), len(nodesVisitedDFS)

    for control in controls:
        child = (current[0] + control[0], current[1] + control[1])
        if not collisionCheck(current, control, O) and child not in nodesVisitedDFS and isInBounds(child, n, m):
            solutionDFS[child] = current
            actions, cost, number = depthFirstSearch(child, xG, n, m, O)
        if len(actions) > 0:
            return actions, cost, number
        else:
            continue

    return [], 99999, 0


def breadthFirstSearch(xI, xG, n, m, O):
    """
      Search the shallowest nodes in the search tree first [p 85].

      Your search algorithm needs to return a list of actions
      leading to a valid path to the goal. Make sure to implement a graph
      search algorithm.
      Your algorithm also needs to return the cost of the path.
      Use the getCostOfActions function to do this.
      Finally, the algorithm should return the number of visited
      nodes in your search.

    """
    "*** YOUR CODE HERE ***"
    frontier = deque()
    nodeVisited = set()
    solution = {}
    solution[xI] = xI
    actions = []
    frontier.append(xI)

    while len(frontier) > 0:
        current = frontier.popleft()
        controls = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        nodeVisited.add(current)
        if current == xG:
            while current is not xI:
                parent = solution[current]
                actions.append((current[0] - parent[0], current[1] - parent[1]))
                current = parent
            actions.reverse()
            return actions, getCostOfActions(xI, actions, O), len(nodeVisited)

        for control in controls:
            new = (current[0] + control[0], current[1] + control[1])
            if (not collisionCheck(current, control, O)) and (new not in nodeVisited) and isInBounds(new, n, m):
                frontier.append(new)
                solution[new] = current

    return [], 99999, 0

def DijkstraSearch(xI,xG,n,m,O,cost='westCost'):
    """
    Search the nodes with least cost first.

    Your search algorithm needs to return a list of actions
    leading to a valid path to the goal. Make sure to implement a graph
    search algorithm.
    Your algorithm also needs to return the total cost of the path using
    either the stayWestCost or stayEastCost function.
    Finally, the algorithm should return the number of visited
    nodes in your search.
    ""
    "*** YOUR CODE HERE ***"""

    actualCost = { xI : 0 }
    parent = { xI : xI }
    nodesVisitedDijkstra = set()
    actions = []
    pathCost = 0
    queue = deque()
    queue.append(xI)

    while queue:
        current = queue.popleft()
        controls = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        costCurrent = actualCost[current]
        nodesVisitedDijkstra.add(current)
        childCost = {}
        if current == xG:
            while current != xI:
                father = parent[current]
                actions.append((current[0] - father[0], current[1] - father[1]))
                current = father
            actions.reverse()
            if cost == 'westCost':
                pathCost = stayWestCost(xI, actions, O)
            else:
                pathCost = stayEastCost(xI, actions, O)
            return actions, pathCost, len(nodesVisitedDijkstra)

        else:
            for control in controls:
                child = (current[0] + control[0], current[1] + control[1])
                if not collisionCheck(current, control, O) and child not in nodesVisitedDijkstra and isInBounds(child, n ,m):
                    if cost == 'westCost':
                        nodeCost = costCurrent + stayWestCost(current, [control], O)
                    else:
                        nodeCost = costCurrent + stayEastCost(current, [control], O)
                    if child in actualCost:
                        if actualCost[child] >= nodeCost:
                            actualCost[child] = nodeCost
                            parent[child] = current
                    else:
                        actualCost[child] = nodeCost
                        parent[child] = current
                    childCost[child] = actualCost[child]
        if bool(childCost):
            new = min(childCost, key = childCost.get)
        else:
            unvisitedNodes = {node: actualCost[node] for node in parent if node not in nodesVisitedDijkstra}
            new = min(unvisitedNodes, key=unvisitedNodes.get)
        queue.append(new)

    return [], 99999, 0


def nullHeuristic(state,goal):
    """
# A heuristic function estimates the cost from the current state to the nearest
# goal.  This heuristic is trivial.
"""
    return 0

def iscollision(x, y, O):
    nextx = [x, y]
    for l in range(len(O)):
        west, east = [O[l][0], O[l][1]]
        south, north = [O[l][2], O[l][3]]
        if west <= nextx[0] <= east and south <= nextx[1] <= north:
            return True
    return False

def manhattenDist(x, xG):
    manhattenDist  = abs(xG[0] - x[0]) + abs(xG[1] -x[1])
    return manhattenDist

def euclideanDist(x, xG):
    euclideanDist = np.sqrt((xG[0] - x[0])**2 + (xG[1] - x[1])**2)
    return euclideanDist


def aStarSearch(xI, xG, n, m, O, heu):

    grid = []
    actions = []
    for _ in range(n):
        grid.append([0] * m)

    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            if iscollision(i, j, O) == True:
                grid[i][j] = 1
    grid[xG[0]][xG[1]] = 2

    width = len(grid[0])
    height = len(grid)

    compare = lambda state: state[2] + state[3]
    if (heu) == 'manhattenDist':
        heuristic = manhattenDist
    elif (heu) == 'euclideanDist':
        heuristic = euclideanDist

    dictionary = [(xI, list(), 0, heuristic(xI, xG))]
    nodesVisitedaStar = {}

    while True:
        state = dictionary.pop(0)
        (i, j) = state[0]
        if grid[i][j] == 2:
            path = [state[0]] + state[1]
            path.reverse()
            for position in range(len(path) - 1):
                temp = (path[position + 1][0] - path[position][0], path[position + 1][1] - path[position][1])
                actions.append(temp)
            return actions, getCostOfActions(xI, actions, O), len(nodesVisitedaStar)

        nodesVisitedaStar[(i, j)] = state[2]

        child = list()
        if i > 0 and grid[i - 1][j] != 1:
            child.append((i - 1, j))
        if i < height - 1 and grid[i + 1][j] != 1:
            child.append((i + 1, j))
        if j > 0 and grid[i][j - 1] != 1:
            child.append((i, j - 1))
        if j < width - 1 and grid[i][j + 1] != 1:
            child.append((i, j + 1))

        for k in child:
            next_cost = state[2] + 1
            if k in nodesVisitedaStar and nodesVisitedaStar[k] <= next_cost:
                continue
            dictionary.append((k, [state[0]] + state[1], next_cost, heuristic(k, xG)))

        dictionary.sort(key=compare)


# Plots the path
def showPath(xI, xG, path, n, m, O):
    gridpath = makePath(xI, xG, path, n, m, O)
    fig, ax = plt.subplots(1, 1)  # make a figure + axes
    ax.imshow(gridpath)  # Plot it
    ax.invert_yaxis()  # Needed so that bottom left is (0,0)


if __name__ == '__main__':
    # Run test using smallMaze.py (loads n,m,O)
    # from smallMaze import *
    # from testGrid import *
    from mediumMaze import *  # try these mazes too
    # from bigMaze import *     # try these mazes too
    # from testGrid import *
    maze(n, m, O)  # prints the maze

    # Sample collision check
    # x, u = (5,4), (1,0)
    # testObs = [[6,6,4,4]]
    # collided = collisionCheck(x,u,testObs)
    # print('Collision!' if collided else 'No collision!')

    # Sample path plotted to goal
    xI = (1, 1)
    xG = (34, 16)

    # xI = (19, 32)
    # xG = (9, 32)
    #
    # xI = (9, 32)
    # xG = (8, 20)
    #
    # xI = (8, 20)
    # xG = (5, 5)

    # actions = [(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(0,1),
    #           (1,0),(1,0),(1,0),(0,-1),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0)]

    # Cost of that path with various cost functions
    # simplecost = getCostOfActions(xI,actions,O)
    # westcost = stayWestCost(xI,actions,O)
    # eastcost = stayEastCost(xI,actions,O)
    # print('Basic cost was %d, stay west cost was %d, stay east cost was %d' %
    #      (simplecost,westcost,eastcost))

    # Code for plotting the search algorithms

    # ########################################
    #                  #DFS#
    # ########################################
    # actionsofDFS, costOfDFS, visitedNodesofDFS = depthFirstSearch(xI, xG, n, m, O)
    # path = getPathFromActions(xI, actionsofDFS)
    # print('The DFS algorithm visits', visitedNodesofDFS, 'nodes')
    # print('The actions are: ', actionsofDFS)
    # print('The cost to go the goal for DFS is ', costOfDFS, '\n')
    # showPath(xI, xG, path, n, m, O)
    # plt.show()

    # # ########################################
    #                  #BFS#                 #
    # ########################################
    # actionsofBFS, costOfBFS, visitedNodesofBFS = breadthFirstSearch(xI, xG, n, m, O)
    # path = getPathFromActions(xI, actionsofBFS)
    # print('The BFS algorithm visits', visitedNodesofBFS,'nodes')
    # print('The actions are: ', actionsofBFS)
    # print('The cost to go the goal for BFS is ',costOfBFS, '\n')
    # showPath(xI, xG, path, n, m, O)
    # plt.show()

    # ########################################
    #          #Dijkstra West Cost#          #
    # ########################################
    # actionsofDijkstra, costOfDijkstra, visitedNodesofDijkstra = DijkstraSearch(xI, xG, n, m, O, 'westCost')
    # path = getPathFromActions(xI, actionsofDijkstra)
    # print('The Dijkstra algorithm visits', visitedNodesofDijkstra, 'nodes')
    # print('The actions are: ', actionsofDijkstra)
    # print('The cost to go the goal for Dijkstra is ', costOfDijkstra, '\n')
    # showPath(xI, xG, path, n, m, O)
    # plt.show()
    #
    # #########################################
    # #          #Dijkstra East Cost#         #
    # #########################################
    # actionsofDijkstra, costOfDijkstra, visitedNodesofDijkstra = DijkstraSearch(xI, xG, n, m, O, 'eastCost')
    # path = getPathFromActions(xI, actionsofDijkstra)
    # print('The Dijkstra algorithm visits', visitedNodesofDijkstra, 'nodes')
    # print('The actions are: ', actionsofDijkstra)
    # print('The cost to go the goal for Dijkstra is ', costOfDijkstra, '\n')
    # showPath(xI, xG, path, n, m, O)
    # plt.show()

    # #########################################
    # #            #A-star Manhatten#         #
    # #########################################
    # actionsofAstar, costOfAstar, visitedNodesofAstar = aStarSearch(xI, xG, n, m, O, 'manhattenDist')
    # path = getPathFromActions(xI, actionsofAstar)
    # print('The A-star algorithm visits', visitedNodesofAstar, 'nodes')
    # print('The actions are: ', actionsofAstar)
    # print('The cost to go the goal for A-star is ', costOfAstar, '\n')
    # showPath(xI, xG, path, n, m, O)
    # plt.show()

    # #########################################
    # #            #A-star Euclidean#         #
    # #########################################
    actionsofAstar, costOfAstar, visitedNodesofAstar = aStarSearch(xI, xG, n, m, O, 'euclideanDist')
    path = getPathFromActions(xI, actionsofAstar)
    print('The A-star algorithm visits', visitedNodesofAstar, 'nodes')
    print('The actions are: ', actionsofAstar)
    print('The cost to go the goal for A-star is ', costOfAstar, '\n')
    showPath(xI, xG, path, n, m, O)
    plt.show()






