# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    # A node is a triple (state, action, cost)
    def checkPath(node):
        if node[0] in visited:                              # Check if this node has been visited before
            return []
        visited.append(node[0])                             # Node has now been seen
        if problem.isGoalState(node[0]):                    # Check if goal state was reached
            return [node[1]]                                # Return the direction to the node to the path
        successors = util.Stack()                           # Not a goal state so we need to check successors
        for successor in problem.getSuccessors(node[0]):    # Get all successors of current node
            successors.push(successor)                      # Add successors to a stack
        while (not successors.isEmpty()):
            path = checkPath(successors.pop())              # Check if successor has path to goal state
            if (len(path) != 0):
                if (node[1] == 'Start'): 
                    return path                             # Path completed
                else:
                    return [node[1], *path]                 # Add the direction to the node to the path
        return []                                           # No goal state found from this node
    visited = []                                            # Create a list to store visited nodes (cycle detection)
    return checkPath((problem.getStartState(), 'Start', 0)) # Start the algorithm with the start state



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visited = []                                            # Create a list to store visited nodes (cycle detection)
    frontier = util.Queue()                                 # Create a Queue to maintain node visiting order
    frontier.push((problem.getStartState(), 'Start', 0))    # Push the starting state onto the queue
    predecessors = {}                                       # Create dictionary to keep track of parent nodes
    while (not frontier.isEmpty()):
        node = frontier.pop()                               # Visit the top node on the queue
        if problem.isGoalState(node[0]):                    # Check if goal state has been reached
            print("Goal State reached")
            path = []                                       # Goal state has been reached and a path needs to be created
            currNode = node
            while (currNode[1] != 'Start'):
                print(path)
                path = [currNode[1], *path]                 # Add direction to get to node to path
                currNode = predecessors[currNode]           # Set the current node to its parent node
            return path                                     # Path has been fully constructed
        if node[0] not in visited:
            visited.append(node[0])                         # Node has now been visited
            for successor in problem.getSuccessors(node[0]):
                print(successor)
                if successor not in predecessors:
                    frontier.push(successor)                # Add all successors to the queue
                    predecessors[successor] = node          # Define the predecessor to all successors as the popped node
    return []                                               # No goal state was found

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    visited = []                                            # Create a list to store visited nodes (cycle detection)
    frontier = util.PriorityQueue()                         # Create a Priority Queue to maintain node visiting order
    startNode = (problem.getStartState(), 'Start', 0)
    frontier.push(startNode, 0)                             # Push the starting state onto the priority queue
    predecessors = {}                                       # Create dictionary to keep track of parent nodes
    cost = {startNode : 0}                                  # Create dictionary to keep track of cheapest cost to get to a node
    while (not frontier.isEmpty()):
        node = frontier.pop()                               # Visit the top node on the priority queue (cheapest cost node)
        if problem.isGoalState(node[0]):                    # Check if goal state has been reached
            path = []                                       # Goal state has been reached and a path needs to be created
            currNode = node
            while (currNode[1] != 'Start'):
                path = [currNode[1], *path]                 # Add direction to get to node to path
                currNode = predecessors[currNode]           # Set the current node to its parent node
            return path                                     # Path has been fully constructed
        if node[0] not in visited:
            visited.append(node[0])                         # Node has now been visited
            for successor in problem.getSuccessors(node[0]):
                currCost = cost[node] + successor[2]
                if successor[0] not in visited:
                    frontier.update(successor, currCost)    # Add all successors to the queue
                if (successor not in cost)\
                        or (currCost < cost[successor]):
                    predecessors[successor] = node          # Define the predecessor to all successors as the popped node
                    cost[successor] = currCost
    return []                                               # No goal state was found

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    frontier = util.PriorityQueue()                        #Create a priority queue to track visited nodes
    startNode = (problem.getStartState(), 'Start',0)
    frontier.push(startNode, 0)                            #Adds start state with cost 0
    visited = []                                           #Keep track of visited nodes
    predecessors = {}
    cost = {startNode : 0}
    while (not frontier.isEmpty()):
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            path = []                                       # Goal state has been reached and a path needs to be created
            currNode = node
            while (currNode[1] != 'Start'):
                path = [currNode[1], *path]                 # Add direction to get to node to path
                currNode = predecessors[currNode]           # Set the current node to its parent node
            return path                                     # Path has been fully constructed
        if node[0] not in visited:
            visited.append(node[0])                         # Node has now been visited
            for successor in problem.getSuccessors(node[0]):
                currCost = cost[node] + successor[2]
                costAll = currCost + heuristic(successor[0], problem)
                if successor[0] not in visited:
                    frontier.update(successor, costAll)    # Add all successors to the queue
                if (successor not in cost)\
                        or (costAll < cost[successor]):
                    predecessors[successor] = node          # Define the predecessor to all successors as the popped node
                    cost[successor] = costAll
    return []                                               #No goal state found

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
