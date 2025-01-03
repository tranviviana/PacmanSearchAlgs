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

from util import *
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    Start: (5, 5)
    Is the start a goal? False
    Start's successors: [((5, 4), 'South', 1), ((4, 5), 'West', 1)]
        A search problem defines the state space, start state, goal test, successor function and cost function.  This search problem can be used to find paths to a particular point on the pacman board.
    """
    reached = []
    frontier = Stack() #LIFO
    path = {}
    location = problem.getStartState()
    frontier.push(location)
    path[location] = []
    while not frontier.isEmpty():
        current_node = frontier.pop()
        if problem.isGoalState(current_node):
            return path[current_node]
        if current_node not in reached:
            reached.append(current_node) #the last item added not seen before
            for child in problem.getSuccessors(current_node):
                location, direction, one = child
                if location not in reached:
                    frontier.push(location)
                    path[location] = path[current_node] + [direction]
    return None
        
        
    

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodeseaFs in the search tree first."""
    "*** YOUR CODE HERE ***"
    reached = set()
    frontier = Queue() #FIFO
    path = {}
    location = problem.getStartState()
    frontier.push(location)
    path[location] = []
    reached.add(location)
    while not frontier.isEmpty():
        current_node = frontier.pop()
        if problem.isGoalState(current_node):
            return path[current_node]
        for child_siblings in problem.getSuccessors(current_node):
            location, direction, one = child_siblings
            if location not in reached:
                reached.add(location) #first sibling added not seen
                frontier.push(location)
                path[location] = path[current_node] + [direction]
    return None
    

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    reached = set()
    frontier = PriorityQueue()
    path = {}
    location = problem.getStartState()
    frontier.push(location, 0)
    path[location] = []
    location_cost = {}
    location_cost[location] = 0
    total_cost = 0
    while not frontier.isEmpty():
        current_node = frontier.pop()
        current_cost = location_cost[current_node]
        reached.add(current_node)
        if problem.isGoalState(current_node):
            return path[current_node]
        for child_sibling in problem.getSuccessors(current_node):
            location, direction, cost = child_sibling
            total_cost = current_cost + cost
            if total_cost < location_cost.get(location, float('inf')):
                location_cost[location] = total_cost
                frontier.update(location, total_cost)
                path[location] = path[current_node] + [direction]
                
            

    return None
   

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    #heuristics(state, problem)
    "*** YOUR CODE HERE ***"
    #used structures
    reached = set()
    path = {}
    best_cost = {}
    frontier = PriorityQueue()
    
    #initial values
    current_node = problem.getStartState()
    initial_cost = heuristic(current_node, problem)
    
    #initial values into structures
    best_cost[current_node] = 0
    frontier.update(current_node, initial_cost)
    path[current_node] = []
    while not frontier.isEmpty():
        current_node = frontier.pop()
        current_cost = best_cost[current_node]
        reached.add(current_node)
        
        if problem.isGoalState(current_node):
            return path.get(current_node)
            
        for child in problem.getSuccessors(current_node):
            location_child, direction_child, cost_child = child
            child_total_cost = cost_child + current_cost
            total_cost = heuristic(location_child, problem) + child_total_cost
            if total_cost < best_cost.get(location_child, float('inf')) + heuristic(location_child, problem):
                best_cost[location_child] = child_total_cost
                frontier.update(location_child, total_cost)
                path[location_child] = path[current_node] + [direction_child]
    
    
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
