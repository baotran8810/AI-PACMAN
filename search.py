"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from collections import defaultdict

import util
import sys
from time import sleep

from game import Directions, PriorityQueue

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST
p = Directions.STOP


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


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    frontier = util.Stack()
    walked = []
    result = []
    frontier.push((problem.getStartState(), result))
    while not frontier.isEmpty():
        successor, result = frontier.pop()
        if problem.isGoalState(successor):
            return result
        if successor not in walked:
            walked.append(successor)
            for item1, item2, item3 in problem.getSuccessors(successor)[::-1]:
                frontier.push((item1, result + [item2]))
    return result


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    frontier = util.Queue()
    walked = []
    result = []
    frontier.push((problem.getStartState(), result))
    while not frontier.isEmpty():
        successor, result = frontier.pop()
        if problem.isGoalState(successor):
            return result
        if successor not in walked:
            walked.append(successor)
            for item1, item2, item3 in problem.getSuccessors(successor):
                frontier.push((item1, result + [item2]))
    return result
    
def getDirections(startState, pathMap, expansion):
    path = [expansion[-1]]  # the path to the goal state
    precedingState = pathMap[expansion[-1]]
    i = -1  # reverse iterator
    # trace back the preceding state till start state is reached
    while precedingState != startState:
        path.insert(0, precedingState)
        i -= 1
        precedingState = pathMap[path[i]]
    path.insert(0, precedingState)  # add start state to path
    # only get non STOP actions
    directions = [state[1] for state in path if state[1] != p]
    return directions
    pass


def uniformCostSearch(problem):
    startState = (problem.getStartState(), p, 0)
    frontier = util.PriorityQueue()
    frontier.push(startState, 0)
    expansion = []
    currCost = 0  # cost of each new action
    pathMap = defaultdict(list)  # for saving which state precedes which states
    while not frontier.isEmpty():  # do UCS
        currState = frontier.pop()
        expansion.append(currState)
        currCost += currState[2]
        if problem.isGoalState(currState[0]):
            break
        else:
            successors = problem.getSuccessors(currState[0])
            for nextState in successors:
                if (nextState not in expansion) and (nextState not in frontier.heap):
                    if requireEvasiveManeuver(nextState, problem.ghostPositions):  # evade ghost checking
                        frontier.push(nextState, currCost + nextState[2] + 999999)
                    else:
                        frontier.push(nextState, currCost + nextState[2])
                    pathMap[nextState] = currState
    # return path to the goal state
    return getDirections(expansion[0], pathMap, expansion)
    pass



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic):
    expand_node = []
    path = defaultdict(list)
    frontier = PriorityQueue()
    cost = 0
    start = (problem.getStartState(), Directions.STOP, 0)
    frontier.push(start, heuristic(start, problem) + cost)
    while not frontier.isEmpty():
        current = frontier.pop()
        expand_node.append(current)
        cost = cost + current[2]
        if heuristic(current, problem) == 0:
            return getDirections(expand_node[0],path, expand_node)
        if problem._expanded > 0 and problem._expanded < 2  and detecFront(problem.ghostPositions, current):
            return getDirections(expand_node[0],path, expand_node)
        successors = problem.getSuccessors(current[0])
        for state in successors:
            if state not in expand_node and state not in frontier.heap:
                if requireEvasiveManeuver(state, problem.ghostPositions) == True or detecFront(problem.ghostPositions, state):  # evade ghost
                    frontier.push(state, heuristic(state, problem) + cost + 999999999)
                else:
                    frontier.push(state, heuristic(state, problem) + cost)
                path[state] = current
    return 0




def requireEvasiveManeuver(nextState, ghostPositions):
    dangerZone = []
    for pos in ghostPositions:
        x = pos[0]
        y = pos[1]
        dangerZone.append((x, y))
        for i in range(1, 2):
                dangerZone.append((x - i, y + i))
                dangerZone.append((x, y + i))
                dangerZone.append((x + i, y + i))
                dangerZone.append((x - i, y))
                dangerZone.append((x + i, y))
                dangerZone.append((x - i, y - i))
                dangerZone.append((x, y - i))
                dangerZone.append((x + i, y - i))
    dangerZone.append((x-2, y-2))
    dangerZone.append((x-2, y+2))
    dangerZone.append((x+2, y-2))
    dangerZone.append((x+2, y-2))
    return nextState[0][0] in dangerZone
    pass




def detecFront(ghostPositions, current):
    dangerZone = []
    x = current[0][0][0]
    y = current[0][0][1]
    for i in range(1, 2):
        if(current[1] == "West"):
            dangerZone.append((x - i, y + i))
            dangerZone.append((x - i, y))
            dangerZone.append((x - i, y - i))
        if (current[1] == "East"):
            dangerZone.append((x + i, y + i))
            dangerZone.append((x + i, y))
            dangerZone.append((x + i, y - i))
        if (current[1] == "South"):
            dangerZone.append((x - i, y - i))
            dangerZone.append((x, y - i))
            dangerZone.append((x + i, y - i))
        if (current[1] == "North"):
            dangerZone.append((x - i, y + i))
            dangerZone.append((x, y + i))
            dangerZone.append((x + i, y + i))

    for ghost in ghostPositions:
        if ghost in dangerZone:
            return True

    return False



    pass
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch