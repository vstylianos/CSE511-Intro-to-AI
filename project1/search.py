# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack 

    frontier = Stack ()
    explored = []
    temp_solution = Stack()
    solution = []
    frontier.push(problem.getStartState())
    node = frontier.pop()
    while problem.isGoalState(node) is False:
        if node not in explored:
            explored.append(node)
            succ = problem.getSuccessors(node)
            for state, actions, stepCost in succ:
                frontier.push(state)
                temp = solution +[actions]
                temp_solution.push(temp)
        node = frontier.pop()
        solution = temp_solution.pop()
    return solution


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    "*** YOUR CODE HERE ***"

    from util import Queue  

    frontier = Queue()
    explored = []
    temp_solution = Queue()
    solution = []
    frontier.push(problem.getStartState())
    node = frontier.pop()
    while problem.isGoalState(node) is False:
        if node not in explored:
            explored.append(node)
            succ = problem.getSuccessors(node)
            for state, actions, stepCost in succ:
                frontier.push(state)
                temp = solution +[actions]
                temp_solution.push(temp)
        node = frontier.pop()
        solution = temp_solution.pop()
    return solution
    



















def uniformCostSearch(problem):
    "Search the node of least total cost first. "

    # Implementing a different approach because I was getting illegal moves using the one above.
    # On the other hand, it was working for the eightpuzzle. 

    #I am using a dictionary to story the path and cost of the locations and a PriorityQueuewithFunction 
    #for the frontier. And this is not working for the eightpuzzle!

    
    from util import PriorityQueueWithFunction
    
    temp_solution = []
    path_cost = 0
    node = problem.getStartState()
    solution = {}
    solution[node] = [temp_solution, path_cost]
    explored = []
    def cost(child):
        return solution[child][1]

    frontier = PriorityQueueWithFunction(cost)

    
    while not problem.isGoalState(node):
       
        if node not in explored:
            explored.append(node)
            successors = problem.getSuccessors(node)
            for state, actions, stepCost in successors:
                try:
                    solution[state]
                    if path_cost + stepCost < solution[state][1]:
                        solution[state] = [temp_solution +[actions], path_cost + stepCost]
                except:
                    solution[state] = [temp_solution + [actions], path_cost + stepCost]
                    frontier.push(state)
                
        node = frontier.pop()
        temp_solution = solution[node][0]
        path_cost = solution[node][1]
    return solution[node][0]

   











def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
 

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    

    "*** YOUR CODE HERE ***"
    # Astar is essentially the uniform cost search and greedy search algorithms combined!
    #The implementation is the same as the uniform cost search with an addition of the heuristi
    #in the cost function.

    from util import PriorityQueueWithFunction

    
    temp_solution = []
    path_cost = 0
    node = problem.getStartState()
    solution = {}
    solution[node] = [temp_solution, path_cost]
    explored = []
    def cost(x):
        return solution[x][1] +heuristic(x,problem)

    frontier = PriorityQueueWithFunction(cost)
    
    while problem.isGoalState(node) is False:
       
        if node not in explored:
            explored.append(node)
            successors = problem.getSuccessors(node)
            for state, actions, stepCost in successors:
                try:
                    solution[state]
                    if path_cost + stepCost < solution[state][1]:
                        solution[state] = [temp_solution +[actions], path_cost + stepCost]
                except:
                    solution[state] = [temp_solution + [actions], path_cost + stepCost]
                    frontier.push(state)
                
        node = frontier.pop()
        temp_solution = solution[node][0]
        path_cost = solution[node][1]

    return solution[node][0]
































# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
