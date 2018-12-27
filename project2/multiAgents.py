# multiAgents.py
# --------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
  """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
  """


  def getAction(self, gameState):
    """
    You do not need to change this method, but you're welcome to.

    getAction chooses among the best options according to the evaluation function.

    Just like in the previous project, getAction takes a GameState and returns
    some Directions.X for some X in the set {North, South, West, East, Stop}
    """
    # Collect legal moves and successor states
    legalMoves = gameState.getLegalActions()

    # Choose one of the best actions
    scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
    bestScore = max(scores)
    bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
    chosenIndex = random.choice(bestIndices) # Pick randomly among the best

    "Add more of your code here if you want to"

    return legalMoves[chosenIndex]

  def evaluationFunction(self, currentGameState, action):
    """
    Design a better evaluation function here.

    The evaluation function takes in the current and proposed successor
    GameStates (pacman.py) and returns a number, where higher numbers are better.

    The code below extracts some useful information from the state, like the
    remaining food (newFood) and Pacman position after moving (newPos).
    newScaredTimes holds the number of moves that each ghost will remain
    scared because of Pacman having eaten a power pellet.

    Print out these variables to see what you're getting, then combine them
    to create a masterful evaluation function.
    """
    # Useful information you can extract from a GameState (pacman.py)
    
    successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = successorGameState.getPacmanPosition()
    newFood = successorGameState.getFood().asList()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    #These are essentially the  used as weights
    food_Score = 10.0
    ghost_Score = 10.0

    score = successorGameState.getScore()

    ghostDist = manhattanDistance(newPos, newGhostStates[0].getPosition())
    if ghostDist > 0:
      score -= ghost_Score / ghostDist

    foodDist = [manhattanDistance(newPos, x) for x in newFood]
    score += food_Score / min(foodDist) if len(foodDist) else 0
      

    return score



def scoreEvaluationFunction(currentGameState):
  """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
  """
  return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
  """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
  """

  def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
    self.index = 0 # Pacman is always agent index 0
    self.evaluationFunction = util.lookup(evalFn, globals())
    self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
  """
    Your minimax agent (question 2)
  """

  def getAction(self, gameState):
    """
      Returns the minimax action from the current gameState using self.depth
      and self.evaluationFunction.

      Here are some method calls that might be useful when implementing minimax.

      gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

      Directions.STOP:
        The stop direction, which is always legal

      gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

      gameState.getNumAgents():
        Returns the total number of agents in the game
    """
    "*** YOUR CODE HERE ***"


      
    def MiniMax(gameState, agentIndex, Depth = 0):
            
      if Depth == depthToExplore:
        return (self.evaluationFunction(gameState), Directions.STOP)

           
      nextAgentIndex = (agentIndex + 1) % agentsNumber
      nextDepth = Depth + 1
      nodes = []
      actions = gameState.getLegalActions(agentIndex)
      for action in actions:
        successors = gameState.generateSuccessor( agentIndex, action)
        v, _ = MiniMax( successors, nextAgentIndex, nextDepth)
        nodes.append((v, action))

      if len(nodes) == 0:
        return (self.evaluationFunction(gameState), Directions.STOP)

      if agentIndex == 0:
        return max(nodes)
      elif agentIndex >0:
        return min(nodes) 

            

    agentsNumber = gameState.getNumAgents()
    depthToExplore = self.depth * agentsNumber
    _, action = MiniMax(gameState, 0) 
    return action
    

      
    



class AlphaBetaAgent(MultiAgentSearchAgent):
  """
    Your minimax agent with alpha-beta pruning (question 3)
  """

  def getAction(self, gameState):
    """
      Returns the minimax action using self.depth and self.evaluationFunction
    """
    "*** YOUR CODE HERE ***"
    max_value = float("inf")

        
    def Alpha_Beta(currentGameState, agentIndex, current_Depth, alpha, beta):

      if current_Depth == depthToExplore:
        return (self.evaluationFunction(currentGameState), Directions.STOP)

            # Decide on whether minimization or maximization should be performed
      toMaximize = (agentIndex == 0)

      actions = currentGameState.getLegalActions(agentIndex)
      if len(actions) == 0:
        return (self.evaluationFunction(currentGameState), Directions.STOP)
            
      nextAgentIndex = (agentIndex + 1) % agentsNumber
      nextDepth = current_Depth + 1

      if agentIndex ==0:
        v = -max_value
      else:
        v = max_value
      best = -1
      for action in actions:
        succGameState = currentGameState.generateSuccessor( agentIndex, action )
        action_value, _ = Alpha_Beta( succGameState, nextAgentIndex, nextDepth, alpha, beta )
                
        if agentIndex ==0:
          if action_value > v:
            v = action_value
            best = action
          if v > beta:
            return (v, best)
          alpha = max(v, alpha)
        else:
          if action_value < v:
            v = action_value
            best = action
          if v < alpha:
            return (v, best)
          beta = min(v, beta)

      return (v, best)
            

    agentsNumber = gameState.getNumAgents()
    depthToExplore = self.depth * agentsNumber
        
    _, action = Alpha_Beta(gameState, 0, 0, -max_value, max_value)
    return action



    

class ExpectimaxAgent(MultiAgentSearchAgent):
  """
    Your expectimax agent (question 4)
  """

  def getAction(self, gameState):
    """
      Returns the expectimax action using self.depth and self.evaluationFunction

      All ghosts should be modeled as choosing uniformly at random from their
      legal moves.
    """
    "*** YOUR CODE HERE ***"

    def max_value(gameState, Depth):
      current_depth = Depth +1
      if gameState.isWin() | gameState.isLose() | current_depth == self.depth:
        return self.evaluationFunction(gameState)

      v = -float("inf")
      actions = gameState.getLegalActions(0)

      for action in actions:
        succ = gameState.generateSuccessor(0, action)
        v = max(v, expected_value(succ, 1, current_depth))
      return v

    def expected_value(gameState, agentIndex, Depth):
      if gameState.isWin() | gameState.isLose():
        return self.evaluationFunction(gameState) 


      actions = gameState.getLegalActions(agentIndex)
      exp_value = 0
      for action in actions:
        succ = gameState.generateSuccessor(agentIndex, action)
        if agentIndex == (gameState.getNumAgents() - 1):
          v = max_value(succ, Depth)

        else:
          v = expected_value(succ, agentIndex+1, Depth)
        exp_value = exp_value + v
      
      return float(exp_value)/float(len(actions))


    actions = gameState.getLegalActions(0)
    Score = -float("inf")
    for action in actions:
      succ = gameState.generateSuccessor(0, action)
      score = expected_value(succ, 1, 0)
      if score > Score:
        Action = action
        Score = score
    return Action















def betterEvaluationFunction(currentGameState):
  """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
  """
  "*** YOUR CODE HERE ***"
  Pos = currentGameState.getPacmanPosition()
  Food = currentGameState.getFood().asList()
  GhostStates = currentGameState.getGhostStates()
  ghostPositions = currentGameState.getGhostPositions()
  ScaredTimes = [ghostState.scaredTimer for ghostState in GhostStates]
  wall = currentGameState.getFood()
  distF = wall.width + wall.height
  score = currentGameState.getScore()

  def Distance(pos, List):
    return 0 if len(List) == 0 else min( manhattanDistance(pos, p) for p in List)
  
  Coeff_Ghosts = 60
  Coeff_eat = 50
  Coeff_Food = 40
  Coeff_scaredTime = 30
  Coeff_score = 20

  Coeff_eat *= distF
  Coeff_Ghosts *= distF

  ghosts_dist_pac = 2

  evaluation = 0

  evaluation -= Coeff_eat * len(Food)

  evaluation -= Coeff_Food * Distance(Pos, Food)

  evaluation += Coeff_eat * min( Distance(Pos, ghostPositions), ghosts_dist_pac )

  evaluation += Coeff_score * score

  evaluation += Coeff_scaredTime * sum(ScaredTimes)
  

  return evaluation

    





# Abbreviation
better = betterEvaluationFunction

class ContestAgent(MultiAgentSearchAgent):
  """
    Your agent for the mini-contest
  """

  def getAction(self, gameState):
    """
      Returns an action.  You can use any method you want and search to any depth you want.
      Just remember that the mini-contest is timed, so you have to trade off speed and computation.

      Ghosts don't behave randomly anymore, but they aren't perfect either -- they'll usually
      just make a beeline straight towards Pacman (or away from him if they're scared!)
    """
    "*** YOUR CODE HERE ***"
    def max_value(gameState, Depth):
      current_depth = Depth +1
      if gameState.isWin() | gameState.isLose() | current_depth == self.depth:
        return self.evaluationFunction(gameState)

      v = -float("inf")
      actions = gameState.getLegalActions(0)

      for action in actions:
        succ = gameState.generateSuccessor(0, action)
        v = max(v, expected_value(succ, 1, current_depth))
      return v

    def expected_value(gameState, agentIndex, Depth):
      if gameState.isWin() | gameState.isLose():
        return self.evaluationFunction(gameState) 


      actions = gameState.getLegalActions(agentIndex)
      exp_value = 0
      for action in actions:
        succ = gameState.generateSuccessor(agentIndex, action)
        if agentIndex == (gameState.getNumAgents() - 1):
          v = max_value(succ, Depth)

        else:
          v = expected_value(succ, agentIndex+1, Depth)
        exp_value = exp_value + v
      
      return float(exp_value)/float(len(actions))


    actions = gameState.getLegalActions(0)
    Score = -float("inf")
    for action in actions:
      succ = gameState.generateSuccessor(0, action)
      score = expected_value(succ, 1, 0)
      if score > Score:
        Action = action
        Score = score
    return Action

















