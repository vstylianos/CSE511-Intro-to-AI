# valueIterationAgents.py
# -----------------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

import mdp, util

from learningAgents import ValueEstimationAgent

class ValueIterationAgent(ValueEstimationAgent):
  """
      * Please read learningAgents.py before reading this.*

      A ValueIterationAgent takes a Markov decision process
      (see mdp.py) on initialization and runs value iteration
      for a given number of iterations using the supplied
      discount factor.
  """
  def __init__(self, mdp, discount = 0.9, iterations = 100):
    """
      Your value iteration agent should take an mdp on
      construction, run the indicated number of iterations
      and then act according to the resulting policy.
    
      Some useful mdp methods you will use:
          mdp.getStates()
          mdp.getPossibleActions(state)
          mdp.getTransitionStatesAndProbs(state, action)
          mdp.getReward(state, action, nextState)
    """
    self.mdp = mdp
    self.discount = discount
    self.iterations = iterations
    self.values = util.Counter() # A Counter is a dict with default 0
     
    States = mdp.getStates()

    for i in range(self.iterations):
      values1 = util.Counter()
      for s in States:
        if self.mdp.isTerminal(s):
          continue
        actions = self.mdp.getPossibleActions(s)
        act_dict = util.Counter()
        for a in actions:
          values2 = []
          transitions = self.mdp.getTransitionStatesAndProbs(s, a)
          for (next_state, prob) in transitions:
            reward = self.mdp.getReward(s, a , next_state)
            values2.append(prob*(reward +self.discount*self.values[next_state]))
          act_dict[a] = sum(values2)
        values1[s] = max(list(act_dict.values()))
      self.values = values1












    
  def getValue(self, state):
    """
      Return the value of the state (computed in __init__).
    """
    return self.values[state]


  def getQValue(self, state, action):
    """
      The q-value of the state action pair
      (after the indicated number of value iteration
      passes).  Note that value iteration does not
      necessarily create this quantity and you may have
      to derive it on the fly.
    """
    "*** YOUR CODE HERE ***"
    values1 = []
    transitions = self.mdp.getTransitionStatesAndProbs(state, action)
    for (next_state, prob) in transitions:
      reward = self.mdp.getReward(state, action, next_state)
      values1.append(prob*(reward+self.discount*self.values(next_state)))
    return sum(values1)


  def getPolicy(self, state):
    """
      The policy is the best action in the given state
      according to the values computed by value iteration.
      You may break ties any way you see fit.  Note that if
      there are no legal actions, which is the case at the
      terminal state, you should return None.
    """
    "*** YOUR CODE HERE ***"
    actions = self.mdp.getPossibleActions(state)
    act_dict = util.Counter()
    for action in actions:
      values1 = []
      transitions = self.mdp.getTransitionStatesAndProbs(state, action)
      for (next_state, prob) in transitions:
        reward = self.mdp.getReward(state, action, next_state)
        values1.append(prob*(reward+self.discount*self.values[next_state]))
      act_dict[action] = sum(values1)
    policy = act_dict.argMax()
    return policy

  def getAction(self, state):
    "Returns the policy at the state (no exploration)."
    actions = seld.mdp.getPossibleActions(state)
    act_dict = util.Counter()
    for action in actions:
      values1 = []
      transitions = self.mdp.getTransitionStatesAndProbs(state, action)
      for (next_state, prob) in transitions:
        reward = self.mdp.getReward(state, action, next_state)
        values1.append(prob*(reward+self.discount*self.values1[next_state]))
      act_dict[action] = sum(values1)
    policy = act_dict.argMax()
    return policy
  
