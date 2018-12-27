# inference.py
# ------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

import util
import random
import busters
import game
import itertools

class InferenceModule:
  """
  An inference module tracks a belief distribution over a ghost's location.
  This is an abstract class, which you should not modify.
  """
  
  ############################################
  # Useful methods for all inference modules #
  ############################################
  
  def __init__(self, ghostAgent):
	"Sets the ghost agent for later access"
	self.ghostAgent = ghostAgent
	self.index = ghostAgent.index

  def getJailPosition(self):
	 return (2 * self.ghostAgent.index - 1, 1)
	
  def getPositionDistribution(self, gameState):
	"""
	Returns a distribution over successor positions of the ghost from the given gameState.
	
	You must first place the ghost in the gameState, using setGhostPosition below.
	"""
	ghostPosition = gameState.getGhostPosition(self.index) # The position you set
	actionDist = self.ghostAgent.getDistribution(gameState)
	dist = util.Counter()
	for action, prob in actionDist.items():
	  successorPosition = game.Actions.getSuccessor(ghostPosition, action)
	  dist[successorPosition] = prob
	return dist
  
  def setGhostPosition(self, gameState, ghostPosition):
	"""
	Sets the position of the ghost for this inference module to the specified
	position in the supplied gameState.
	"""
	conf = game.Configuration(ghostPosition, game.Directions.STOP)
	gameState.data.agentStates[self.index] = game.AgentState(conf, False)
	return gameState
  
  def observeState(self, gameState):
	"Collects the relevant noisy distance observation and pass it along."
	distances = gameState.getNoisyGhostDistances()
	if len(distances) >= self.index: # Check for missing observations
	  obs = distances[self.index - 1]
	  self.observe(obs, gameState)
	  
  def initialize(self, gameState):
	"Initializes beliefs to a uniform distribution over all positions."
	# The legal positions do not include the ghost prison cells in the bottom left.
	self.legalPositions = [p for p in gameState.getWalls().asList(False) if p[1] > 1]   
	self.initializeUniformly(gameState)
	
  ######################################
  # Methods that need to be overridden #
  ######################################
  
  def initializeUniformly(self, gameState):
	"Sets the belief state to a uniform prior belief over all positions."
	pass
  
  def observe(self, observation, gameState):
	"Updates beliefs based on the given distance observation and gameState."
	pass
  
  def elapseTime(self, gameState):
	"Updates beliefs for a time step elapsing from a gameState."
	pass
	
  def getBeliefDistribution(self):
	"""
	Returns the agent's current belief state, a distribution over
	ghost locations conditioned on all evidence so far.
	"""
	pass

class ExactInference(InferenceModule):
  """
  The exact dynamic inference module should use forward-algorithm
  updates to compute the exact belief function at each time step.
  """
  
  def initializeUniformly(self, gameState):
	"Begin with a uniform distribution over ghost positions."
	self.beliefs = util.Counter()
	for p in self.legalPositions: self.beliefs[p] = 1.0
	self.beliefs.normalize()
  
  def observe(self, observation, gameState):
	"""
	Updates beliefs based on the distance observation and Pacman's position.
	
	The noisyDistance is the estimated manhattan distance to the ghost you are tracking.
	
	The emissionModel below stores the probability of the noisyDistance for any true 
	distance you supply.  That is, it stores P(noisyDistance | TrueDistance).

	self.legalPositions is a list of the possible ghost positions (you
	should only consider positions that are in self.legalPositions).

	A correct implementation will handle the following special case:
	  *  When a ghost is captured by Pacman, all beliefs should be updated so
		 that the ghost appears in its prison cell, position self.getJailPosition()

		 You can check if a ghost has been captured by Pacman by
		 checking if it has a noisyDistance of None (a noisy distance
		 of None will be returned if, and only if, the ghost is
		 captured).
		 
	"""
	noisyDistance = observation
	emissionModel = busters.getObservationDistribution(noisyDistance)
	pacmanPosition = gameState.getPacmanPosition()
	jail = self.getJailPosition()
	
	"*** YOUR CODE HERE ***"
	# Replace this code with a correct observation update
	# Be sure to handle the jail.
	allPossible = util.Counter()
	if noisyDistance == None:
		allPossible[jail] = 1
	else:
		for p in self.legalPositions:
			trueDistance = util.manhattanDistance(p, pacmanPosition)
			if emissionModel[trueDistance] > 0: 
				allPossible[p] += emissionModel[trueDistance]*self.beliefs[p] 
	
	allPossible.normalize()
		
	"*** YOUR CODE HERE ***"
	self.beliefs = allPossible
	
  def elapseTime(self, gameState):
	"""
	Update self.beliefs in response to a time step passing from the current state.
	
	The transition model is not entirely stationary: it may depend on Pacman's
	current position (e.g., for DirectionalGhost).  However, this is not a problem,
	as Pacman's current position is known.

	In order to obtain the distribution over new positions for the
	ghost, given its previous position (oldPos) as well as Pacman's
	current position, use this line of code:

	  newPosDist = self.getPositionDistribution(self.setGhostPosition(gameState, oldPos))

	Note that you may need to replace "oldPos" with the correct name
	of the variable that you have used to refer to the previous ghost
	position for which you are computing this distribution.

	newPosDist is a util.Counter object, where for each position p in self.legalPositions,
	
	newPostDist[p] = Pr( ghost is at position p at time t + 1 | ghost is at position oldPos at time t )

	(and also given Pacman's current position).  You may also find it useful to loop over key, value pairs
	in newPosDist, like:

	  for newPos, prob in newPosDist.items():
		...

	As an implementation detail (with which you need not concern
	yourself), the line of code above for obtaining newPosDist makes
	use of two helper methods provided in InferenceModule above:

	  1) self.setGhostPosition(gameState, ghostPosition)
		  This method alters the gameState by placing the ghost we're tracking
		  in a particular position.  This altered gameState can be used to query
		  what the ghost would do in this position.
	  
	  2) self.getPositionDistribution(gameState)
		  This method uses the ghost agent to determine what positions the ghost
		  will move to from the provided gameState. The ghost must be placed
		  in the gameState with a call to self.setGhostPosition above.
	"""
	
	"*** YOUR CODE HERE ***"
	pos = self.legalPositions
	new_pos_distr = util.Counter()
	for p in pos:
		pos_distr = self.getPositionDistribution(self.setGhostPosition(gameState, p))
		for d in pos_distr:
			prob_new = pos_distr[d]
			prob_old = self.beliefs[p]
			new_pos_distr[d] += prob_new*prob_old
	self.beliefs = new_pos_distr


  def getBeliefDistribution(self):
	return self.beliefs

class ParticleFilter(InferenceModule):
  """
  A particle filter for approximately tracking a single ghost.
  
  Useful helper functions will include random.choice, which chooses
  an element from a list uniformly at random, and util.sample, which
  samples a key from a Counter by treating its values as probabilities.
  """

  
  def __init__(self, ghostAgent, numParticles=300):
	 InferenceModule.__init__(self, ghostAgent);
	 self.setNumParticles(numParticles)
  
  def setNumParticles(self, numParticles):
	self.numParticles = numParticles

  
  def initializeUniformly(self, gameState):
	"Initializes a list of particles. Use self.numParticles for the number of particles"
	"*** YOUR CODE HERE ***"
	pos = self.legalPositions
	numPart = self.numParticles
	self.particles = []
	particle = 0
	while particle < numPart:
		for p in pos:
			if particle < numPart:
				particle += 1
				self.particles.append(p)
  
  def observe(self, observation, gameState):
	"""
	Update beliefs based on the given distance observation. Make
	sure to handle the special case where all particles have weight
	0 after reweighting based on observation. If this happens,
	resample particles uniformly at random from the set of legal
	positions (self.legalPositions).

	A correct implementation will handle two special cases:
	  1) When a ghost is captured by Pacman, all particles should be updated so
		 that the ghost appears in its prison cell, self.getJailPosition()

		 You can check if a ghost has been captured by Pacman by
		 checking if it has a noisyDistance of None (a noisy distance
		 of None will be returned if, and only if, the ghost is
		 captured).
		 
	  2) When all particles receive 0 weight, they should be recreated from the
		  prior distribution by calling initializeUniformly. Remember to
		  change particles to jail if called for.
	"""
	noisyDistance = observation
	emissionModel = busters.getObservationDistribution(noisyDistance)
	pacmanPosition = gameState.getPacmanPosition()
	"*** YOUR CODE HERE ***"
	belief_dist = self.getBeliefDistribution()
	new_belief_dist = util.Counter()

	if noisyDistance is None:
		self.particles = []
		for i in range(self.numParticles): 
			self.particles.append(self.getJailPosition())
	
	else:
		for pos in self.legalPositions:
			trueDistance = util.manhattanDistance(pacmanPosition,pos)
			if emissionModel[trueDistance] > 0:
				new_belief_dist[pos] = float(emissionModel[trueDistance]) * float(belief_dist[pos])
			
			# Case when total weight of new Distribution is 0
		if new_belief_dist.totalCount() == 0:
			self.initializeUniformly(gameState)
		else:
			self.particles =[]
			new_belief_dist.normalize()
			for i in range(self.numParticles):
				self.particles.append(util.sample(new_belief_dist))


	
  def elapseTime(self, gameState):
	"""
	Update beliefs for a time step elapsing.

	As in the elapseTime method of ExactInference, you should use:

	  newPosDist = self.getPositionDistribution(self.setGhostPosition(gameState, oldPos))

	to obtain the distribution over new positions for the ghost, given
	its previous position (oldPos) as well as Pacman's current
	position.
	"""
	"*** YOUR CODE HERE ***"
	new_particles = []
	new_pos_distr = util.Counter()
	for i in self.particles:
		new_pos_distr = self.getPositionDistribution(self.setGhostPosition(gameState, i))
		new_particles.append(util.sample(new_pos_distr))
	self.particles = new_particles

  def getBeliefDistribution(self):
	"""
	Return the agent's current belief state, a distribution over
	ghost locations conditioned on all evidence and time passage.
	"""
	"*** YOUR CODE HERE ***"
	distr = util.Counter()

	for p in self.particles:
		distr[p] +=1
	distr.normalize()
	return distr

class MarginalInference(InferenceModule):
  "A wrapper around the JointInference module that returns marginal beliefs about ghosts."

  def initializeUniformly(self, gameState):
	"Set the belief state to an initial, prior value."
	if self.index == 1: jointInference.initialize(gameState, self.legalPositions)
	jointInference.addGhostAgent(self.ghostAgent)
	
  def observeState(self, gameState):
	"Update beliefs based on the given distance observation and gameState."
	if self.index == 1: jointInference.observeState(gameState)
	
  def elapseTime(self, gameState):
	"Update beliefs for a time step elapsing from a gameState."
	if self.index == 1: jointInference.elapseTime(gameState)
	
  def getBeliefDistribution(self):
	"Returns the marginal belief over a particular ghost by summing out the others."
	jointDistribution = jointInference.getBeliefDistribution()
	dist = util.Counter()
	for t, prob in jointDistribution.items():
	  dist[t[self.index - 1]] += prob
	return dist
  
class JointParticleFilter:
	"JointParticleFilter tracks a joint distribution over tuples of all ghost positions."

	def __init__(self, numParticles=600):
		self.setNumParticles(numParticles)

	def setNumParticles(self, numParticles):
		self.numParticles = numParticles

	def initialize(self, gameState, legalPositions):
		"Stores information about the game, then initializes particles."
		self.numGhosts = gameState.getNumAgents() - 1
		self.ghostAgents = []
		self.legalPositions = legalPositions
		self.initializeParticles()

	def initializeParticles(self):
		"Initializes particles randomly.  Each particle is a tuple of ghost positions. Use self.numParticles for the number of particles"
		"*** YOUR CODE HERE ***"
		pos = list(itertools.product(self.legalPositions, repeat = self.numGhosts))
		random.shuffle(pos)

		count = 0
		self.particles = list()
		while count < self.numParticles:
			for position in pos:
				if count < self.numParticles:
					self.particles.append(position)
					count += 1	

	def addGhostAgent(self, agent):
		self.ghostAgents.append(agent)
	
  	def elapseTime(self, gameState):
	 
		newParticles = []
		for oldParticle in self.particles:
			newParticle = list(oldParticle) # A list of ghost positions
			for i in range(self.numGhosts):
				post = getPositionDistributionForGhost(setGhostPositions(gameState, newParticle),
														i, self.ghostAgents[i])
				newParticle[i] = util.sample(post)

			newParticles.append(tuple(newParticle))
		self.particles = newParticles



	def getJailPosition(self, i):
		return (2 * i + 1, 1);
  
	def observeState(self, gameState):
	
		pacmanPosition = gameState.getPacmanPosition()
		noisyDistances = gameState.getNoisyGhostDistances()
		if len(noisyDistances) < self.numGhosts: 
			return
		emissionModels = [busters.getObservationDistribution(dist) for dist in noisyDistances]

		u = 1
		post_distr = util.Counter()
		for i in range(self.numParticles):
			particle = self.particles[i]
			prob = u
			for j in range(self.numGhosts):
				if noisyDistances[j] !=None:
					dist = util.manhattanDistance(particle[j], pacmanPosition)
					prob *= emissionModels[j][dist]
				else:
					particle = self.getGhostInJail(particle, j)
			post_distr[particle] += prob

		Bool = False
		for k in post_distr:
			if post_distr[k] != 0:
				Bool = True
				break
		if not Bool:
			self.initializeParticles()
			for i in range(self.numParticles):
				particle = self.particles[i]
				for j in range(self.numGhosts):
					if noisyDistances[j] == None:
						particle = self.getGhostInJail(particle, j)

			return
		post_distr.normalize()
		self.particles = [util.sample(post_distr) for i in range(self.numParticles)]


	def getGhostInJail(self, particle, index):
		particle = list(particle)
		particle[index] = self.getJailPosition(index)
		return tuple(particle)
  
	def getBeliefDistribution(self):
		dist = util.Counter()
		particles = self.particles
		for part in particles: 
			dist[part] += 1
		dist.normalize()
		return dist

# One JointInference module is shared globally across instances of MarginalInference 
jointInference = JointParticleFilter()

def getPositionDistributionForGhost(gameState, ghostIndex, agent):
  """
  Returns the distribution over positions for a ghost, using the supplied gameState.
  """

  # index 0 is pacman, but the students think that index 0 is the first ghost.
  ghostPosition = gameState.getGhostPosition(ghostIndex+1) 
  actionDist = agent.getDistribution(gameState)
  dist = util.Counter()
  for action, prob in actionDist.items():
	successorPosition = game.Actions.getSuccessor(ghostPosition, action)
	dist[successorPosition] = prob
  return dist
  

def setGhostPositions(gameState, ghostPositions):
  "Sets the position of all ghosts to the values in ghostPositionTuple."
  for index, pos in enumerate(ghostPositions):
	conf = game.Configuration(pos, game.Directions.STOP)
	gameState.data.agentStates[index + 1] = game.AgentState(conf, False)
  return gameState  

