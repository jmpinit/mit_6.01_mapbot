import maze

reload(maze)
import search
reload(search)
import lib601.util as util
import lib601.sonarDist as sonarDist
import time
import math
from soar.io import io
import soar.outputs.simulator as sim
import random
import checkoff
import noiseModel

import lib601.dist as dist
import lib601.markov as markov

def e(x):
	if len(x) > 117:
		return checkoff.encode(x[:117]) + 'G' + e(x[117:])
	return checkoff.encode(x) 

###### SETUP

NOISE_ON = True
SPEED = 4
ANG_SPEED = 10

bigFrustrationWorld = [0.2, util.Point(7.0, 1.0), (-0.5, 8.5, -0.5, 8.5)]
frustrationWorld = [0.15, util.Point(3.5, 0.5), (-0.5, 5.5, -0.5, 5.5)]
raceWorld = [0.1, util.Point(2.0, 5.5), (-0.5, 5.5, -0.5, 8.5)]
bigPlanWorld = [0.25, util.Point(3.0, 1.0), (-0.5, 10.5, -0.5, 10.5)]
realRobotWorld = [0.1, util.Point(1.5,0.0), (-2.0, 6.0, -2.0, 6.0)]
robotRaceWorld = [0.1, util.Point(3.0,0.0), (-2.0, 6.0, -2.0, 6.0)]
																 
THE_WORLD = raceWorld
(gridSquareSize, goalPoint, (xMin, xMax, yMin, yMax)) = THE_WORLD

###### SOAR CONTROL

# this function is called when the brain is (re)loaded 
def setup():
	#initialize robot's internal map
	width = int((xMax-xMin)/gridSquareSize)
	height = int((yMax-yMin)/gridSquareSize)
	robot.map = maze.DynamicRobotMaze(height,width,xMin,yMin,xMax,yMax)
	robot.map.redrawWorld()
	robot.map.update()
	sim.SONAR_VARIANCE = (lambda mean: 0.001) if NOISE_ON else (lambda mean: 0) #sonars are accurate to about 1 mm
	robot.plan = None

	robot.dirty = False

	prior = {}
	
	def transModel(s):
		if(s): return dist.DDist({True:1, False:0})
		return dist.DDist({True:0, False:1})

	def obsModel(s):
		if(s):
			return dist.DDist({True:0.9, False:0.1})
		else:
			return dist.DDist({True:0.1, False:0.9})

	robot.mentalmap = {}
	for y in range(0, height):
		for x in range(0, width):
			robot.mentalmap[(y, x)] = markov.StateEstimator(dist.DDist({True:0.1, False:0.9}), transModel, obsModel)

# this function is called when the start button is pushed
def brainStart():
	robot.count = 0
	robot.startTime = time.time()

# this function is called 10 times per second
def step():
	robot.count += 1
	inp = io.SensorInput(cheat=True)
	for c in ('orange','cyan','blue','red'):
		robot.map.clearColor(c)

	# discretize sonar readings
	# each element in discreteSonars is a tuple (d, cells)
	# d is the distance measured by the sonar
	# cells is a list of grid cells (r,c) between the sonar and the point d meters away
	discreteSonars = []
	for (sonarPose,d) in zip(sonarDist.sonarPoses,inp.sonars):
		if NOISE_ON:
			d = noiseModel.noisify(d,gridSquareSize)
		discreteSonars.append((d,util.lineIndices(robot.map.pointToIndices(inp.odometry.transformPose(sonarPose)), robot.map.pointToIndices(sonarDist.sonarHit(d, sonarPose, inp.odometry)))))
	
	# update map
	for (d,cells) in discreteSonars:
		# update probabilities
		if(d<=1.5):
			robot.mentalmap[cells[-1]].update(True)
			for c in cells[:-1]:
				robot.mentalmap[c].update(False)
		else:
			for i in range(0, len(cells)/4):
				robot.mentalmap[cells[i]].update(False)

		# update the map
		for c in cells:
			belief = robot.mentalmap[c].belief
			state = belief.maxProbElt()
			if(state):
				robot.map.sonarHit(c)
			else:
				robot.map.sonarPass(c)

	# null the path if necessary
	if not robot.plan is None:
		for pt in robot.plan:
			if(not robot.map.isPassable(pt)):
				robot.plan = None
				break

	if robot.plan is None:
		print 'REPLANNING'
		robot.dirty = True
		robot.plan = search.ucSearch(search.MazeSearchNode(robot.map,
							  robot.map.pointToIndices(inp.odometry.point()),None,0), 
							  lambda x: x == robot.map.pointToIndices(goalPoint), 
							  lambda x: 0)

	# graphics (draw robot's plan, robot's location, goalPoint)
	# do not change this block
	robot.map.markCells(robot.plan,'blue')
	robot.map.markCell(robot.map.pointToIndices(inp.odometry.point()),'red')
	robot.map.markCell(robot.map.pointToIndices(goalPoint),'green')

	# move to target point (similar to driving task in DL2)
	currentPoint = inp.odometry.point()
	currentAngle = inp.odometry.theta

	destinationPoint = robot.map.indicesToPoint(robot.plan[0])
	
	# check if we are ahead in the path
	if(robot.dirty):
		for i in range(0, len(robot.plan)):
			if(robot.map.pointToIndices(currentPoint)==robot.plan[i]):
				destination = robot.map.indicesToPoint(robot.plan[i+1])

	while currentPoint.isNear(destinationPoint,0.1) and len(robot.plan)>1:
		robot.plan.pop(0)
		destinationPoint = robot.map.indicesToPoint(robot.plan[0])

	if not currentPoint.isNear(destinationPoint,0.1):
		angle = util.fixAnglePlusMinusPi(currentPoint.angleTo(destinationPoint)-currentAngle)
		if abs(angle)<0.1:
			#if close enough to pointing, use proportional control on forward velocity
			fv = SPEED*currentPoint.distance(destinationPoint)
			rv = 0
		else:
			#otherwise, use proportional control on angular velocity
			fv = 0
			rv = ANG_SPEED*angle
	else:
		t = time.time() - robot.startTime;o=inp.odometry.xytTuple();w=THE_WORLD[:1]+[THE_WORLD[1].xyTuple()]+THE_WORLD[2:];n=NOISE_ON
		raise Exception, 'Goal Reached!\n\n%s' % e(repr((t,o,w,n)))
	robot.map.update()
	io.Action(fvel=fv,rvel=rv).execute()

	robot.dirty = True

# called when the stop button is pushed
def brainStop():
	stopTime = time.time()
	print 'Total steps:', robot.count
	print 'Elapsed time in seconds:', stopTime - robot.startTime
