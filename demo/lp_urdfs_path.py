from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.talos_abstract import Robot
import pickle
Robot.urdfName += "_large"

packageName = 'hpp_environments'
meshPackageName = 'hpp_environments'
# pbName = 'playground'
pbName = 'ground'

# f = open('data/pb_config/'+pbName+".txt",'r')
# line = f.readline().split()
# p_start = [float(v) for v in line]; p_start[2] += 0.98
# line = f.readline().split()
# p_goal = [float(v) for v in line]; p_goal[2] += 0.98
# line = f.readline().split()
# x_min = float(line[0]); x_max = float(line[1])
# line = f.readline().split()
# y_min = float(line[0]); y_max = float(line[1])
# line = f.readline().split()
# z_min = float(line[0]); z_max = float(line[1])


#################################################################
#################################################################
# from random import *
# LIST_LINKS = [[[[-0.5, 0.5], [1.5, 2.5], [0.0, 0.02]], [[1.5, 2.5], [1.5, 2.5], [0.0, 0.02]]], [[[-0.5, 0.5], [1.5, 2.5], [0.0, 0.02]], [[-0.5, 0.5], [5.5, 6.5], [-0.37714285714285717, -0.35714285714285715]]], [[[-0.5, 0.5], [5.5, 6.5], [-0.37714285714285717, -0.35714285714285715]], [[-0.5, 0.5], [9.5, 10.5], [-0.37714285714285717, -0.35714285714285715]]], [[[-0.5, 0.5], [9.5, 10.5], [-0.37714285714285717, -0.35714285714285715]], [[-0.5, 0.5], [11.5, 12.5], [-0.37714285714285717, -0.35714285714285715]]], [[[1.5, 2.5], [1.5, 2.5], [0.0, 0.02]], [[5.5, 6.5], [1.5, 2.5], [-0.7114285714285714, -0.6914285714285714]]], [[[1.5, 2.5], [1.5, 2.5], [0.0, 0.02]], [[1.5, 2.5], [3.5, 4.5], [0.0, 0.02]]], [[[1.5, 2.5], [3.5, 4.5], [0.0, 0.02]], [[3.5, 4.5], [3.5, 4.5], [0.0, 0.02]]], [[[1.5, 2.5], [3.5, 4.5], [0.0, 0.02]], [[1.5, 2.5], [7.5, 8.5], [-0.5666666666666667, -0.5466666666666666]]], [[[1.5, 2.5], [7.5, 8.5], [-0.5666666666666667, -0.5466666666666666]], [[3.5, 4.5], [7.5, 8.5], [-0.5666666666666667, -0.5466666666666666]]], [[[1.5, 2.5], [7.5, 8.5], [-0.5666666666666667, -0.5466666666666666]], [[1.5, 2.5], [11.5, 12.5], [-1.098095238095238, -1.078095238095238]]], [[[1.5, 2.5], [11.5, 12.5], [-1.098095238095238, -1.078095238095238]], [[1.5, 2.5], [13.5, 14.5], [-1.098095238095238, -1.078095238095238]]], [[[1.5, 2.5], [13.5, 14.5], [-1.098095238095238, -1.078095238095238]], [[1.5, 2.5], [17.5, 18.5], [-1.098095238095238, -1.078095238095238]]], [[[3.5, 4.5], [3.5, 4.5], [0.0, 0.02]], [[7.5, 8.5], [3.5, 4.5], [-0.6387499999999999, -0.6187499999999999]]], [[[3.5, 4.5], [3.5, 4.5], [0.0, 0.02]], [[3.5, 4.5], [7.5, 8.5], [-0.5666666666666667, -0.5466666666666666]]], [[[3.5, 4.5], [7.5, 8.5], [-0.5666666666666667, -0.5466666666666666]], [[7.5, 8.5], [7.5, 8.5], [-0.17500000000000004, -0.15500000000000005]]], [[[3.5, 4.5], [9.5, 10.5], [-1.354285714285714, -1.334285714285714]], [[5.5, 6.5], [9.5, 10.5], [-1.354285714285714, -1.334285714285714]]], [[[5.5, 6.5], [1.5, 2.5], [-0.7114285714285714, -0.6914285714285714]], [[9.5, 10.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]]], [[[5.5, 6.5], [9.5, 10.5], [-1.354285714285714, -1.334285714285714]], [[5.5, 6.5], [11.5, 12.5], [-1.354285714285714, -1.334285714285714]]], [[[7.5, 8.5], [3.5, 4.5], [-0.6387499999999999, -0.6187499999999999]], [[7.5, 8.5], [7.5, 8.5], [-0.17500000000000004, -0.15500000000000005]]], [[[9.5, 10.5], [-0.5, 0.5], [-0.1885714285714285, -0.1685714285714285]], [[9.5, 10.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]]], [[[9.5, 10.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]], [[13.5, 14.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]]], [[[9.5, 10.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]], [[9.5, 10.5], [5.5, 6.5], [-0.7114285714285714, -0.6914285714285714]]], [[[9.5, 10.5], [5.5, 6.5], [-0.7114285714285714, -0.6914285714285714]], [[9.5, 10.5], [9.5, 10.5], [-0.5930357142857141, -0.5730357142857141]]], [[[11.5, 12.5], [13.5, 14.5], [-0.3866666666666666, -0.3666666666666666]], [[11.5, 12.5], [15.5, 16.5], [-0.3609523809523809, -0.3409523809523809]]], [[[13.5, 14.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]], [[17.5, 18.5], [1.5, 2.5], [-0.5385714285714285, -0.5185714285714285]]], [[[13.5, 14.5], [1.5, 2.5], [-0.1885714285714285, -0.1685714285714285]], [[13.5, 14.5], [5.5, 6.5], [-0.7114285714285714, -0.6914285714285714]]], [[[13.5, 14.5], [5.5, 6.5], [-0.7114285714285714, -0.6914285714285714]], [[13.5, 14.5], [9.5, 10.5], [-0.5930357142857141, -0.5730357142857141]]], [[[13.5, 14.5], [9.5, 10.5], [-0.5930357142857141, -0.5730357142857141]], [[17.5, 18.5], [9.5, 10.5], [0.18571428571428594, 0.20571428571428593]]], [[[13.5, 14.5], [9.5, 10.5], [-0.5930357142857141, -0.5730357142857141]], [[13.5, 14.5], [11.5, 12.5], [-0.5930357142857141, -0.5730357142857141]]], [[[15.5, 16.5], [3.5, 4.5], [-1.3930357142857142, -1.3730357142857141]], [[15.5, 16.5], [5.5, 6.5], [-1.3930357142857142, -1.3730357142857141]]], [[[17.5, 18.5], [1.5, 2.5], [-0.5385714285714285, -0.5185714285714285]], [[17.5, 18.5], [5.5, 6.5], [-0.2142857142857143, -0.1942857142857143]]], [[[17.5, 18.5], [5.5, 6.5], [-0.2142857142857143, -0.1942857142857143]], [[17.5, 18.5], [9.5, 10.5], [0.18571428571428594, 0.20571428571428593]]]]
# index = randint(0, len(LIST_LINKS))
# start_r = LIST_LINKS[index][0]
# goal_r = LIST_LINKS[index][1]
# p_start = [uniform(l,u) for (l,u) in start_r]; p_start[2] += 0.98
# p_goal = [uniform(l,u) for (l,u) in goal_r]; p_goal[2] += 0.98


rbprmBuilder = Robot ()
# rbprmBuilder.setJointBounds ("root_joint", [-3.2,1.8,0.19,0.21, 0.95,1.7])
# rbprmBuilder.setJointBounds ("root_joint", [x_min, x_max, y_min, y_max, z_min + 0.95, z_max + 1.0])
rbprmBuilder.setJointBounds ("root_joint", [-5,5, -1.5, 1.5, 0.95, 1.05]) #flat ground

# As this scenario only consider walking, we fix the DOF of the torso :
rbprmBuilder.setJointBounds ('torso_1_joint', [0,0])
rbprmBuilder.setJointBounds ('torso_2_joint', [0.,0.])
vMax = 1.# linear velocity bound for the root
aMax = 2. # linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
rbprmBuilder.setFilter([Robot.rLegId,Robot.lLegId])

rbprmBuilder.setAffordanceFilter(Robot.rLegId, ['Support',])
rbprmBuilder.setAffordanceFilter(Robot.lLegId, ['Support'])
rbprmBuilder.boundSO3([-4.,4.,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
extraDofBounds = [-vMax,vMax,-vMax,vMax,-10.,10.,-aMax,aMax,-aMax,aMax,-10.,10.]
rbprmBuilder.client.robot.setExtraConfigSpaceBounds(extraDofBounds)
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

from hpp.corbaserver.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
# afftool.loadObstacleModel (packageName, "multicontact/daeun/"+pbName, "planning", vf)#,reduceSizes=[0.07,0.,0.])
afftool.loadObstacleModel ("hpp_environments", "multicontact/ground", "planning", vf) # flat ground
v = vf.createViewer(displayArrows = True)
afftool.visualiseAffordances('Support', v, [0.25, 0.5, 0.5])
v.addLandmark(v.sceneName,1)

######random goal
#from sl1m.rbprm.surfaces_from_path import *
#from random import *
#import numpy as np
#all_surfaces = getAllSurfaces(afftool)
#a = np.array(all_surfaces[randrange(0,len(all_surfaces))][0])
#p_goal = [uniform(a.min(axis=0)[0], a.max(axis=0)[0]), uniform(a.min(axis=0)[1], a.max(axis=0)[1]), uniform(a.min(axis=0)[2], a.max(axis=0)[2])]; p_goal[2] += 1.00
##p_goal = list(a.mean(axis=0)); p_goal[2] += 1.00
#############

ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
# force the orientation of the trunk to match the direction of the motion
ps.setParameter("Kinodynamic/forceYawOrientation",True)
ps.setParameter("Kinodynamic/synchronizeVerticalAxis",True)
ps.setParameter("Kinodynamic/verticalAccelerationBound",10.)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",mu)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",100)
# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer ("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")

### BEGIN climb the stairs #####
ps.setParameter("Kinodynamic/velocityBound",0.3)
ps.setParameter("Kinodynamic/accelerationBound",0.1)
# q_init = rbprmBuilder.getCurrentConfig ();
# q_init [0:3] = p_start 
# # q_init [3:7] = [ 0., 0., 1., 0.]; v (q_init)
# q_goal = q_init [::]
# q_goal [0:3] = p_goal #top of stairs
# q_goal[-6:-3] = [0,0,0]; v (q_goal)
# # q_goal [3:7] = [ 0., 0., 1., 0.]; v (q_goal)
# ps.setInitialConfig (q_init)
# ps.addGoalConfig (q_goal)
# v(q_goal)


############################################# flat ground
# Setting initial configuration
q_init = rbprmBuilder.getCurrentConfig ()
q_init[3:7] = [0,0,0,1]
q_init [0:3] = [-3.0, 0, 1.]
v (q_init)
ps.setInitialConfig (q_init)
# set goal config
rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]
q_goal[0] = 3.0
v(q_goal)
ps.addGoalConfig (q_goal)


t = ps.solve ()
print "done planning, optimize path ..."
#v.solveAndDisplay('rm',2,0.005)

for i in range(5):
  ps.optimizePath(ps.numberPaths() -1)

pathId = ps.numberPaths() -1


print "done optimizing."
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)
pp.dt=0.1
pp.displayVelocityPath(pathId)
v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
pp.dt = 0.01


q_far = q_goal[::]
q_far[2] = -5
v(q_far)

