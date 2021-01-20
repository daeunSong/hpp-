# ~ from hpp.corbaserver.rbprm.talos_abstract import Robot
from talos_rbprm.talos_abstract import Robot
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
import pickle
Robot.urdfName += "_large"

#################################################################
#################################################################

packageName = 'hpp_environments'
meshPackageName = 'hpp_environments'
INDEX = 0
TEST = True
GUIDE = True
CONTINUOUS = False
INTERSECT = True

pb_names = ['bridge_1','stairs','debris','ground']
pb_name = pb_names[INDEX]
step_nums = [16,9,14,12,36,19,0]
step_num = step_nums[INDEX]


step_size = 0.7
if CONTINUOUS:
    step_size = 1.0

SOLVER = 0 # GUROBI
CPP = False

#################################################################
#################################################################

if pb_name not in ['stairs', 'ground', 'debris']:
    f = open('data/pb_config/'+pb_name+".txt",'r')
    line = f.readline().split()
    p_start = [float(v) for v in line]; p_start[2] += 0.98
    line = f.readline().split()
    p_goal = [float(v) for v in line]; p_goal[2] += 0.98
    line = f.readline().split()
    x_min = float(line[0]); x_max = float(line[1])
    line = f.readline().split()
    y_min = float(line[0]); y_max = float(line[1])
    line = f.readline().split()
    z_min = float(line[0]); z_max = float(line[1])

#################################################################
#################################################################

vMax = 0.5# linear velocity bound for the root
aMax = 0.7# linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
if pb_name == 'ground':
    rbprmBuilder.setJointBounds ("root_joint", [-5, 5, -1.5, 1.5, 0.95, rbprmBuilder.ref_height]) #flat ground
elif pb_name == 'stairs' :
    rbprmBuilder.setJointBounds ("root_joint", [-2.5,3,0.,3.3, 0.95, 2.])
elif pb_name == 'debris' :
    rbprmBuilder.setJointBounds ("root_joint", [-10,10 ,-10, 10, 0.95, 2.])
else:
    rbprmBuilder.setJointBounds ("root_joint", [x_min, x_max, y_min, y_max, z_min + 0.95, z_max + 1.0])
rbprmBuilder.setJointBounds ('torso_1_joint', [0,0])
rbprmBuilder.setJointBounds ('torso_2_joint', [0,0])


# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
# rbprmBuilder.setFilter([Robot.rLegId,Robot.lLegId])
rbprmBuilder.setFilter(['talos_lleg_rom','talos_rleg_rom'])
rbprmBuilder.setAffordanceFilter('talos_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('talos_rleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-4,4,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
extraDofBounds = [-vMax,vMax,-vMax,vMax,-10.,10.,-aMax,aMax,-aMax,aMax,-10.,10.]
rbprmBuilder.client.robot.setExtraConfigSpaceBounds(extraDofBounds)
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()


# Creating an instance of HPP problem solver
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods :
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("Kinodynamic/forceYawOrientation",True)
ps.setParameter("Kinodynamic/synchronizeVerticalAxis",True)
ps.setParameter("Kinodynamic/verticalAccelerationBound",10.)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",mu)

if pb_name == 'stairs':
    step_size = 0.9
    ps.setParameter("Kinodynamic/velocityBound",0.3) 
    ps.setParameter("Kinodynamic/accelerationBound",0.1)
elif pb_name == 'debris':
    step_size = 0.7
    ps.setParameter("Kinodynamic/velocityBound",0.4) 
    ps.setParameter("Kinodynamic/accelerationBound",0.1)
# if pb_name == 'rubbles_stairs_1':
#     ps.setParameter("Kinodynamic/velocityBound",0.25)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",100)


# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])

if pb_name == 'ground':
    afftool.loadObstacleModel (packageName, "multicontact/ground", "planning", vf) # flat ground
elif pb_name == 'stairs':
    # ~ afftool.loadObstacleModel (packageName, "multicontact/daeun/"+pb_name, "planning", vf,reduceSizes=[0.015,0.,0.])    
    afftool.loadObstacleModel ("package://hpp_environments/urdf/multicontact/daeun/"+pb_name+".urdf", "planning", vf,reduceSizes=[0.15,0.,0.])
elif pb_name == 'debris':
    # ~ afftool.loadObstacleModel (packageName, "multicontact/daeun/"+pb_name, "planning", vf,reduceSizes=[0.015,0.,0.])    
    afftool.loadObstacleModel ("package://hpp_environments/urdf/multicontact/daeun/"+pb_name+".urdf", "planning", vf,reduceSizes=[0.03,0.,0.])
else:
    afftool.loadObstacleModel ("package://hpp_environments/urdf/multicontact/daeun/"+pb_name+".urdf", "planning", vf)
    # ~ afftool.loadObstacleModel (packageName, "multicontact/daeun/"+pb_name, "planning", vf)

#load the viewer
v = vf.createViewer(displayArrows = True)
afftool.visualiseAffordances('Support', v, v.color.lightBrown)
v.addLandmark(v.sceneName,1)

# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer ("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")


#################################################################
#################################################################

from sl1m.rbprm.surfaces_from_planning import *
from sl1m.constants_and_tools import *
from sl1m.problem_definition import *
from sl1m.planner_scenarios.talos.constraints import *
from numpy import array, asmatrix, matrix, ones

# generate problem
def gen_pb(init, c0, R, surfaces):
    nphases = len(surfaces)
    res = { "p0" : init, "c0" : c0, "nphases": nphases}

    # phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [copyKin(kinematicConstraints) for _ in range(len(surfaces[i]))], "relativeK" : [relativeConstraints[(i)%2] for _ in range(len(surfaces[i]))], "S" : surfaces[i] } for i in range(nphases)]
    phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(left_foot_constraints,right_foot_constraints,index = i, rotation = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(right_foot_in_lf_frame_constraints,left_foot_in_rf_frame_constraints,index = i, rotation = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i] } for i in range(nphases)]
    res ["phaseData"] = phaseData
    return res

def footPosFromCOM(init_com):
    lf_0 = array(init_com[0:3]) + array([0, 0.085,-0.98])
    rf_0 = array(init_com[0:3]) + array([0,-0.085,-0.98])
    return [lf_0,rf_0]


# for plotting
import mpl_toolkits.mplot3d as a3
import matplotlib.colors as colors
import matplotlib.pylab as plt
import scipy as sp
import numpy as np


def plotSurface (points, ax, plt,color_id = -1):
    xs = np.append(points[0,:] ,points[0,0] ).tolist()
    ys = np.append(points[1,:] ,points[1,0] ).tolist()
    zs = (np.append(points[2,:] ,points[2,0]) - np.ones(len(xs))*0.005*color_id).tolist()
    colors = ['r','g','b','m','y','c']
    if color_id == -1: ax.plot(xs,ys,zs)
    else: ax.plot(xs,ys,zs, color = colors[color_id%6])

def draw_scene(surfaces, ax = None):
    colors = ['r','g','b','m','y','c']
    color_id = 0
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    # [draw_rectangle(l,ax) for l in all_surfaces]
    for i, surfaces_phase in enumerate(surfaces):
        for surface in surfaces_phase:
            plotSurface(surface, ax, plt, color_id)
        color_id += 1

    plt.ion()
    plt.show()

    return ax


import pickle

def readFromFile (fileName):
  data = []
  try:
      with open(fileName,'rb') as f:
        while True:
          try:
            line = pickle.load(f)
          except EOFError:
            break
          data.append(line)
  except:
      return None
  return data[0]


#################################################################
#################################################################

run = 0
cnt = 0

if TEST:
    PLOT = True
    SAVE = False
    OPT = True
    MAX_RUN = 1
else:
    PLOT = False
    SAVE = True
    OPT = True
    MAX_RUN = 100
    v = None


from sl1m.fix_sparsity import solveMIP, solveL1
from random import *

while run < MAX_RUN :
    q_init = rbprmBuilder.getCurrentConfig ()
    ### path planning
    if pb_name == 'ground':
        q_init [0:7] = [-3.0, 0, 0.98, 0,0,0,1]
        q_goal = q_init [::]
        q_goal[0] = 3.0
        p_goal = q_goal[0:3]
    elif pb_name == 'stairs':
        q_init [0:3] = [-0.8,0.8,0.98]
        q_init[-6:-3] = [0,0,0]
        q_goal = q_init [::]
        p_goal = [1.87,0.8,1.58]
        q_goal[0:3] = p_goal
    elif pb_name == 'debris':
        q_init [0:3] = [-1.3,0.8,0.98]
        q_init[-6:-3] = [0,0,0]
        q_goal = q_init [::]
        p_goal = [1.8,0.8,0.98]
        q_goal[0:3] = p_goal
    else:
        q_init [0:3] = p_start
        q_goal = q_init [::]
        q_goal [0:3] = p_goal
        q_goal[-6:-3] = [0,0,0]

    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)

    t = ps.solve ()
    print ("done planning, optimize path ...")

    for i in range(5):
        ps.optimizePath(ps.numberPaths() -1)

    pathId = ps.numberPaths() -1
    print ("done optimizing.")


    ### contact planning
    print ("############### run : ", run+1)

    s_p0 = ps.getInitialConfig()[0:3]; init = footPosFromCOM(s_p0)
    g_p0 = ps.getGoalConfigs()[0][0:3]; goal = footPosFromCOM(g_p0)

    ################################ MIP ################################
    ### generate contact candidates
    if GUIDE:
        if CONTINUOUS:
            R, surfaces = getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,v,step_size,INTERSECT)
        else:
            R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT)
    else:
        R, surfaces = getSurfacesAll(ps,afftool,step_num)

    ### generate contact planning problem
    pb = gen_pb(init, s_p0, R, surfaces); phase = len(pb["phaseData"])
    ### solve
    res_MI = solveMIP(pb, surfaces, draw_scene, False, CPP)
    print(res_MI)

    if not res_MI.success: # MIP FAIL
        continue


    ################################ SL1M ################################
    ### generate contact candidates
    if GUIDE:
        if CONTINUOUS:
            R, surfaces = getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,v,step_size,INTERSECT)
        else:
            R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT)
    else:
        R, surfaces = getSurfacesAll(ps,afftool,step_num)

    ### generate contact planning problem
    pb = gen_pb(init, s_p0, R, surfaces)
    ### solve
    res_L1 = solveL1(pb, surfaces, draw_scene, PLOT, CPP, SOLVER, OPT)
    print(res_L1)

    run += 1
