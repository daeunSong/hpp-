from talos_rbprm.talos_abstract import Robot
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import pickle
# Robot.urdfName += "_large_reducedROM"

#################################################################
#################################################################

packageName = 'hpp_environments'
meshPackageName = 'hpp_environments'
I = 2
TEST = True
GUIDE = False
CONTINUOUS = False
INTERSECT = True
COST=False

#pbNames = ['bridge_1','stairs','debris','rubbles_1','rubbles_stairs_1','ground', 'playground']
#step_nums = [16,9,14,12,36,19,0]
pbName = 'ramp_stairs2'#pbNames[I]
step_num = 29

if COST:
    folderName = "exp_cost/1/"
else:
    folderName = "exp/1/"
fileName = folderName+pbName

if not GUIDE:
    fileName += "_WO"
    #step_num = step_nums[I]
else:
    if INTERSECT:
        fileName += "_Wi"
    else: fileName += "_Ww"
    if CONTINUOUS:
        fileName += "_c"

step_size = 0.75
if CONTINUOUS:
    step_size = 0.85

SOLVER = 0 # GUROBI
CPP = True

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
rbprmBuilder.setJointBounds ("root_joint", [-10,10 ,-10, 10, 0.95, 2.])
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
ps.setParameter("Kinodynamic/velocityBound",0.2)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("Kinodynamic/forceYawOrientation",True)
ps.setParameter("Kinodynamic/synchronizeVerticalAxis",True)
ps.setParameter("Kinodynamic/verticalAccelerationBound",10.)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",mu)
ps.setParameter("Kinodynamic/velocityBound",0.4)
ps.setParameter("Kinodynamic/accelerationBound",0.1)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",100)


# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])


# ~ afftool.loadObstacleModel ("package://hpp_environments/urdf/multicontact/daeun/"+pbName+".urdf", "planning", vf,reduceSizes=[0.015,0.,0.])
afftool.loadObstacleModel ("package://hpp_environments/urdf/multicontact/bauzil_stairs.urdf", "planning", vf,reduceSizes=[0.015,0.,0.])

#load the viewer
v = vf.createViewer(displayArrows = False)
# afftool.visualiseAffordances('Support', v, v.color.white)
# v.addLandmark(v.sceneName,1)
v.client.gui.setBackgroundColor1(v.sceneName,[1.,1.,1.,1.])
v.client.gui.setBackgroundColor2(v.sceneName,[1.,1.,1.,1.])


# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer ("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")


# # display solution :
# from hpp.gepetto import PathPlayer
# pp = PathPlayer (v)
# pp.dt=0.1
# pp.displayVelocityPath(pathId)
# v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
# pp.dt = 0.01
# #pp(pathId)

# q_far = q_goal[::]##
# q_far[2] = -5
# v(q_far)


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
data = readFromFile(fileName)

if TEST:
    PLOT = False
    SAVE = False
    OPT = True
    MAX_RUN = 1
else:
    PLOT = False
    SAVE = True
    OPT = False
    MAX_RUN = 100
    v = None

if COST:
    if data != None:
        phase_num = data[0]
        candidate_num = data[1]
        mip_comp_gr = data[2]
        fail = data[3]
    else :
        phase_num = []
        candidate_num = []
        mip_comp_gr = []
        fail = 0
else:
    if data != None:
        phase_num = data[0]
        candidate_num = data[1]
        mip_comp_gr = data[2]
        sl1m_comp_gr = data[3]
        fail1 = data[4]
        fail2 = data[5]
        failcase = data[6]
    else :
        phase_num = []
        candidate_num = []
        mip_comp_gr = []
        sl1m_comp_gr = []
        fail1 = 0
        fail2 = 0
        failcase = []




from sl1m.fix_sparsity import solveMIP, solveMIP_cost, solveL1
from random import *

while run < MAX_RUN :
    ps.resetGoalConfigs()
    q_init = rbprmBuilder.getCurrentConfig ()

    ### BEGIN climb the stairs #####
    ps.setParameter("Kinodynamic/velocityBound",0.3)
    ps.setParameter("Kinodynamic/accelerationBound",0.1)
    q_init = rbprmBuilder.getCurrentConfig ();
    q_init [0:3] = [-0.5,1.2,0.98]
    q_init[-6:-3] = [0,0,0]
    # q_init [0:3] =  [-0.3, 0.2,0.98] ; v(q_init) # between rubble and stairs
    #q_init = q_goal[::] ; v(q_init) # between rubble and stairs
    #q_init[-6:-3] = [0.15,0,0]
    q_goal = q_init [::]
    q_goal [0:3] = [2.0, 1.2, 1.58]; v (q_goal) #top of stairs
    q_goal[-6:-3] = [0,0,0]
    ps.setInitialConfig (q_init)
    q_init_0 = q_init[::]
    ps.addGoalConfig (q_goal)
    v(q_goal)

    s_p0 = ps.getInitialConfig()[0:3]; init = footPosFromCOM(s_p0)

    t = ps.solve ()
    print("done planning, optimize path ...")
    #v.solveAndDisplay('rm',2,0.005)

    for i in range(5):
      ps.optimizePath(ps.numberPaths() -1)

    pId_stairs = ps.numberPaths() -1
    ### END climb the stairs #####
    # rbprmBuilder.setJointBounds ("root_joint", [-3.2,2.5,-1.0,0.3, 1.4,2.])
    ps.resetGoalConfigs()
    ### BEGIN turn around on the platform #####
    ps.setParameter("Kinodynamic/velocityBound",0.2)
    ps.setParameter("Kinodynamic/accelerationBound",0.07)
    q_init = rbprmBuilder.getCurrentConfig ();
    q_init = q_goal[::] ; v (q_init) #top of stairs
    q_init[-6:-3] = [0.2,0,0]
    q_goal = q_init [::]
    q_goal [0:3] = [2.0, 0.1, 1.58]; v (q_goal) # before bridge
    q_goal[3:7] = [0,0,1,0]
    q_goal[-6:-3] = [-0.2,0,0]
    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    v(q_goal)

    t = ps.solve ()
    print("done planning, optimize path ...")
    #v.solveAndDisplay('rm',2,0.005)
    for i in range(5):
      ps.optimizePath(ps.numberPaths() -1)

    pId_platform =  ps.numberPaths() -1
    ### END turn around on the platform #####
    ps.resetGoalConfigs()
    ### BEGIN bridge cross #####
    ps.setParameter("Kinodynamic/velocityBound",0.2)
    ps.setParameter("Kinodynamic/accelerationBound",0.2)
    q_init = rbprmBuilder.getCurrentConfig ();
    q_init = q_goal[::]; v (q_init) #top of stairs
    q_goal [0:3] = [-1.7, 0.1, 1.58]; v (q_goal) # after bridge
    # q_goal[3:7] = [0,0,0.7071,0.7071]; v (q_goal)
    q_goal[3:7] = [0,0,1,0]
    q_goal[-6:-3] = [0,0,0]
    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    v(q_goal)

    t = ps.solve ()
    print("done planning, optimize path ...")
    #v.solveAndDisplay('rm',2,0.005)
    for i in range(5):
      ps.optimizePath(ps.numberPaths() -1)

    pId_bridge =  ps.numberPaths() -1
    ### END bridge cross #####


    ### BEGIN climb down the stairs #####
    # ps.setParameter("Kinodynamic/velocityBound",0.3)
    # ps.setParameter("Kinodynamic/accelerationBound",0.1)
    # q_init = q_goal[::] ; v (q_init)
    # # q_init[-6:-3] = [-0.2,0,0]
    # q_goal = q_init [::]
    # q_goal [0:3] = [-1.7, 2.1, 0.98]; #v (q_goal)
    # # q_goal [3:7] = [0,0,0.7071,0.7071]
    # q_goal[-6:-3] = [0,0.0,0]
    # ps.setInitialConfig (q_init)
    # ps.addGoalConfig (q_goal)
    # v(q_goal)
    #
    # t = ps.solve ()
    # print("done planning, optimize path ...")
    # # v.solveAndDisplay('rm',2,0.005)
    #
    # for i in range(5):
    #   ps.optimizePath(ps.numberPaths() -1)
    #
    # pId_stairs_down = ps.numberPaths() -1
    ### END climb down the stairs #####

    # merge all paths
    pathId = pId_stairs
    ps.concatenatePath(pathId,pId_platform)
    ps.concatenatePath(pathId,pId_bridge)
    # ps.concatenatePath(pathId,pId_stairs_down)

    # display solution :
    from hpp.gepetto import PathPlayer
    pp = PathPlayer (v)
    pp.dt=0.1
    pp.displayPath(pathId,[0.9290, 0.6940, 0.1250, 1.0])
    # v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
    v.client.gui.setCurveLineWidth("path_"+str(pathId)+"_root",4.0)

    # q_far = q_goal[::]
    # q_far[2] = -5
    # v(q_far)


    ### contact planning
    print ("############### run : ", run+1)

    g_p0 = ps.getGoalConfigs()[0][0:3]; goal = footPosFromCOM(g_p0)
    # R, surfaces = getSurfacesAll(ps,afftool,33)
    R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,None,step_size,INTERSECT,pathId)
    # R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT,pathId)

    pb = gen_pb(init, s_p0, R, surfaces); phase = len(pb["phaseData"])
    # res_L1 = solveMIP(pb, surfaces, draw_scene, PLOT, CPP, 1)
    res_L1 = solveL1(pb, surfaces, draw_scene, False, CPP, SOLVER, OPT)
    print(res_L1)

    run += 1

#
# pathLength = ps.pathLength(pathId) #length of the path
# configs = []
# # get configuration along the path
# for s in arange (0, pathLength, step_size) :
#     configs.append(ps.configAtParam(pathId, s))
#
# phase = 16
# colors = plt.cm.Dark2.colors
# colors = list(colors)[:-2]
#
# # # for i,ss in enumerate(surfaces):
# # ss = surfaces[phase]
# # i = phase
# # for j,s in enumerate(ss):
# #     s = s.T
# #     p0 = s[0];p0[2]+=0.001
# #     s[1][2]+=0.001*(i+1)
# #     for k in range(len(s)-2):
# #         p1 = s[k+1]; p2 = s[k+2]; p3 = s[k]
# #         p2[2]+= 0.001*(i+1)
# #         name = "face"+"."+str(i)+"."+str(j)+"."+str(k)
# #         # color = list(colors[(i-2)%len(colors)])+[1.0]
# #         color = list(colors[0])+[0.5]
# #         if i%2 == 0:
# #             v.client.gui.addTriangleFace(name,p2.tolist(),p1.tolist(),p0.tolist(),color)
# #             v.client.gui.setWireFrameMode(name, "FILL")
# #             v.client.gui.addToGroup(name, v.sceneName)
# #         # v.client.gui.setWireFrameMode(name, "FILL_AND_WIREFRAME")
# #         # # v.client.gui.setVisibility(name,"ALWAYS_ON_TOP")
# #         # v.client.gui.addToGroup(name, v.sceneName)
#
#
#
# from pinocchio import XYZQUATToSE3, SE3ToXYZQUAT
# from sl1m.constants_and_tools import default_transform_from_pos_normal
# # from sl1m.tools.transformations import quaternion_from_matrix
#
# for i,config in enumerate(configs):
#     p = config[:7]
#     name = "sphere"+str(i)
#     v.client.gui.addSphere(name,0.015,[0.9290, 0.6940, 0.1250, 1.0])
#     # v.client.gui.addLandmark(name,0.15)
#     # v.client.gui.addArrow(name+"_x",0.01, 0.15,[1.,0.,0.,1.])
#     # # v.client.gui.setVisibility(name+"_x","ALWAYS_ON_TOP")
#     # v.client.gui.addArrow(name+"_y",0.01, 0.15,[0.,0.5,0.,1.])
#     # v.client.gui.addArrow(name+"_z",0.01, 0.15,[0.,0.,1.,1.])
#
#     #
#     q_rot = config[3:7]
#     R = XYZQUATToSE3(p)
#     R_y = R; R_z = R
#     R = R.rotation
#
#     # R_y.rotation = np.dot(R,array([[0,-1,0],[1,0,0],[0,0,1]]))
#     # q_rot_y = SE3ToXYZQUAT(R_y)
#     # p_y=q_rot_y.tolist()
#     # R_z.rotation = np.dot(R,array([[0,0,-1],[0,1,0],[1,0,0]]))
#     # q_rot_z = SE3ToXYZQUAT(R_z)
#     # p_z=q_rot_z.tolist()
#     #
#
#     x_end = p[:3] + np.dot(R,array([1,0,0]))*0.1
#     v.client.gui.addCurve(name+"_x",[p[:3],x_end.tolist()],[1.,0.,0.,1.])
#     y_end = p[:3] + np.dot(R,array([0,1,0]))*0.1
#     v.client.gui.addCurve(name+"_y",[p[:3],y_end.tolist()],[0.,0.5,0.,1.])
#     z_end = p[:3] + np.dot(R,array([0,0,1]))*0.1
#     v.client.gui.addCurve(name+"_z",[p[:3],z_end.tolist()],[0.,0.,1.,1.])
#     v.client.gui.setCurveLineWidth(name+"_x", 3.0)
#     v.client.gui.setCurveLineWidth(name+"_y", 3.0)
#     v.client.gui.setCurveLineWidth(name+"_z", 3.0)
#
#
#
#
#
#     v.client.gui.applyConfiguration(name, p)
#     # v.client.gui.applyConfiguration(name+"_x", p)
#     # v.client.gui.applyConfiguration(name+"_y", p_y)
#     # v.client.gui.applyConfiguration(name+"_z", p_z)
#     v.client.gui.addToGroup(name, v.sceneName)
#     v.client.gui.addToGroup(name+"_x", v.sceneName)
#     v.client.gui.addToGroup(name+"_y", v.sceneName)
#     v.client.gui.addToGroup(name+"_z", v.sceneName)
#     # v.client.gui.refresh()
#
# v.client.gui.addLight("light", v.sceneName, 0.1, [0.3,0.3,0.3,1.0])
# v.client.gui.applyConfiguration("light", [0.,0.,3.,0.,0.,0.,0.])
# v.client.gui.addToGroup("light", v.sceneName)
#
# # v.client.gui.setCurveLineWidth(name, 3.0)
# # v(configs[phase])
# q_far = configs[0]
# q_far[2]-= 5.0
# # v(q_far)
# v(configs[14])


from talos_rbprm.talos import Robot    as talosFull
fb2 = talosFull()
allfeetpos = res_L1.res[2]

z_offset=0.05

q_init = fb2.referenceConfig.copy()
# ~ q_init[:3] = p_start
q_end = q_init.copy()
# ~ q_end [0:3] = p_goal
# ~ q_end[-6:-3] = [0,0,0.]
from sl1m.constants_and_tools import replace_surfaces_with_ineq_in_problem

pb = res_L1.pb
del pb["phaseData"][0];
replace_surfaces_with_ineq_in_problem(pb)

q_init[:7] = ps.configAtParam(pathId, 0.001)[:7]
q_end [:7]= ps.configAtParam(pathId, ps.pathLength(pathId) - 0.001)[:7]
q_end[2] += z_offset
q_init[2] += z_offset

from sl1m.sl1m_to_mcapi import build_cs_from_sl1m_mip
from sl1m.sl1m_to_mcapi import build_cs_from_sl1m_mip
cs = build_cs_from_sl1m_mip(pb, allfeetpos, fb2, q_init, q_end,z_offset=0.005)
# ~ cs = build_cs_from_sl1m_mip(pb, allfeetpos, fb2, q_init, q_end,z_offset=0.05)
cs.saveAsBinary("talos_bauzil.cs")

v(q_init)
