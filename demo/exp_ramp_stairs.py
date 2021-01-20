from talos_rbprm.talos_abstract import Robot
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import pickle
Robot.urdfName += "_large"

#################################################################
#################################################################

packageName = 'hpp_environments'
meshPackageName = 'hpp_environments'
I = 2
TEST = True
GUIDE = False
CONTINUOUS = True
INTERSECT = True
COST=False
OPT = True

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
ps.setParameter("Kinodynamic/velocityBound",vMax)
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
afftool.loadObstacleModel ("package://hpp_environments/urdf/multicontact/daeun/"+pbName+".urdf", "planning", vf,reduceSizes=[0.07,0.,0.])

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
    # OPT = True
    MAX_RUN = 1
else:
    PLOT = False
    SAVE = True
    # OPT = False
    MAX_RUN = 10
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
    ### path planning
    q_init [0:3] = [-1.3,0.8,0.98]
    q_init[-6:-3] = [0,0,0]
    q_goal = q_init [::]
    p_goal = [1.4,0.8,0.98]
    #p_goal = [1.6,0.8,0.98]
    q_goal[0:3] = p_goal

    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)

    s_p0 = ps.getInitialConfig()[0:3]; init = footPosFromCOM(s_p0)

    t = ps.solve ()
    print ("done planning, optimize path ...")

    for i in range(5):
        ps.optimizePath(ps.numberPaths() -1)

    path_rubbles = ps.numberPaths() -1
    print ("done optimizing.")

    ## turn around
    ps.resetGoalConfigs()
    ps.setParameter("Kinodynamic/velocityBound",0.2)
    ps.setParameter("Kinodynamic/accelerationBound",0.1)
    q_init = q_goal [::]
    q_goal[0:7] = [1.8,0.2,0.98,0,0,-0.7071,0.7071]

    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    t = ps.solve ()
    print ("done planning, optimize path ...")

    for i in range(5):
        ps.optimizePath(ps.numberPaths() -1)

    path_turn = ps.numberPaths() -1

    ## stairs
    ps.resetGoalConfigs()
    q_init = q_goal [::]
    q_goal[1]-=2.47
    # q_goal[0]+=2.47
    q_goal[2]=1.58

    ps.setInitialConfig (q_init)
    ps.addGoalConfig (q_goal)
    t = ps.solve ()
    print ("done planning, optimize path ...")

    for i in range(5):
        ps.optimizePath(ps.numberPaths() -1)

    path_stairs = ps.numberPaths() -1

    pathId = path_rubbles
    ps.concatenatePath(pathId,path_turn)
    ps.concatenatePath(pathId,path_stairs)

    # display solution :
    #from hpp.gepetto import PathPlayer
    #pp = PathPlayer (v)
    #pp.dt=0.1
    #pp.displayVelocityPath(pathId)
    #v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
    #pp.dt = 0.01
    #pp(pathId)

    # q_far = q_goal[::]
    # q_far[2] = -5
    # v(q_far)


    ### contact planning
    print ("############### run : ", run+1)

    g_p0 = ps.getGoalConfigs()[0][0:3]; goal = footPosFromCOM(g_p0)

    if COST:
        ################################ MIP ################################
        ### generate contact candidates
        if GUIDE:
            # if pbName == 'rubbles_1' or 'stairs_2':
            #     R, surfaces = getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,v,step_size,INTERSECT)
            # else:
            # R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT)
            if CONTINUOUS:
                R, surfaces = getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,v,step_size,INTERSECT,pathId)
            else:
                R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT,pathId)
        else:
            R, surfaces_ = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT,pathId)
            R, surfaces = getSurfacesAll(ps,afftool,step_num)
            surfaces[-1]=surfaces_[-1]

        ### generate contact planning problem
        pb = gen_pb(init, s_p0, R, surfaces); phase = len(pb["phaseData"])
        ### solve
        # res_MI = solveMIP_cost(pb, surfaces,p_goal, draw_scene, PLOT, CPP)
        res_MI = solveMIP(pb, surfaces, draw_scene, PLOT, CPP, OPT)

        print(res_MI)

        if not res_MI.success: # MIP FAIL
            fail+=1
            continue
        else:
            mip_comp_gr += [res_MI.time]


        total_candidate = 0
        for surfs in surfaces:
            total_candidate += len(surfs)

        phase_num += [phase]
        candidate_num += [float(total_candidate-2)/float(len(surfaces)-2)]

        run += 1

    else:
        ################################ MIP ################################
        ### generate contact candidates
        if GUIDE:
            if CONTINUOUS:
                R, surfaces = getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,None,step_size,INTERSECT,pathId)
            else:
                R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,None,step_size,INTERSECT,pathId)
        else:
            R, surfaces_ = getSurfacesFromGuide(rbprmBuilder,ps,afftool,None,step_size,INTERSECT,pathId)
            R, surfaces = getSurfacesAll(ps,afftool,step_num)
            surfaces[-1]=surfaces_[-1]

        ### generate contact planning problem
        pb = gen_pb(init, s_p0, R, surfaces); phase = len(pb["phaseData"])
        ### solve
        res_MI = solveMIP(pb, surfaces, draw_scene, PLOT, CPP)

        print(res_MI)

        if not res_MI.success: # MIP FAIL
            continue
        else:
            mip_comp_gr += [res_MI.time]

        total_candidate = 0
        for surfs in surfaces:
            total_candidate += len(surfs)

        ################################ SL1M ################################
        ### generate contact candidates
        #if GUIDE:
        #    if CONTINUOUS:
        #        R, surfaces = getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,v,step_size,INTERSECT,pathId)
        #    else:
        #        R, surfaces = getSurfacesFromGuide(rbprmBuilder,ps,afftool,v,step_size,INTERSECT,pathId)
        #else:
        #    R, surfaces = getSurfacesAll(ps,afftool,step_num)
        #    surfaces[-1]=surfaces_[-1]

        ### generate contact planning problem
        pb = gen_pb(init, s_p0, R, surfaces)
        ### solve
        # res_L1 = solveL1Reweighted(pb, surfaces, draw_scene, PLOT, CPP, SOLVER, OPT)
        res_L1 = solveL1(pb, surfaces, draw_scene, PLOT, CPP, SOLVER, OPT)
        print(res_L1)

        if not res_L1.success: # SL1M FAIL
            fail1 += 1
            failcase +=[res_L1.case]
        else:
            sl1m_comp_gr += [res_L1.time]

        ########################## validation with MIP ##########################

        # if res_L1.success:
        #     # ~ pb_ = readFromFile("pb")
        #     pb_ = gen_pb(init, s_p0, R, surfaces)
        #     surfaces_ = []
        #     for phase_ in pb_["phaseData"]:
        #         surfaces_ += [phase_["S"]]
        #     res_MI_validation = solveMIP(pb_, surfaces_, draw_scene, PLOT, CPP)
        #     print(res_MI_validation)
        #
        #     if not res_MI_validation.success: # SL1M UNFEASIBLE
        #         fail2 += 1
        #     else:
        #         sl1m_comp_gr += [res_L1.time]
        #         if TEST:
        #             with open(fileName+"_res",'wb') as f:
        #                 pickle.dump(res_L1.res, f)
        #
        # phase_num += [phase]
        # candidate_num += [float(total_candidate-2)/float(len(surfaces)-2)]

        run += 1

# if COST:
#     data = [phase_num, candidate_num, mip_comp_gr, fail]
#     if SAVE:
#         with open(fileName,'wb') as f:
#             pickle.dump(data,f)
# else:
#     data = [phase_num, candidate_num, mip_comp_gr, sl1m_comp_gr, fail1, fail2, failcase, THRESHOLD]
#     if SAVE:
#         with open(fileName,'wb') as f:
#             pickle.dump(data,f)
#
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
cs = build_cs_from_sl1m_mip(pb, allfeetpos, fb2, q_init, q_end,z_offset= 0.01)
cs.saveAsBinary("talos_ramp_stairs2.cs")

v(q_init)
