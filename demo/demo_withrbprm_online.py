############################################################################## 
########################### initial path planning ############################
##############################################################################

print "Plan guide trajectory ..."

from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.talos_abstract import Robot
import pickle
Robot.urdfName += "_large"

packageName = 'hpp_environments'
meshPackageName = 'hpp_environments'
pbName = 'stairs_1'

f = open('data/pb_config/'+pbName+'.txt','r')
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

rbprmBuilder = Robot ()
# rbprmBuilder.setJointBounds ("root_joint", [-3.2,1.8,0.19,0.21, 0.95,1.7])
rbprmBuilder.setJointBounds ("root_joint", [x_min, x_max, y_min, y_max, z_min + 0.95, z_max + 1.0])
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
afftool.loadObstacleModel (packageName, "multicontact/daeun/"+pbName, "planning", vf)#,reduceSizes=[0.07,0.,0.])
v = vf.createViewer(displayArrows = True)
afftool.visualiseAffordances('Support', v, [0.25, 0.5, 0.5])
v.addLandmark(v.sceneName,1)

from sl1m.rbprm.surfaces_from_path import *
from random import *
import numpy as np
all_surfaces = getAllSurfaces(afftool)
a = np.array(all_surfaces[randrange(len(all_surfaces)/3*2-2,len(all_surfaces)-3)][0])
#p_goal = [uniform(a.min(axis=0)[0], a.max(axis=0)[0]), uniform(a.min(axis=0)[1], a.max(axis=0)[1]), uniform(a.min(axis=0)[2], a.max(axis=0)[2])]; p_goal[2] += 1.00
##p_goal = list(a.mean(axis=0)); p_goal[2] += 0.98
p_goal[0]-=1.0
#a = np.array(all_surfaces[randrange(0,len(all_surfaces)/3)][0])
#p_start = list(a.mean(axis=0)); p_start[2] += 1.00

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
q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = p_start 
# q_init [3:7] = [ 0., 0., 1., 0.]; v (q_init)
q_goal = q_init [::]
q_goal [0:3] = p_goal #top of stairs
q_goal[-6:-3] = [0,0,0]; v (q_goal)
# q_goal [3:7] = [ 0., 0., 1., 0.]; v (q_goal)
ps.setInitialConfig (q_init)
q_init_0 = q_init[::]
ps.addGoalConfig (q_goal)
v(q_goal)

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
#pp.displayVelocityPath(pathId)
#v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
pp.dt = 0.01


q_far = q_goal[::]
q_far[2] = -5
v(q_far)
q_init = q_init_0[::]

print "Guide planned."


from sl1m.rbprm.surfaces_from_path import *
from sl1m.constants_and_tools import *

from numpy import array, asmatrix, matrix, zeros, ones

from sl1m.planner import *
from sl1m.tools.plot_plytopes import *
from sl1m.planner_scenarios.talos.constraints import *


def gen_pb(root_init, R, surfaces):
    
    # nphases = len(surfaces)
    # lf_0 = array(root_init[0:3]) + array([0, 0.085,-0.98]) # values for talos ! 
    # rf_0 = array(root_init[0:3]) + array([0,-0.085,-0.98]) # values for talos ! 
    # p0 = [lf_0,rf_0];
    # kinematicConstraints = genKinematicConstraints(left_foot_constraints, right_foot_constraints)
    # relativeConstraints = genFootRelativeConstraints(right_foot_in_lf_frame_constraints, left_foot_in_rf_frame_constraints)
    nphases = len(surfaces)
    # p0 = [array([0.,0., 0.]), array([0.,0., 0.])] 
    p0 = None
    #p0 = [root_init[0:3],root_init[0:3]];
    
    res = { "p0" : p0, "c0" : None, "nphases": nphases}
    
    #TODO in non planar cases, K must be rotated
    #phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [copyKin(kinematicConstraints) for _ in range(len(surfaces[i]))], "relativeK" : [relativeConstraints[(i)%2] for _ in range(len(surfaces[i]))], "S" : surfaces[i] } for i in range(nphases)]
    phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(left_foot_constraints,right_foot_constraints,index = i, rotation = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(right_foot_in_lf_frame_constraints,left_foot_in_rf_frame_constraints,index = i, rotation = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i] } for i in range(nphases)]
    res ["phaseData"] = phaseData
    return res 

import mpl_toolkits.mplot3d as a3
import matplotlib.colors as colors
import scipy as sp

all_surfaces = []

def draw_rectangle(l, ax):
    #~ plotPoints(ax,l)
    l = l[0]
    lr = l + [l[0]]
    cx = [c[0] for c in lr]
    cy = [c[1] for c in lr]
    cz = [c[2] for c in lr]
    ax.plot(cx, cy, cz)

def plotSurface (points, ax, plt,color_id = -1):
    xs = np.append(points[0,:] ,points[0,0] ).tolist()
    ys = np.append(points[1,:] ,points[1,0] ).tolist()
    zs = (np.append(points[2,:] ,points[2,0] ) - np.ones(len(xs))*0.005*color_id).tolist()
    colors = ['r','g','b','m','y','c']
    if color_id == -1: ax.plot(xs,ys,zs)
    else: ax.plot(xs,ys,zs,colors[color_id])
        
def draw_scene(surfaces, ax = None):
    colors = ['r','g','b','m','y','c']
    color_id = 0
    if ax is None:        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    # [draw_rectangle(l,ax) for l in all_surfaces]
    for surfaces_phase in surfaces: 
      for surface in surfaces_phase:
        plotSurface(surface, ax, plt,color_id)
      color_id += 1
      if color_id >= len(colors):
        color_id = 0
        
    plt.ion()
    plt.show()
    
    return ax    


############################################################################## 
############################ data initialization #############################
##############################################################################

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
  
fileName = "data/replanning_gurobi/stairs_fixed"#all_in_one_rubbles"#+pbName
    
run = 0
step_size = 1.0
        
all_surfaces = getAllSurfaces(afftool) # only needed for plotting
surfaces_dict = getAllSurfacesDict(afftool)   
    
#phase_num = 0;         candidate_num = 0
#total_mip_comp = 0;    total_sl1m_comp = 0
cnt = 0
    
data = readFromFile(fileName)
    
if data != None:
    phase_num = data[0]
    candidate_num = data[1]
    mip_comp = data[2]
    max_mip_comp = data[3]
    min_mip_comp = data[4]
    sl1m_comp = data[5]
    max_sl1m_comp = data[6]
    min_sl1m_comp = data[7]
else :    
    phase_num = []
    candidate_num = []
    mip_comp = []
    sl1m_comp = []
    max_mip_comp = -1;     
    max_sl1m_comp = -1
    min_mip_comp = 100000; 
    min_sl1m_comp = 100000


############################################################################## 
############################ initial MIP solving #############################
##############################################################################

from sl1m.fix_sparsity import solveL1_gr, solveMIP_gr, solveMIP

configs = getConfigsFromPath (ps, pathId, step_size) # need for non-continuous function
R, surfaces = getSurfacesFromPath(rbprmBuilder, configs, surfaces_dict, v, False, False)

pb = gen_pb(q_init, R, surfaces)
pb, res, time_MI = solveMIP_gr(pb, surfaces, True, draw_scene, False)

if pb is not None : 
    print "### INITIAL MIP successful"
else : 
    print "### INITIAL MIP fail"
# print "MIP DONE"

############################################################################## 
###################### replanning from random position #######################
##############################################################################

from random import *
from sl1m import planner_l1 as pl1

### find the n-k th phase
k = 5#randrange(2,6)
phaseLen = len(pb["phaseData"])
startPhase = phaseLen -k -1


### set initial position again
q_init = configs[startPhase]; v(q_init)
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

### redo planning
t = ps.solve ()
print "done planning, optimize path ..."

for i in range(5):
  ps.optimizePath(ps.numberPaths() -1)

pathId = ps.numberPaths() -1
print "done optimizing."
v(q_far)
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)
pp.dt=0.1
pp.displayVelocityPath(pathId)
v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
pp.dt = 0.01

### solve it with MIP
configs = getConfigsFromPath (ps, pathId, step_size) # need for non-continuous function
R, surfaces = getSurfacesFromPath(rbprmBuilder, configs, surfaces_dict, v, False, False)
surfaces[0] = [surfaces[0][0]]
surfaces[len(surfaces)-1] = [surfaces[len(surfaces)-1][len(surfaces[len(surfaces)-1])-1]]

pb = gen_pb(q_init, R, surfaces)
pb, res, time_MI = solveMIP_gr(pb, surfaces, True, draw_scene, True)

if pb is not None : 
    print "### MIP successful"
else : 
    print "### MIP fail"


CONT = True
### solve it with SL1M
#R, surfaces = getSurfacesFromPathContinuous_(rbprmBuilder, ps, surfaces_dict, pathId, v, step_size, False)
configs = getConfigsFromPath (ps, pathId, step_size) # need for non-continuous function
R, surfaces = getSurfacesFromPath(rbprmBuilder, configs, surfaces_dict, v, False, False)
surfaces[0] = [surfaces[0][0]]
surfaces[len(surfaces)-1] = [surfaces[len(surfaces)-1][len(surfaces[len(surfaces)-1])-1]]

pb = gen_pb(q_init, R, surfaces); phase = len(pb["phaseData"])
pb, res, time_l1 = solveL1_gr(pb, surfaces, draw_scene, True)

if type(pb) is int : 
    print "### L1 fail"
    CONT = False
    
else : 
    print "### L1 successful"



############################################################################## 
################################ data saving #################################
##############################################################################

if CONT :    
    total_candidate = 0
    for surfs in surfaces:
        total_candidate += len(surfs)

    phase_num += [phase]
    candidate_num += [(float(total_candidate)-2)/(float(phase)-2)]
    #total_mip_comp += time_MI
    #total_sl1m_comp += time_l1

    if min_mip_comp > time_MI: min_mip_comp = time_MI
    if max_mip_comp < time_MI: max_mip_comp = time_MI
    if min_sl1m_comp > time_l1: min_sl1m_comp = time_l1
    if max_sl1m_comp < time_l1: max_sl1m_comp = time_l1

    mip_comp += [time_MI]
    sl1m_comp += [time_l1]
        

    #phase_num = phase_num/run
    #candidate_num = candidate_num/run  # average candidate per phase
    #mip_comp_avg = total_mip_comp/run
    #sl1m_comp_avg = total_sl1m_comp/run

            
    data = [phase_num, candidate_num, mip_comp, max_mip_comp, min_mip_comp, sl1m_comp, max_sl1m_comp, min_sl1m_comp]


    with open(fileName,'wb') as f:
        pickle.dump(data,f)
        
