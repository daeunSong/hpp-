print "Plan guide trajectory ..."
import lp_urdfs_path as tp # change here to try different demo
print "Guide planned."

from sl1m.rbprm.surfaces_from_path import *
from sl1m.constants_and_tools import *
from sl1m.problem_definition import *

# from sl1m.planner import *
# from sl1m.tools.plot_plytopes import *
from sl1m.planner_scenarios.talos.constraints import *

def footPosFromCOM(init_com):
    lf_0 = array(init_com[0:3]) + array([0, 0.085,-0.98])
    rf_0 = array(init_com[0:3]) + array([0,-0.085,-0.98])
    return [lf_0,rf_0]
    

def gen_pb(init, s_p0, goal, R, surfaces):
    
    nphases = len(surfaces)
    
    res = { "p0" : init, "c0" : s_p0, "goal" : goal, "nphases": nphases}
    #res = { "p0" : None, "c0" : None, "goal" : None, "nphases": nphases}
    
    #TODO in non planar cases, K must be rotated
    #phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [copyKin(kinematicConstraints) for _ in range(len(surfaces[i]))], "relativeK" : [relativeConstraints[(i)%2] for _ in range(len(surfaces[i]))], "S" : surfaces[i] } for i in range(nphases)]
    phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(left_foot_constraints,right_foot_constraints,index = i, rotation = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(right_foot_in_lf_frame_constraints,left_foot_in_rf_frame_constraints,index = i, rotation = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i] } for i in range(nphases)]
    res ["phaseData"] = phaseData
    return res 
    
import mpl_toolkits.mplot3d as a3
import matplotlib.colors as colors
import matplotlib.pylab as plt
import scipy as sp
import numpy as np

all_surfaces = []

def draw_rectangle(l, ax):
    #~ plotPoints(ax,l)
    l = l[0]
    lr = l + [l[0]]
    cx = [c[0] for c in lr]
    cy = [c[1] for c in lr]
    cz = [c[2] for c in lr]
    ax.plot(cx, cy, cz)

def plotSurface (points, ax, color_id = -1):
    xs = np.append(points[0,:] ,points[0,0] ).tolist()
    ys = np.append(points[1,:] ,points[1,0] ).tolist()
    zs = (np.append(points[2,:] ,points[2,0] ) - np.ones(len(xs))*0.005*color_id).tolist()
    colors = ['r','g','b','m','y','c']
    if color_id == -1: ax.plot(xs,ys,zs,'black')
    else: ax.plot(xs,ys,zs,colors[color_id])

def draw_scene(surfaces, ax = None):
    if ax is None:        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    for surface in surfaces[0]:
        plotSurface(surface, ax)
    plt.ion()
    return ax  
        
def draw_contacts(surfaces, ax = None):
    colors = ['r','g','b','m','y','c']
    color_id = 0
    if ax is None:        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    # [draw_rectangle(l,ax) for l in all_surfaces]
    for surfaces_phase in surfaces: 
      for surface in surfaces_phase:
        plotSurface(surface, ax, color_id)
      color_id += 1
      if color_id >= len(colors):
        color_id = 0
    plt.ion()
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
  

##### get waypoints
#step_size = 0.1
#configs = getConfigsFromPath(tp.ps, tp.pathId, step_size)

#waypoints = tp.ps.getWaypoints(tp.pathId)[0]


############################# just for plotting
#configs_ = array(configs)[:,:3].T
#waypoints_ = array(waypoints)[:,:3].T

#configs_xs = configs_[0]
#configs_ys = configs_[1]
#configs_zs = configs_[2]

#waypoints_xs = waypoints_[0]
#waypoints_ys = waypoints_[1]
#waypoints_zs = waypoints_[2]

#fig = plt.figure()
#ax = fig.add_subplot(111, projection="3d")

#ax.plot(configs_xs, configs_ys, configs_zs, 'blue', linewidth=2.)
#ax.plot(waypoints_xs, waypoints_ys, waypoints_zs, 'red')
#ax.scatter(waypoints_xs, waypoints_ys, waypoints_zs, color='red')

##ax.set_xlim([-1, 7])
##ax.set_ylim([-4,4])
##ax.set_zlim([-4,4])

#plt.ion()
#plt.show()
#############################


#### MPC-style
def getDist (waypoint1, waypoint2):
    return np.sqrt(np.power(waypoint2[0] - waypoint1[0],2) + np.power(waypoint2[1] - waypoint1[1],2) + np.power(waypoint2[2] - waypoint1[2],2))
    
###### TO DO 
# add a way point when the orientation changes rapidly
    
###### TO DO 
# get a rotation matrix/quat from the vector
#def getRot

surfaces_dict = getAllSurfacesDict(tp.afftool)  
step_size = 1.0
DISCRETIZE_SIZE = 0.1
EPSILON = 1.0
NUM_STEP = 4

from sl1m.fix_sparsity import solveL1_gr_cost, solveMIP_gr_cost, solveMIP, solveL1_gr, solveL1
import sl1m.planner   as pl
import sl1m.planner_l1   as pl1


goals = getConfigsFromPath (tp.ps, tp.pathId, step_size*4)
configs = getConfigsFromPath (tp.ps, tp.pathId, step_size)
configs_per_goals = []
del goals[0]

for i in range(len(goals)-1):
    configs_ = configs[4*i:4*(i+1)]
    configs_per_goals.append(configs_)

surfaces_dict = getAllSurfacesDict(tp.afftool)
all_surfaces = getAllSurfaces(tp.afftool)  

s_p0 = configs[0][0:3]; init = footPosFromCOM(s_p0)

####
configs = getConfigsFromPath (tp.ps, tp.pathId, step_size)
surfaces_dict = getAllSurfacesDict(tp.afftool)
all_surfaces = getAllSurfaces(tp.afftool)   
s_p0 = configs[0][0:3]; init = footPosFromCOM(s_p0)
g_p0 = configs[-1][0:3]; g_p0[0]+=1.0; goal = footPosFromCOM(g_p0)


R, surfaces = getSurfacesFromPath_mpc(tp.rbprmBuilder, configs, surfaces_dict, NUM_STEP, tp.v, False)
    

MIP = False
i = 0

#pb = gen_pb(init, s_p0, goal, R, surfaces)
#pb, res, time = solveL1_gr_cost(pb, surfaces, draw_scene, False, weight)
#init = [res[20:23],res[30:33]]
#s_p0 = res[30:33]; s_p0[2] +=0.98

#pb = gen_pb(init, s_p0, goal, R, surfaces)
#pb, res, time = solveL1_gr_cost(pb, surfaces, draw_scene, False, weight)
#init = [res[20:23],res[30:33]]
#s_p0 = res[30:33]; s_p0[2] +=0.98

####

MIP = False
i = 0
weight = 100


for j,goal in enumerate(goals) :
    g_p0 = goal[0:3]; goal = footPosFromCOM(g_p0)
    R, surfaces = getSurfacesFromPath_mpc(tp.rbprmBuilder, configs_per_goals[j], surfaces_dict, NUM_STEP, tp.v, False)
    weight = 0.001

    while getDist(init[0],goal[0]) > EPSILON :
        if i == 5:
            break;

    #while i < 1:
        print i,"th iteration"
        pb = gen_pb(init, s_p0, goal, R, surfaces)

        if MIP:
            pb, res, time = solveMIP_gr_cost(pb, surfaces, True, draw_scene, True)
            coms, footpos, allfeetpos = pl1.retrieve_points_from_res(pb, res)
        else:
            pb, res, time = solveL1_gr_cost(pb, surfaces, draw_scene, True, weight)
            coms, footpos, allfeetpos = pl1.retrieve_points_from_res(pb, res)
            
        #weight += 0.001
        #weight += 10
        i += 1

        init = [allfeetpos[-2],allfeetpos[-1]]
        s_p0 = coms[-1]
        print s_p0, getDist(s_p0,g_p0)
        ##i += 1

