print "Plan guide trajectory ..."
import lp_urdfs_path as tp # change here to try different demo
print "Guide planned."

from sl1m.rbprm.surfaces_from_path import *
from sl1m.constants_and_tools import *
from sl1m.problem_definition import *

# from sl1m.planner import *
# from sl1m.tools.plot_plytopes import *
from sl1m.planner_scenarios.talos.constraints import *


# def gen_pb(p0, surfaces):
    # kinematicConstraints = genKinematicConstraints(left_foot_constraints, right_foot_constraints)
    # relativeConstraints = genFootRelativeConstraints(right_foot_in_lf_frame_constraints, left_foot_in_rf_frame_constraints)
    # nphases = len(surfaces)
    # p0 = None
    # # p0 = [array([0.,0., 0.]), array([0.,0., 0.])];
    
    # res = { "p0" : p0, "c0" : None, "nphases": nphases}
    
    # phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [copyKin(kinematicConstraints) for _ in range(len(surfaces[i]))], "relativeK" : [relativeConstraints[(i) % 2] for _ in range(len(surfaces[i]))], "S" : surfaces[i] } for i in range(nphases)]
    # res ["phaseData"] = phaseData
    # return res 
    
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
    
    res = { "p0" : p0, "c0" : None, "nphases": nphases}
    
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
  

#### get waypoints
step_size = 0.1
configs = getConfigsFromPath(ps, pathId, step_size)

waypoints = ps.getWaypoints(pathId)[0]

configs = array(configs)[:,:3].T
waypoints = array(waypoints)[:,:3].T

configs_xs = configs[0]
configs_ys = configs[1]
configs_zs = configs[2]

waypoints_xs = waypoints[0]
waypoints_ys = waypoints[1]
waypoints_zs = waypoints[2]



#### plot waypoints
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(configs_xs, configs_ys, configs_zs, 'blue', linewidth=2.)
ax.plot(waypoints_xs, waypoints_ys, waypoints_zs, 'red')
ax.scatter(waypoints_xs, waypoints_ys, waypoints_zs, color='red')

ax.set_xlim([-1, 7])
ax.set_ylim([-4,4])
ax.set_zlim([-4,4])

plt.ion()
plt.show()



#### MPC-style
