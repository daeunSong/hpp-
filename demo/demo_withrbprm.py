print "Plan guide trajectory ..."
import lp_urdfs_path as tp # change here to try different demo
print "Guide planned."

# from surfaces_from_path import *
# from plot_surfaces import draw

from sl1m.rbprm.surfaces_from_path import *

from sl1m.constants_and_tools import *

from numpy import array, asmatrix, matrix, zeros, ones
from numpy import array, dot, stack, vstack, hstack, asmatrix, identity, cross, concatenate
from numpy.linalg import norm


from sl1m.planner import *


from sl1m.tools.plot_plytopes import *
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
    
    nphases = len(surfaces)
    #nphases = 20
    lf_0 = array(root_init[0:3]) + array([0, 0.085,-0.98]) # values for talos ! 
    rf_0 = array(root_init[0:3]) + array([0,-0.085,-0.98]) # values for talos ! 
    #p0 = [array([-3.0805096486250154, 0.335, 0.]genFootRelativeConstraints), array([-3.0805096486250154, 0.145,0.])];  ## FIXME : get it from planning too
    #p0 = [array([-0.1805096486250154, 0.335, 0.]), array([-0.1805096486250154, 0.145,0.])];  ## FIXME : get it from planning too
    p0 = [lf_0,rf_0];
    # print "p0 used : ",p0
    
    res = { "p0" : p0, "c0" : None, "nphases": nphases}
    #res = { "p0" : None, "c0" : None, "nphases": nphases}
    
    # print "surfaces = ",surfaces
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
  
############# main ###################    

if __name__ == '__main__':
    
    step_size = 1.15

    configs = getConfigsFromPath (tp.ps, tp.pathId, step_size)
    all_surfaces = getAllSurfaces(tp.afftool) # only needed for plotting
    surfaces_dict = getAllSurfacesDict(tp.afftool)   

    # R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, False)

    # draw(surfaces, all_surfaces) 
    # draw(surfaces) # plot the result

    from sl1m.fix_sparsity import solveL1, solveMIP#, solveL1_MIP

    R, surfaces = getSurfacesFromPathContinuous(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, False)
    # R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
    pb_MI = gen_pb(tp.q_init, R, surfaces)
    # pb = gen_pb(tp.q_init, surfaces)
    pb, res, time_MI = solveMIP(pb_MI, surfaces, True, draw_scene)
    
    if type(pb) is int:
        print "### MIP fail"
    else:
        print "### MIP successful"

    # step_size = 1.0
    R, surfaces = getSurfacesFromPathContinuous(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, True)
    # R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, False)
    pb_L1_i = gen_pb(tp.q_init, R, surfaces)
    # pb = gen_pb(tp.q_init, surfaces)
    # pb, coms, footpos, allfeetpos, res = solveL1(pb, surfaces, draw_scene)
    pb, res, time_l1_int = solveL1(pb_L1_i, surfaces, draw_scene)
    
    if type(pb) is int:
        print "### L1 with intersection fail"
    else:
        print "### L1 with intersection successful"

    # step_size = 1.0
    R, surfaces = getSurfacesFromPathContinuous(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, False)
    # R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
    pb_L1 = gen_pb(tp.q_init, R, surfaces)
    # pb = gen_pb(tp.q_init, surfaces)
    # pb, coms, footpos, allfeetpos, res = solveL1(pb, surfaces, draw_scene)
    pb, res, time_l1 = solveL1(pb_L1, surfaces, draw_scene)

    if type(pb) is int:
        print "### L1 fail"
    else:
        print "### L1 successful"

    

