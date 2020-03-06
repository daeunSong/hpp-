print "Plan guide trajectory ..."
import lp_urdfs_path as tp # change here to try different demo
print "Guide planned."


from sl1m.rbprm.surfaces_from_path import *
from sl1m.constants_and_tools import *

from numpy import array, asmatrix, matrix, zeros, ones

from sl1m.planner import *
from sl1m.tools.plot_plytopes import *
from sl1m.planner_scenarios.talos.constraints import *


def gen_pb(root_init, R, surfaces, slack_scale):
    
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
    phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(left_foot_constraints,right_foot_constraints,index = i, rotation = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(right_foot_in_lf_frame_constraints,left_foot_in_rf_frame_constraints,index = i, rotation = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i], "slack_scale" : slack_scale } for i in range(nphases)]
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
    
    fileName = "data/new/"+tp.pbName+"___"
    fileName_ps = "data/new/"+tp.pbName+"_ps___" #phase and stepsizes
    
    CONTINUOUS = False
    if CONTINUOUS:
        fileName += "_c"
        fileName_ps += "_c"
    
    
    data = readFromFile(fileName)
    data_ps = readFromFile(fileName_ps)
    
    if data != None:
        success_MI = data[0]
        success_l1_int = data[1]
        phase_l1_int = data[2]
        step_size_l1_int = data[3]
        success_l1 = data[4]
        phase_l1 = data[5]
        step_size_l1 = data[6]
    else :    
        success_MI = success_l1_int = success_l1 = 0
        phase_l1_int = phase_l1 = 0
        step_size_l1_int = step_size_l1 = 0
        
        
    if data_ps != None:
        s = data_ps[0]
        s_i = data_ps[1]
        s_f = data_ps[2]
        s_if = data_ps[3]
        sl = data_ps[4]
        sl_i = data_ps[5]
        sl_f = data_ps[6]
        sl_if = data_ps[7]
    else:
        s = []
        s_i = []
        s_f = []
        s_if = []
        sl = []
        sl_i = []
        sl_f = [[],[]]
        sl_if = [[],[]]
        
    
    run = 0
    sl_if_phase = sl_if[0]; sl_if_case = sl_if[1]
    sl_f_phase = sl_f[0]; sl_f_case = sl_f[1]
        
    all_surfaces = getAllSurfaces(tp.afftool) # only needed for plotting
    surfaces_dict = getAllSurfacesDict(tp.afftool)   

    from sl1m.fix_sparsity import solveL1, solveMIP#, solveL1_MIP
    from random import *
    
    while run < 10 :   
                
        print "############### run : ", run+1
        
        # set the random step size
        step_size = uniform(0.7,1.3)
        slack_scale = float(randrange(700,1300,10))
        print "step size: ", step_size
        print "slack scale: ", slack_scale
        
        ### run MIP
        if CONTINUOUS:
            R, surfaces = getSurfacesFromPathContinuous(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, False)
            # draw_scene(surfaces)
        else:
            configs = getConfigsFromPath (tp.ps, tp.pathId, step_size) # need for non-continuous function
            R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        pb_MI = gen_pb(tp.q_init, R, surfaces, slack_scale)
        pb, coms, res = solveMIP(pb_MI, surfaces, True, draw_scene, False)
    
        if pb is not None : 
            print "### MIP successful"
            success_MI += 1
        else : 
            print "### MIP fail"
            continue
        # print "MIP DONE"
        
        run += 1
        
        
        ### run SL1M with intersection
        if CONTINUOUS:
            R, surfaces = getSurfacesFromPathContinuous(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, True)
        else:
            R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, True)
        pb = gen_pb(tp.q_init, R, surfaces, slack_scale); phase = len(pb["phaseData"])
        pb_, coms, res_ = solveL1(pb, surfaces, draw_scene, False)
    
        if type(pb_) is int : 
            print "### L1 with intersection fail"
            s_if += [step_size]            
            sl_if_phase += [slack_scale]
            sl_if_case += [pb_]
            # continue
        else : 
            print "### L1 with intersection successful"
            success_l1_int += 1
            phase_l1_int += phase
            step_size_l1_int += step_size
            s_i += [step_size]
            sl_i += [slack_scale]
        # print "L1 with intersection DONE"

        
        ### run SL1M 
        if CONTINUOUS:
            R, surfaces = getSurfacesFromPathContinuous(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, False)
        else:
            # draw_scene(surfaces)p_f = [p_f_phase,p_f_case]
            R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        pb = gen_pb(tp.q_init, R, surfaces, slack_scale); phase = len(pb["phaseData"])
        pb_, coms, res_ = solveL1(pb, surfaces, draw_scene, False)
        
        if type(pb_) is int : 
            print "### L1 fail"
            s_f += [step_size]
            sl_f_phase += [slack_scale]
            sl_f_case += [pb_]
            continue
        else : 
            print "### L1 successful"
            success_l1 += 1
            phase_l1 += phase
            step_size_l1 += step_size
            s += [step_size]
            sl += [slack_scale]
        # print "L1 DONE"
        
        
    sl_f = [sl_f_phase, sl_f_case]
    sl_if = [sl_if_phase, sl_if_case]
    data = [success_MI, success_l1_int, phase_l1_int, step_size_l1_int, success_l1, phase_l1, step_size_l1]
    data_ps = [s, s_i, s_f, s_if, sl, sl_i, sl_f, sl_if]
    

    with open(fileName,'wb') as f:
        pickle.dump(data,f)
        
    with open(fileName_ps,'wb') as f:
        pickle.dump(data_ps,f)

                
