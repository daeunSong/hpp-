print "Plan guide trajectory ..."
import lp_urdfs_path as tp # change here to try different demo
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
    
    fileName = "data/comptime_rand/"+tp.pbName
    
    run = 0
    step_size = 1.0
        
    all_surfaces = getAllSurfaces(tp.afftool) # only needed for plotting
    surfaces_dict = getAllSurfacesDict(tp.afftool)   
    
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
    
    

    from sl1m.fix_sparsity import solveL1, solveMIP#, solveL1_MIP
    from random import *
    
    while run < 50 :   
                
        print "############### run : ", run+1
        
        ### run MIP
        configs = getConfigsFromPath (tp.ps, tp.pathId, step_size) # need for non-continuous function
        R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        
        pb = gen_pb(tp.q_init, R, surfaces)
        pb, res, time_MI = solveMIP(pb, surfaces, True, draw_scene, False)
    
        if pb is not None : 
            print "### MIP successful"
        else : 
            print "### MIP fail"
            continue
        # print "MIP DONE"
        
        ### run SL1M 
        R, surfaces = getSurfacesFromPathContinuous_(tp.rbprmBuilder, tp.ps, surfaces_dict, tp.pathId, tp.v, step_size, False)
        
        pb = gen_pb(tp.q_init, R, surfaces); phase = len(pb["phaseData"])
        pb, res, time_l1 = solveL1(pb, surfaces, draw_scene, False)
        
        if type(pb) is int : 
            print "### L1 fail"
            cnt += 1
            continue
        else : 
            print "### L1 successful"
        # print "L1 DONE"
        
        run += 1
        
        total_candidate = 0
        for surfs in surfaces:
            total_candidate += len(surfs)
        
        phase_num += [phase]
        candidate_num += [float(total_candidate)/float(phase)]
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
        
