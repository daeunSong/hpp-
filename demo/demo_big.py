print "Plan guide trajectory ..."
import lp_urdfs_path as tp # change here to try different demo
# import lp_complex_path as tp # change here to try different demo
print "Guide planned."

from sl1m.rbprm.surfaces_from_path import *
from sl1m.constants_and_tools import *
from sl1m.problem_definition import *


#from sl1m.planner import *
#from sl1m.tools.plot_plytopes import *
from sl1m.planner_scenarios.talos.constraints import *

from numpy import array, asmatrix, matrix, zeros, ones

def gen_pb(init, c0, R, surfaces):
    
    # nphases = len(surfaces)
    # lf_0 = array(root_init[0:3]) + array([0, 0.085,-0.98]) # values for talos ! 
    # rf_0 = array(root_init[0:3]) + array([0,-0.085,-0.98]) # values for talos ! 
    # p0 = [lf_0,rf_0];
    kinematicConstraints = genKinematicConstraints(left_foot_constraints, right_foot_constraints)
    relativeConstraints = genFootRelativeConstraints(right_foot_in_lf_frame_constraints, left_foot_in_rf_frame_constraints)
    nphases = len(surfaces)
    # p0 = [array([0.,0., 0.]), array([0.,0., 0.])] 
    # p0 = None
    
    res = { "p0" : init, "c0" : c0, "nphases": nphases}
    
    #TODO in non planar cases, K must be rotated
    phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [copyKin(kinematicConstraints) for _ in range(len(surfaces[i]))], "relativeK" : [relativeConstraints[(i)%2] for _ in range(len(surfaces[i]))], "S" : surfaces[i] } for i in range(nphases)]
    # phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(left_foot_constraints,right_foot_constraints,index = i, rotation = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(right_foot_in_lf_frame_constraints,left_foot_in_rf_frame_constraints,index = i, rotation = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i] } for i in range(nphases)]
    res ["phaseData"] = phaseData
    return res 
def footPosFromCOM(init_com):
    lf_0 = array(init_com[0:3]) + array([0, 0.085,-0.98])
    rf_0 = array(init_com[0:3]) + array([0,-0.085,-0.98])
    return [lf_0,rf_0]

# def gen_pb(init, s_p0, goal, R, surfaces):
    
#     nphases = len(surfaces)
    
#     res = { "p0" : init, "c0" : s_p0, "goal" : goal, "nphases": nphases}
#     #res = { "p0" : None, "c0" : None, "goal" : None, "nphases": nphases}
    
#     #TODO in non planar cases, K must be rotated
#     #phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [copyKin(kinematicConstraints) for _ in range(len(surfaces[i]))], "relativeK" : [relativeConstraints[(i)%2] for _ in range(len(surfaces[i]))], "S" : surfaces[i] } for i in range(nphases)]
#     phaseData = [ {"moving" : i%2, "fixed" : (i+1) % 2 , "K" : [genKinematicConstraints(left_foot_constraints,right_foot_constraints,index = i, rotation = R, min_height = 0.3) for _ in range(len(surfaces[i]))], "relativeK" : [genFootRelativeConstraints(right_foot_in_lf_frame_constraints,left_foot_in_rf_frame_constraints,index = i, rotation = R)[(i) % 2] for _ in range(len(surfaces[i]))], "rootOrientation" : R[i], "S" : surfaces[i] } for i in range(nphases)]
#     res ["phaseData"] = phaseData
#     return res 

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
    ax.plot(cx, cy, cz, color = 'black')

def plotSurface (points, ax, plt,color_id = -1):
    xs = np.append(points[0,:] ,points[0,0] ).tolist()
    ys = np.append(points[1,:] ,points[1,0] ).tolist()
    zs = np.append(points[2,:] ,points[2,0] ).tolist()# - np.ones(len(xs))*0.005*color_id).tolist()
    colors = ['r','g','b','m','y','c']
    if color_id == -1: ax.plot(xs,ys,zs)
    else: ax.plot(xs,ys,zs, color = 'black')#,colors[color_id])
        
def draw_scene(surfaces, ax = None):
    colors = ['r','g','b','m','y','c']
    color_id = 0
    if ax is None:        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    # [draw_rectangle(l,ax) for l in all_surfaces]
    for i, surfaces_phase in enumerate(surfaces): 
      for surface in surfaces_phase:
        plotSurface(surface, ax, plt,i)#color_id)
      # color_id += 1
      # if color_id >= len(colors):
      #   color_id = 0
        
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
    
    fileName = "data/comptime/tmp/cpp/complex"#tp.pbName
    
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
        mip_comp_gr = data[2]
        mip_wslack_comp_gr = data[3]
        sl1m_comp_gr = data[4]
        sl1m_i_comp_gr = data[5]
        iterations = data[6]
    else :    
        phase_num = []
        candidate_num = []
        mip_comp_gr = []
        sl1m_comp_gr = []
        mip_wslack_comp_gr = []
        sl1m_comp_gr = []
        sl1m_i_comp_gr = []
        iterations = []


    from sl1m.fix_sparsity import solveL1,solveL1_re,solveL1_cost,solveL1_cost_re,solveMIP,solveMIP_cost,solveMIP_fix
    from random import *

    PLOT = True
    CPP = True
    fail = 0
    
    while run < 1 and fail < 3 :   
                
        print "############### run : ", run+1

        configs = getConfigsFromPath (tp.ps, tp.pathId, step_size)
        s_p0 = configs[0][0:3]; init = footPosFromCOM(s_p0)
        g_p0 = configs[-1][0:3]; goal = footPosFromCOM(g_p0)
        
        ### run MIP GUROBI
        configs = getConfigsFromPath (tp.ps, tp.pathId, step_size) # need for non-continuous function
        R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        # R, surfaces = getSurfacesFromPath_mpc(tp.rbprmBuilder, configs, surfaces_dict, 10, tp.v, False)
        # R[0]=[R[0][0]]; R[-1]=[R[-1][-1]]; surfaces[0]=[surfaces[0][0]]; surfaces[-1]=[surfaces[-1][-1]]
        # pb = gen_pb(init, s_p0, R, surfaces); phase = len(pb["phaseData"])
        pb = gen_pb(None, None, R, surfaces); phase = len(pb["phaseData"])
        pb, res, time_MI_gr = solveMIP(pb, surfaces, True, draw_scene, PLOT, CPP, False)
        print time_MI_gr
        
        if type(pb) is int : 
            print "### MIP fail"
            fail += 1
            continue
        else : 
            print "### MIP successful"
            
        #### run MIP GUROBI wslack
        # R, surfaces = getSurfacesFromPath_mpc(tp.rbprmBuilder, configs, surfaces_dict, 10, tp.v, False)
        # R[0]=[R[0][0]]; R[-1]=[R[-1][-1]]; surfaces[0]=[surfaces[0][0]]; surfaces[-1]=[surfaces[-1][-1]]
        R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        # pb = gen_pb(init, s_p0, R, surfaces)
        pb = gen_pb(None, None, R, surfaces)
        pb, res, time_MIP_wslack_gr = solveMIP(pb, surfaces, True, draw_scene, PLOT, CPP, True)
        print time_MIP_wslack_gr
    
        if type(pb) is int : 
            print "### MIP WITH SLACK fail"
            fail += 1
            continue
        else : 
            print "### MIP WITH SLACK successful"

        #### run SL1M GUROBI 
        R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        # pb = gen_pb(init, s_p0, R, surfaces)
        pb = gen_pb(None, None, R, surfaces)
        pb, res, time_MIP_wslack_gr = solveMIP_fix(pb, surfaces, True, draw_scene, PLOT, CPP, False)
        print time_MIP_wslack_gr
    
        if type(pb) is int : 
            print "### FIXED MIP fail"
            fail += 1
            continue
        else : 
            print "### MIP successful"
        
        # #### run SL1M with iterative reweighting GUROBI 
        R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, False, False)
        # pb = gen_pb(init, s_p0, R, surfaces)
        pb = gen_pb(None, None, R, surfaces)
        pb, res, time_MIP_wslack_gr = solveMIP_fix(pb, surfaces, True, draw_scene, PLOT, CPP, True)
        print time_MIP_wslack_gr
    
        if type(pb) is int : 
            print "### FIXED MIP WITH SLACK fail"
            fail += 1
            continue
        else : 
            print "### FIXED MIP WITH SLACK successful"
        
        run += 1
        
        total_candidate = 0
        for surfs in surfaces:
            total_candidate += len(surfs)
        
        # phase_num += [phase]
        # candidate_num += [float(total_candidate)/float(phase)]
        
    #     mip_comp_gr += [time_MI_gr]
    #     mip_wslack_comp_gr += [time_MIP_wslack_gr]
    #     sl1m_comp_gr += [time_l1_gr]    
    #     sl1m_i_comp_gr += [time_L1_i_gr]    
    #     iterations += [iteration]
            
    # data = [phase_num, candidate_num, mip_comp_gr, mip_wslack_comp_gr, sl1m_comp_gr, sl1m_i_comp_gr, iterations]
    

    # with open(fileName,'wb') as f:
    #     pickle.dump(data,f)
        

"""
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

data = readFromFile("data/comptime/tmp/cpp/complex")

phase_num = data[0]
candidate_num = data[1]
mip_comp_gr = data[2]
mip_wslack_comp_gr = data[3]
sl1m_comp_gr = data[4]
sl1m_i_comp_gr = data[5]
iterations = data[6]

len(phase_num)
sum(phase_num)/len(phase_num)
sum(candidate_num)/len(phase_num)
sum(mip_comp_gr)/len(phase_num)
sum(mip_wslack_comp_gr)/len(phase_num)
sum(sl1m_comp_gr)/len(phase_num)
sum(sl1m_i_comp_gr)/len(phase_num)
sum(iterations)


"""
