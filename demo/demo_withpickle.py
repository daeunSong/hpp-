# print "Plan guide trajectory ..."
# import lp_stairs_path as tp # change here to try different demo
# print "Guide planned."

# from surfaces_from_path import *
# from plot_surfaces import draw

import pickle


import matplotlib.pylab as plt
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

def readFromFile (fileName):
  data = []
  with open(fileName,'rb') as f:
    while True:
      try:
        line = pickle.load(f)
      except EOFError:
        break
      data.append(line)  
  return data[0]
  
############# main ###################    

if __name__ == '__main__':
  
  # configs = readFromFile('data/gp_stairs_talos')  
  # surfaces_dict = readFromFile('data/surf_stairs')   
    
  # R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, False)
  # draw(surfaces) # plot the result

    from sl1m.fix_sparsity import solveL1
    pb = readFromFile("pb")
    surfaces = readFromFile("surfaces")
    
    pb, res, time_l1 = solveL1(pb, surfaces, draw_scene)
