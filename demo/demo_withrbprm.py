print "Plan guide trajectory ..."
import lp_stairs_path as tp # change here to try different demo
print "Guide planned."

from surfaces_from_path import *
from plot_surfaces import draw

  
############# main ###################    

if __name__ == '__main__':
    
  configs = getConfigsFromPath (tp.ps, tp.pathId, 1.3)
  #all_surfaces = getAllSurfaces(tp.afftool) # only needed for plotting
  surfaces_dict = getAllSurfacesDict(tp.afftool) 
    
  R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, False)
  #draw(surfaces,all_surfaces)
  draw(surfaces) # plot the result

