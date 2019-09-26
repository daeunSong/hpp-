print "Plan guide trajectory ..."
import lp_stairs_path as tp # still needs to be done for rbprmBuilder parameter ...
print "Guide planned."

from surfaces_from_path import *
from plot_surfaces import draw

import pickle

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
  
  configs = readFromFile('data/gp_stairs_talos')  
  surfaces_dict = readFromFile('data/surf_stairs')   
    
  R, surfaces = getSurfacesFromPath(tp.rbprmBuilder, configs, surfaces_dict, tp.v, True, False)
  draw(surfaces) # plot the result

