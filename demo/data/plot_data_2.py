import pandas as pd
import matplotlib.pyplot as plt
import pickle
from numpy import array

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
    
    # fileName = "stepsize/stairs_1_ps"
    fileName = "replanning_gurobi/all_in_one_rubbles"
    data = readFromFile(fileName)
    
    fig, ax = plt.subplots()    
    
    phase_num = data[0]
    candidate_num = data[1]
    mip_comp = data[2]
    n = data[3]
    n = data[4]
    sl1m_comp = data[5]
    n = data[6]
    n = data[7] 
    
    c = list(array(mip_comp)/array(sl1m_comp))

    ax.grid()
    plt.title('Replanning-Stairs')
    plt.xlabel('phase #')
    plt.ylabel('candidate # per phase')
    # plt.ylabel('slack scale')   
    
    index = []
    max_c = -1
    for i in range(0, len(phase_num)):
        if c[i]<2.0:# 200:
            index.append(i)
        else:
            if max_c < c[i]:
                max_c = c[i]
    
    cnt = 0
    for cc in c :
        if cc < 1 :
           cnt +=1 
            
    n = 0
    for i in index:
        del c[i-n]
        del phase_num[i-n]
        del candidate_num[i-n]
        n += 1
        
    
    
    #plot
    plt.scatter( phase_num, candidate_num, edgecolors='none', c = c,#sl1m_comp, 
                cmap = 'seismic', s = 100, alpha =0.5)
    cbar = plt.colorbar()
    #cbar.set_label('Color Intensity')
    
    plt.clim(0,3.15)
    plt.xlim(2,8)
    plt.ylim(1.2,2.6)
    #plt.ylim(0.5,5.0)

    plt.ion()
    plt.show()
