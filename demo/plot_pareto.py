import pickle
import matplotlib.pyplot as plt
from numpy import array
import numpy as np

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
  
  
data = readFromFile("pareto_bridge_1_")
data = array(data).T
slack = data[0]; #slack = np.delete(slack,0); slack = np.delete(slack,0)
dist = data[1]; #dist = np.delete(dist,0); dist = np.delete(dist,0)
weight = data[2]; #weight = np.delete(weight,0); weight = np.delete(weight,0)

curr = slack[0]
for i in range(len(slack)):
    diff = curr - slack[i]
    if diff > 2.:
        print weight[i]
    curr = slack[i]

fig, ax = plt.subplots()    
ax.scatter(slack, dist, s=100, color='b', alpha=0.5)

plt.title("BRIDGE - 01")
plt.xlabel("sum slack vars")
plt.ylabel("distance")

plt.ion()
plt.show()
