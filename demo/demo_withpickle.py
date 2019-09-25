import pickle
from surfaces_from_path import *
from plot_surfaces import draw

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

############# set up rbprmBuilder ###################    

from hpp.corbaserver.rbprm.talos_abstract import Robot
Robot.urdfName += "_large"
packageName = 'hpp_environments'

rbprmBuilder = Robot ()
rbprmBuilder.setJointBounds ("root_joint", [-3.2,1.8,0.19,0.21, 0.95,1.7])
# As this scenario only consider walking, we fix the DOF of the torso :
rbprmBuilder.setJointBounds ('torso_1_joint', [0,0])
rbprmBuilder.setJointBounds ('torso_2_joint', [0.,0.])
vMax = 1.# linear velocity bound for the root
aMax = 2. # linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
rbprmBuilder.setFilter([Robot.rLegId,Robot.lLegId])

rbprmBuilder.setAffordanceFilter(Robot.rLegId, ['Support',])
rbprmBuilder.setAffordanceFilter(Robot.lLegId, ['Support'])
rbprmBuilder.boundSO3([-4.,4.,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)

from hpp.corbaserver.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "multicontact/bauzil_ramp_simplified", "planning", vf)#,reduceSizes=[0.07,0.,0.])
    
############# main ###################    

if __name__ == '__main__':
  
  configs = readFromFile('data/gp_stairs_talos')  
  surfaces_dict = readFromFile('data/surf_stairs')   
   
  R, surfaces = getSurfacesFromPath(rbprmBuilder, configs, surfaces_dict, None, False, False)
  draw(surfaces)
