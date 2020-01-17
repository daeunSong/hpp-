
from sl1m.constants_and_tools import *
from sl1m.planner import *

from constraints import *
import pickle


def readFromFile (fileName):
    data = []
    with open(fileName,'rb') as f:
        while True:
            try:
                data.append(pickle.load(f))
            except EOFError:
                break
    return data[0]
    
all_surfaces = []

    
############# main ###################    

if __name__ == '__main__':    
    from sl1m.random_scene_gen import scene_gen, problem_gen_mi, problem_gen_sl1m
    from sl1m.fix_sparsity import solveL1, solveMIP
    
    for i in range(0,run):
    
        # scene generation
        all_surfaces = readFromFile('pb_files/scene_square/1_scene_unsolved')  
        pb_MI = None; step_size = 0.5
        surfaces_mi, p_start, p_goal = problem_gen_mi(all_surfaces, step_size)
        
        x_min = y_min = z_min = 100
        x_max = y_max = z_max = -100
        # find the bound of the scene
        for surface in all_surfaces:
            for point in surface:
                if x_min > point[0]:
                    x_min = point[0]
                if y_min > point[1]:
                    y_min = point[1]
                if z_min > point[2]:
                    z_min = point[2]
                if x_max < point[0]:
                    x_max = point[0]
                if y_max < point[1]:
                    y_max = point[1]
                if z_max < point[2]:
                    z_max = point[2]

        
        f.write("%f %f %f\n" %(p_start[0], p_start[1], p_start[2]))
        f.write("%f %f %f\n" %(p_goal[0], p_goal[1], p_goal[2]))
        f.write("%f %f\n" %(x_min, x_max))
        f.write("%f %f\n" %(y_min, y_max))
        f.write("%f %f\n" %(z_min, z_max))
