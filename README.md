# Contact Surfaces Generation from the Guide-path
## Description
This module generates a set of potential contact surfaces for the effector and the root orientation from the guide-path planned by [hpp-rbprm](https://github.com/humanoid-path-planner/hpp-rbprm)

## Dependencies
This module depends on several packages from HPP. 
1. Install HPP - follow the instructions: https://humanoid-path-planner.github.io/hpp-doc/download.html (select 'Developement' in the list)
2. Once the installation is complete, run `make rbprm`

## Inputs & Outputs of the Algorithm
### Inputs:
- 6D trajectory of the root (***configs***)
- all the surfaces in the dictionary format with its name and the surface normal (***surface_dict***)

The inputs could be directly given by running the guide-path planner or by pickle.

### Outputs:
- a set of potential contact surfaces for the effector (***surfaces***)
- a set of root orientation matrices (***R***)

The contact surface could be either the whole surface or the intersection between the surface and the rom of the effector.
This is automatically decided by the algorithm, unless the user wants the whole surface only (***useInterface = False***).


## Demo
The demo is available with Talos robot model. The Demo also visualize you the set of potential contact surfaces.
1. Instal talos-rbprm: https://github.com/humanoid-path-planner/talos-rbprm.git
2. Run the demo code
`./run demo_withrbprm.py`
