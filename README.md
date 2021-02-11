# lightsfm

A lightweight implementation of the Social Force Model for Social Local Navigation. 
It is based on the model proposed by Helbing and Molnar [1] and extended for social groups by Moussaid et Al. [2]:

- **[1]** Helbing, Dirk & Molnar, Peter. (1998). *Social Force Model for Pedestrian Dynamics*. Physical Review E. 51. 10.1103/PhysRevE.51.4282. 
- **[2]** Moussaïd, Mehdi & Perozo, Niriaska & Garnier, Simon & Helbing, Dirk & Theraulaz, Guy. (2010). *The Walking Behaviour of Pedestrian Social Groups and Its Impact on Crowd Dynamics*. PloS one. 5. e10047. 10.1371/journal.pone.0010047. 

The model consists on the definition of different attractive and repulsive forces that describe the local navigation behavior of pedestrians. 

<img src="https://render.githubusercontent.com/render/math?math=F_{total} = f_{goal} %2B f_{obs} %2B f_{ped} %2B f_{group}">

## Social Forces
### 1. Attractive force to reach the goal (DesiredForce f<sub>goal</sub>)

<img src="https://render.githubusercontent.com/render/math?math=f_{goal} = \alpha_{g} \frac{1}{\gamma} (V_{desired} - V_{actual})">

Parameters:

- <img src="https://render.githubusercontent.com/render/math?math=\alpha_{g}+"> Strength Factor of the desire to reach the goal.
- <img src="https://render.githubusercontent.com/render/math?math=\gamma+"> 

### 2. Repulsive force of obstacles (ObstacleForce f<sub>obs</sub>)


### 3. Respulsive force of other pedestrians (SocialForce f<sub>ped</sub>)


### 4. Force of interaction groups (GroupForce f<sub>group</sub>)

This force is a combination of another subforces that keeps the formation of the social group.

*f<sub>group</sub> = f<sub>ggaze</sub> + f<sub>gcoh</sub> + f<sub>grep</sub>*


...


## Code indications

- *sfm.hpp* contains the computation of all the forces.
- *cmd_vel.hpp* make use of the forces in order to compute a velocity command to be sent to the robot. 
- *angle.hpp and vector2d.hpp* contain different structures and methods that are employed in the forces computation.
- *map.hpp* contains a structure for representing the obstacles of the static map, and some virtual methods to check obstacles.
- *rosmap.hpp* is a class that implements the virtual methods of map.hpp and uses ROS to obtain the static navigation map. It also implement a kd-tree to perform a nearest-neighbor search on the map obstacles.
- *astar.hpp* is an auxiliary class that contains the implementation of a Astar path planning algorithm.

 

