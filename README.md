# occupancy_grid_mapping
This is a pseudo C++ package for doing mapping based on the occupancy grid 

In robotics, mapping takes place after SLAM. In the SLAM, the robot pose and environment map is generated while in the mapping, the poses are taken filtered and assumed as known.  


![image](https://user-images.githubusercontent.com/17289954/103077040-5154f080-45cf-11eb-8de8-72ec072567e8.png)

## Posterior Probability

Going back to the graphical model of mapping with known poses, the goal is to implement a mapping algorithm and estimate the map given noisy measurements and assuming known poses.

The Mapping with Known Poses problem can be represented with P(m∣z1:t,x1:t) function. With this function, we can compute the posterior over the map given all the measurements up to time t and all the poses up to time t represented by the robot trajectory.

In estimating the map, we’ll exclude the controls u since the robot path is provided to us from SLAM. However, keep in mind that the robot controls will be included later in SLAM to estimate the robot’s trajectory. 

![image](https://user-images.githubusercontent.com/17289954/103077261-c45e6700-45cf-11eb-8f60-c38883264a29.png)

 ### First Approach:P(m∣z1:t,x1:t)

It is obvious that maps have high dimensionality. Consequently, it will be too pricey in terms of computational memory to compute the posterior under this first approach.


### Second Approach:P(mi∣z1:t,x1:t)

A better approach to estimating the posterior map is to decompose this problem into many separate problems. In each of these problems, we will compute the posterior map *mi* at each instant. However, this approach still presents some drawbacks because we are computing the probability of each cell independently. Thus, we still need to find a different approach that addresses the dependencies between neighboring cells.


### Third Approach:∏iP(mi∣z1:t,x1:t)

In the third approach we relate cells and overcome the need for huge computational memory in order to estimate the map with the product of marginals or factorization. 

## Inverse Measurement Model - P(x | z1:t): Estimating a posterior over the system state given the measurement.

The inverse measurement model is generally used when measurements are more complex than the system's state. 

Because in the occupancy grid approach, the cells are either occupied or free (or unknown), we need a binary bias filter:


![image](https://user-images.githubusercontent.com/17289954/103086212-55d7d400-45e4-11eb-86b9-515724e4b971.png)

where the initial belief represents the initial state of the system before taking any sensor measurements into consideration. 

## Occupancy grid mapping algorithm 

![image](https://user-images.githubusercontent.com/17289954/103086317-9a636f80-45e4-11eb-84cb-03d50ca37084.png)

In this pseudo implementation, a robot is equipped with eight sonar rangefinder sensors circulating in an environment to map it. This robot is provided with its exact poses at each timestamp. 

![image](https://user-images.githubusercontent.com/17289954/103086507-1a89d500-45e5-11eb-92cd-1b1fafe24667.png)

the advatage of using the odds log probablity is due its numerical stability around zero and one as it can be seen:

![image](https://user-images.githubusercontent.com/17289954/103086626-65a3e800-45e5-11eb-92ed-7dbb9bddf626.png)

The inverse sensor model takes the following inputs

![image](https://user-images.githubusercontent.com/17289954/103086740-a13eb200-45e5-11eb-8d1b-fc5257a82031.png)

and delivers if the cell is occupied, free, or unknown. Then, based on the pose data:


![image](https://user-images.githubusercontent.com/17289954/103086877-16aa8280-45e6-11eb-84a2-b77866559093.png)

## Result

![image](https://user-images.githubusercontent.com/17289954/103143043-21ad0200-470f-11eb-989f-fd4e44fbbf47.png)

