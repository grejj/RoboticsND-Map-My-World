# Grid-based FastSLAM

![](Videos/gmapping.gif=764x429)

In this section, the principles of both localization and mapping will be used to solve one of the most fundamental problems in robotics - SLAM. Before going into SLAM algorithms, it should be first noted two types of SLAM problems that SLAM algorithms are used to solve:

1. __Online SLAM__ uses current measurements and controls to estimate the current map and pose.

2. __Full SLAM__ uses all measurements and controls to estimate the map and series of poses (or trajectory) throughout the robot travel time.

## FastSLAM

FastSLAM is a SLAM algorithm that applies the particle filter approach (like MCL) to solve the SLAM problem. However, this approach differs from MCL as an additional dimension, the map, is added to each particle. If we used the same approach as MCL, the problem would scale out of control because the map dimesion has many variables. Thus, a modified approach must be taken.

The modification is called the __Rao-Blackwellized__ particle filter approach. This particle filter approach estimates the posterior (map and pose) using a particle filter and Gaussian. With FastSLAM, the particle filter approach is used to get the robot trajectory or positions and then the mapping is reduced to mapping with known poses. But this problem assumes there are always landmarks. What if there are none?

# Grid-based FastSLAM Algorithm

Grid-based FastSLAM is a modfication of FastSLAM that solves the SLAM problem without landmarks by using grid maps. To solve for the posterior (map and pose), the problem is divided into separate parts:

1. __Estimate Robot Trajectory__

Just as with Monte-Carlo Localization, robot pose is estimated by a set of particles that each have a pose and weight associated with them that defines how accurate that particle is to the actual robot pose. However, in addition to this, each particle also maintains its own map.

2. __Estimate Map__

As each particle has a defined pose, the problem is reduced to solving the map with known poses (Occupancy-Grip Mapping) for each particle. For each particle, the corresponding pose is used to solve the map for that specific particle.

This process is summarized below, using MCL for estimating robot trajectory/pose at each particle and Occupancy-Grid Mapping for estimating the map at each particle using their corresponding pose:

![Grid-based FastSLAM](Images/grid-based_fastslam.png=764x429 "Grid-based FastSLAM")

The Grid-based FastSLAM algorithm can be further summarized as below:

![Grid-based FastSLAM Algorithm](Images/grid-based_fastslam_algorithm.png=764x429 "Grid-based FastSLAM Algorithm")

The Grid-based FastSLAM algorithm takes input as previous belief of map and pose, current command, and current measurement and outputs the current belief of map pose using the following steps for every particle (green box):

1. __Sampling Motion__ - current pose is determined by current command and previous poses.
2. __Importance Weight__ - the weight of the particle is updated.
3. __Map Estimation__ - occupancy grid mapping is used to update particle map.

At the end during resampling (red box), the particles with the lowest weight are discarded.

## IMPLEMENTATION - SLAM_GMAPPING

To test out what has been learned about Grid-based FastSLAM, the ROS __gmapping__ package will be used with Turtlebot3 to produce a map using the Gazebo simulator. Gmapping is fed laser data as well as robot odometry to produce a 2D occupancy grid map.

![GMapping](Images/gmapping.png=764x429 "GMapping")

Make sure that you have install TurtleBot3 and slam gmapping packages:

```
$ sudo apt-get install ros-melodic-turtlebot3-*
$ sudo apt-get install ros-melodic-slam-gmapping
```

Set a Turtlebot3 model in your bashrc. Example:

```
export TURTLEBOT3_MODEL=waffle_pi
```

Launch Turtlebot3 in Gazebo:

```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch gmapping SLAM:

```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

You should now be able to view the generating map in RViz. To move the TurtleBot3 around and fill out the rest of the map:

```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

To save an image of the current map:

```
$ rosrun map_server map_saver -f ~/map
```