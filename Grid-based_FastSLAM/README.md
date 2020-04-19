# Grid-based FastSLAM

In this section, the principles of both localization and mapping will be used to solve one of the most fundamental problems in robotics - SLAM. The full SLAM problem is described below. The measurements z and the controls u are used to solve for the map m and the poses x.

![SLAM Diagraam](Images/slam.png "SLAM Diagram")

Before going into SLAM algorithms, it should be first noted two types of SLAM problems that SLAM algorithms are used to solve: __Full SLAM__ and __Online SLAM__.

![Full vs Online SLAM](Images/full_vs_online_slam.png "Full vs Online SLAM Diagram")

__Online SLAM__ uses measurements and controls to estimate the map and pose that occur only at time t.

__Full SLAM__ uses measurements and controls to estimate the map and series of poses (or trajectory) throughout the robot travel time.



## Grid-based FastSLAM Algorithm



The Grid-based FastSLAM Algorithm shown above follows the following basic steps:

1. 