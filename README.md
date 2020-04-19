# RoboticsND-Map-My-World

This unit of the course involves the investigation and implementation of various mapping and SLAM algorithms. 

Localization is the process of estimating a robot's pose in an assumed known environment. Mapping assumes you know the robot path and estimates the map of the environment. Both of these concepts form one of the most fundamental conecepts of mobile robotics - SLAM. SLAM involves the process of localizing a robot in a map while a simulatenously creating that map from known positions.

In this unit, we started with exploring the concept of mapping. Mapping was implemented using the Occupancy Grid Mapping Algorithm which allows for mapping of any arbitrary area by diving it into a number of finite grid cells.

After learning about mapping, this knowledge was combined with previously learned localization to implement SLAM.

The are typically 5 categories of SLAM algorithms:

* Extended Kalman Filter SLAM (EKF)
* Sparse Extended Information Filter (SEIF)
* Extended Information Form (EIF)
* FastSLAM
* GraphSLAM

In this unit, we started by using the FastSLAM algorithm. We then modified the FastSLAM algorithm to grid maps to implement the Grid-Based FastSLAM algorithm. Finally, GraphSLAM was also learned and along with one of its implementations Real-Time Appearance-Based Mapping (RTAB-Map) being used in the final project.