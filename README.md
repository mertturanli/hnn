# Hopfield Neural Network (HNN) Estimator Repository
    The repository for the source code and dataset for Hopfield Neural Network Testing

The ROS package includes HNN estimator node and dataset from Gazebo with three robots (the source code of the parameter estimator of the main method given in the paper "Multi-Robot Workspace Allocation with Hopfield Networks and Imprecise Localization").

## Installation
1. Install ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu)

2. Clone the repository 
```
   cd ~/catkin_ws/src
   git clone https://github.com/mertturanli/hnn
   cd ~/catkin_ws
   catkin_make
```
3. Run the estimator on the dataset:
```
    $ roslaunch hnn bag.launch 
```

## Notes
1. The rosbag node starts paused; you can unpause it by hitting space button.

2. The parameter vectors of the robots to be estimated are [0.9, 0.9], [1.0, 1.0] and [1.0, 1.0], respectively. The trajectories of the robots were created by using these coefficients.

3. The topic /robotX/estErr gives the estimation error (X is the robot number). In the launch file, the convergence of the estimator can be adjusted by using hfAlpha and hfBeta parameters.
The parameter estimation errors can be plotted using the following command:
```
    $ rqt_plot /robot1/estErr /robot2/estErr /robot3/estErr 
```
