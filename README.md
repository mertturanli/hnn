# hnn
The code and dataset for Hopfield Neural Network Testing

The ROS package includes HNN estimator node and dataset from Gazebo with three robots (the source code of the parameter estimator of the main method given in the paper "Multi-Robot Workspace Allocation with Hopfield Networks and Imprecise Localization").

In order to execute the estimator on the dataset:

$ roslaunch hnn bag.launch 

The parameter vectors of the robots to be estimated are [0.9, 0.9], [1.0, 1.0] and [1.0, 1.0], respectively.

The topic /robotX/estErr gives the estimation error (X is the robot number). In the launch file, the convergence of the estimator can be adjusted by using hfAlpha and hfBeta parameters.

The parameter estimation errors can be plotted using the following command:

$ rqt_plot /robot1/estErr /robot2/estErr /robot3/estErr 

