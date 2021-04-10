# Dynamic calibration (dynamic parameter indentification) for rigid body manipulator
The code was developed in the framework of the human-robot interaction project at [Innopolis University](https://innopolis.university/en/). One of the outputs of the project was a paper -- [Practical Aspects of Model-Based Collision Detection](https://www.frontiersin.org/articles/10.3389/frobt.2020.571574/full), where we provide some review of the recent developments in the field of dynamic calibration, outline the steps required for dynamic parameter identification and provide many useful references. If you have questions from theoretical perspective, please check the paper first. If find paper and code useful, consider citing it in your own papers.

The parameter identification prcedure can be divided into several steps:
## Finding regressor
Right now, there are two ways to find regressor matrix:
1. using screw algorithm: functions *screw_regressor.m* or *screw_regressor2.m*)
2. using generated matrix function. The expression for the regressor is obtained symbolically using Euler-Lagrage equations, and then the function is generated from symbolic expression: script *ur_regressors_lgr.m*.

Keep in mind that two methods output different regressors, the reason for that is the way you structure the vector of standard parameters!

## Finding base parameters
Some standard parameters do not affect the dynamics of the robot (for example, majority of the dynamic parameters of the first link), and some enter the dynamics in linear combination with other parameters. Because of that the regressor matrix is not full rank, and is not invertible - the property we need for least squares estimation. We can find so-called base parameters of the robot, such that corresponding regressor matrix is full rank. You can find two methods for finding base parameters:
1. numerical method that uses QR decomposition: scripts *ur_base_params_QRlgr.m* and *ur_base_params_QRscrw.m*. Because I was mostly using *ur_base_params_QRlgr.m*  it is better documented.
2. symbolic method: function *khalil_gautier.m*)

I encourage you to use numerical method, as it is easy to use and understand. Moreover, when symbolic method works only with inertial parameters of the link.

## Trajectory optimization
To get accurate estimates of the dynamic parameters, we need to make sure that the data we use has the property called persistent excitation. To achieve that it is necessary to perform trajectory optimization, in other words experiment design. As a trajectory I use the combination of truncated Fourier series (*fourier_series_traj.m*) and fifth order polynomial, yielding a function *mixed_traj.m*. As an objective function of the optimization problem, the condition number of the  observation matrix (aggregated base regressor functions) is used (see *traj_cost_lgr.m* or *traj_cost_scrw.m*). 
