<<<<<<< Updated upstream
# Here inertial parameter identification of robotic systems is implemented.
# Identification consistes of several steps:
#   -> finding base parameters 
#   -> trajectory optimization
#   -> running the trajectory on real robot and recording obtained trajectories
#   -> offline data processing
#   -> parameter estimation
=======
# Dynamic calibration (dynamic parameter indentification) for rigid body manipulator
The code was developed in the framework of the human-robot interaction project at [Innopolis University](https://innopolis.university/en/). One of the outputs of the project was a paper -- [Practical Aspects of Model-Based Collision Detection](https://www.frontiersin.org/articles/10.3389/frobt.2020.571574/full), where we provide some review of the recent developments in the field of dynamic calibration, outline the steps required for dynamic parameter identification and provide many useful references. If you have questions from theoretical perspective, please check the paper first. If find paper and code useful, consider citing it in your own papers.

The parameter identification prcedure can be divided into several steps:
## Finding regressor
Right now, there are two ways to find regressor matrix:
1. using screw algorithm (function *screw_regressor.m* or *screw_regressor2.m*)
2. using generated matrix function. The expression for the regressor is obtained symbolically using Euler-Lagrage equations, and then the function is generated from symbolic expression (script *ur_regressors_lgr.m*)
Keep in mind that two methods output different regressors, the reason for that is the way you structure the vector of standard parameters!
>>>>>>> Stashed changes
