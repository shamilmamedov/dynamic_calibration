# Dynamic calibration (dynamic parameter estimation) for rigid body manipulator
The code was developed in the framework of the human-robot interaction project at [Innopolis University](https://innopolis.university/en/). One of the outputs of the project was a paper -- [Practical Aspects of Model-Based Collision Detection](https://www.frontiersin.org/articles/10.3389/frobt.2020.571574/full), where we provide some review of the recent developments in the field of dynamic calibration, outline the steps required for dynamic parameter identification and provide many useful references. If you have questions from theoretical perspective, please check the paper first. If find paper and code useful, consider citing it in your own papers.


## Required external software
1. [YALMIP](https://yalmip.github.io/) for formulating semidefinite programs
2. [SDPT3](https://blog.nus.edu.sg/mattohkc/softwares/sdpt3/) for solving semidefinite programs


## Getting started
The easiest way to get started is to run *main* script.

## Dynamic parameter estimation in a nutshell
The parameter estimation prcedure can be divided into several steps:
### Finding regressor
Inverse dynamics of the rigid body robots can be written in linear-in-parameters form. Therefore the first step in dynamic parameter estimation is deriving inverse dynamics in a regressor form.

### Finding base parameters
Some standard parameters do not affect the dynamics of the robot (for example, majority of the dynamic parameters of the first link), and some enter the dynamics in linear combination with other parameters. Because of that the regressor matrix is not full rank, and is not invertible - the property we need for least squares estimation. We can find so-called base parameters of the robot, such that corresponding regressor matrix is full rank. 

### Trajectory optimization
To get accurate estimates of the dynamic parameters, we need to make sure that the data we use has the property called persistent excitation. To achieve that it is necessary to perform trajectory optimization, in other words experiment design. As a trajectory I use the combination of truncated Fourier series (*fourier_series_traj.m*) and fifth order polynomial, yielding a function *mixed_traj.m*. The condition number of the  observation matrix (aggregated base regressor functions) is used as an objective function of the optimization problem. Constrains include joint limits, maximum velocties and accelerations. As for optimization algorithms, two are used: patter search (patternsearch) and genetic algorithm (ga). From my own experience, pattern search works better.

The main script for experiment design is *experiment_design.m*.

### Data collection
Run the trajectory on your robot in closed loop. In case of UR10E we set it to velocity control mode and provided velocity referenecs based on optimized trajectory. Depending on the controller of the robot you will get a trajectory similar to the one you found from optimization. During the execution of motion record torques (currents), positions and if available velocities. 

What if drive gains are not available? It is possible to identify them, but for that you need to perform one more experiment with a load of some known inertial parameters (at least one, for example, mass) attached to the end effector of the robot. Try to choose heavier load.

### Data processing
Depending on the quality of your sensors, you may or may not filter positions and velocties. Usually current or torque measurements are noisy, so better to filter them with zero-phase filter, to avoid introducing lag (delay). Acceleration measurements are never available, so you need to estimate them using the finite differences. Use central difference for more accurate estimate, and afterwards filter acceleration estimates using zero-phase filter. All the filtering is done by *filterData.m*
function. Filter order and cutt-off frequecy are hardcoded there, better if you play with those number to obtain the best filter for your own data.

### Parameter estimation
The absence of drive gains complicates the identification procedure. Try to get them, for example, email manufacturer or get a rough estimate from datasheet of the drives. In case you decide to identify them, you need trajectory with an attached load. For estimating drive gains three methods were implemented (see function *estimate_drive_gains*):
1. total least squares - theoretically it should not introduce bias, in practice it results in negative or ridiculously large gains for some joints
2. least square - theoretically introduces bias, in pratice also may result in negative drive gains
3. least square with physical feasibility constraints - theoreticall introduces bias, but in pratice results in adequate estimates

I tried weighted least squares, where weights are variances, but didn't get improvement. In my opinion poor estimates for drive gains in case of (total) least squares can be due to unmodeled elasticity of the joints. To better understand drive gain identification read [this](https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/136/5/051025/370830/Global-Identification-of-Joint-Drive-Gains-and?redirectedFrom=fulltext) paper.

Once you have drive gains or torque measurements directly, start with ordinary least squares (*ordinaryLeastSquareEstimation* function in the script *ur_idntfcn_real.m*). More advanced approach is to use least squares with physical feasibility constraints which is posed as semidefinite program (SDP). In the script following notation from [this](https://arxiv.org/pdf/1701.04395.pdf) paper you can choose semi-physical or physical consistency. If you use SDP you can retriev standard parameters from base parameters using bijective mapping that was introduced [here](https://www.researchgate.net/profile/Cristovao-Sousa-3/publication/330251436_Inertia_Tensor_Properties_in_Robot_Dynamics_Identification_A_Linear_Matrix_Inequality_Approach/links/5e1c5c5d4585159aa4cbb5b1/Inertia-Tensor-Properties-in-Robot-Dynamics-Identification-A-Linear-Matrix-Inequality-Approach.pdf). 

### Validation 
On your robot, follow various trajectories in cloed loop and log the data. Then try to predict measured torques (currents) and compare with measurements. If the prediction is not satisfactory get back to trajectory optimization stage and continue from there.

