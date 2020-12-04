clc; clear all; close all;

path_to_urdf = 'planar_manip.urdf';

robot = Pendubot(path_to_urdf);
% robot.get_base_parameters();
% robot.generate_rigid_body_regressor_function();
% pi_base = robot.get_symbolic_base_parameters()

% Start tests
test_regressor(path_to_urdf);
test_base_regressor(path_to_urdf);



function test_regressor(path_to_urdf)
    tol = 1e-8;
    no_iter = 100;

    % Seed the random number generator based on the current time
    rng('shuffle');
        
    % Create a pendubot instance
    robot = Pendubot(path_to_urdf);
    
    % Create a robot instance using Matlab Toolbox
    rbt = importrobot(path_to_urdf);
    rbt.DataFormat = 'column';
    rbt.Gravity = [0 0 -9.81];
    
    for i = 1:no_iter
        q = -pi + 2*pi*rand(2,1);
        qd = -2*pi + 4*pi*rand(2,1);
        q2d = -4*pi + 8*pi*rand(2,1);

        Ylgr = robot.get_rigid_body_regressor(q, qd, q2d, 'standard');
        tau_regressor = Ylgr*robot.dynamic_parameters_from_urdf('standard');
        
        tau_matlab = inverseDynamics(rbt, q, qd, q2d);
        
        err = norm(tau_matlab - tau_regressor);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Standard regressor test has been passed successfully!\n');
end


function test_base_regressor(path_to_urdf)
    tol = 1e-8;
    no_iter = 100;
    
    % Seed the random number generator based on the current time
    rng('shuffle');
    
    % Create a pendubot instance
    robot = Pendubot(path_to_urdf);
    
    % Create a robot instance using Matlab Toolbox
    rbt = importrobot(path_to_urdf);
    rbt.DataFormat = 'column';
    rbt.Gravity = [0 0 -9.81];
    
    for i = 1:no_iter
        q = -pi + 2*pi*rand(2,1);
        qd = -2*pi + 4*pi*rand(2,1);
        q2d = -4*pi + 8*pi*rand(2,1);

        Ylgr = robot.get_rigid_body_regressor(q, qd, q2d, 'base');
        tau_regressor = Ylgr*robot.dynamic_parameters_from_urdf('base');
        
        tau_matlab = inverseDynamics(rbt, q, qd, q2d);
        
        err = norm(tau_matlab - tau_regressor);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Base regressor test has been passed successfully!\n');
end