clc; clear all; close all;


path_to_urdf = 'planar_manip.urdf';


robot = Pendubot(path_to_urdf);

% delete @Pendubot/get_M.m @Pendubot/get_n.m ...
%        @Pendubot/get_C.m @Pendubot/get_g.m

% robot.generate_rigid_body_regressor_function();
% robot.get_base_parameters();
% robot.generate_dynamics_functions();

% Start tests
test_regressor(path_to_urdf);
test_base_regressor(path_to_urdf);
test_frinction_regressor(path_to_urdf);
test_inertia_matrix(path_to_urdf);
test_gravity_torque(path_to_urdf);
test_velocity_product(path_to_urdf);
test_identification(path_to_urdf);


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
        tau_regressor = Ylgr*robot.get_dynamic_parameters_from_urdf('standard');
        
        tau_matlab = inverseDynamics(rbt, q, qd, q2d);
        
        err = norm(tau_matlab - tau_regressor);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Standard Regressor Test -- OK!\n');
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
        tau_regressor = Ylgr*robot.get_dynamic_parameters_from_urdf('base');
        
        tau_matlab = inverseDynamics(rbt, q, qd, q2d);
        
        err = norm(tau_matlab - tau_regressor);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Base Regressor Test -- OK!\n');
end


function test_frinction_regressor(path_to_urdf)
    no_iter = 100;
    
     % Seed the random number generator based on the current time
    rng('shuffle');

    % Create a pendubot instance
    robot = Pendubot(path_to_urdf);
    
    for i = 1:no_iter
        q_dot = -2*pi + 4*pi*rand(2,1);
        F_v = diag(-5 + 10*rand(2,1));
        F_c = diag(rand(2,1));
        pi_frcn = [F_v(1,1) F_c(1,1) F_v(2,2) F_c(2,2)]';
        
        tau_frcn_1 = F_v*q_dot + F_c*sign(q_dot);
        tau_frcn_2 = robot.get_friction_regressor(q_dot, 'discontinuous')*pi_frcn;
        assert(norm(tau_frcn_1 - tau_frcn_2) == 0);
    end
    fprintf('Friction Regressor Test --OK!\n');
end


function test_inertia_matrix(path_to_urdf)
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

        M = robot.get_M(q);
        
        M_matlab = massMatrix(rbt, q);
        
        err = norm(M_matlab - M);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Inertia Matrix Test -- OK!\n');
end


function test_gravity_torque(path_to_urdf)
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
        
        g = robot.get_g(q);
        
        g_matlab = gravityTorque(rbt, q);
        
        err = norm(g_matlab - g);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Gravity Torque Test -- OK!\n');
end


function test_velocity_product(path_to_urdf)
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

        Cqd = robot.get_C(q, qd)*qd;
        
        Cqd_matlab = velocityProduct(rbt, q, qd);
        
        err = norm(Cqd_matlab - Cqd);
        assert(err < tol, 'Regressor matrix is not computed correctly. Check derivation');
    end
    fprintf('Velocity Product Test -- OK!\n');
end


function test_identification(path_to_urdf)
    % perform identification based on simulated data
    
    robot = Pendubot(path_to_urdf);

    % specify a trajectory to follow by the first joint and its parameters
    q_ref = @(t, q0, a) q0 + a(1)*sin(t) + a(2)*sin(2*t) + a(3)*sin(3*t);

    q0 = -pi/2;
    a = [1, 1, 0.75];

    % specify controller paramters
    kp = 10;
    kd = 1;

    % simultion for identification
    t = 0:1e-2:5;
    x0 = [-pi/2, 0, 0, 0]';
    x_sim = x0;
    tau = [];
    for i = 1:length(t)-1
        % compute control signal
        u = kp*(q_ref(t(i), q0, a) - x_sim(1,i)) - kd*x_sim(3,i);
        tau = vertcat(tau, u);

        % simulate
        [~, x] = ode45(@(t,x) robot.ode(x,u), [t(i) t(i+1)], x_sim(:,i));
        x_sim = horzcat(x_sim, x(end,:)');
    end

    q = x_sim(1:2,:);
    q_dot = x_sim(3:4,:);
    
    % animate obtained motion
%     robot.animate_motion(q(:, 1:10:end))

    %estimate accelerations using central difference
    q_2dot = zeros(size(q_dot));
    for k = 2:size(q_dot,2)-1
        delta_qdot = q_dot(:,k+1) - q_dot(:,k-1);
        delta_t = t(k+1) - t(k-1);
        q_2dot(:,k) = delta_qdot./delta_t;
    end

    fprintf('Identification Test: \n');
    [pi_rgb_hat, pi_frcn_hat] = robot.identify_parameters(tau(1:end-1),...
                                                          q(:,2:end-1),...
                                                          q_dot(:,2:end-1), ...
                                                          q_2dot(:,2:end-1));
    pi_real = robot.get_dynamic_parameters_from_urdf('base');
    
    % Printing results of identification
    fprintf('\t parameter \t real \t\t identified OLS \n');
    for k = 1:6
        fprintf('\t %8s \t %8d \t %8d \n', strcat('rgb', num2str(k)), pi_real(k), pi_rgb_hat(k));
    end
   
    for k = 1:4
        fprintf('\t %8s \t %8d \t %8d \n', strcat('frcn', num2str(k)), 0, pi_frcn_hat(k));
    end

    % -------------------------------------------------------------
    % specify a trajectory to follow by the first joint and its parameters
    q_ref = @(t, q0, a) q0 + a(1)*sin(t) + a(2)*sin(2*t) + a(3)*sin(3*t) + ...
                            a(4)*sin(4*t) + a(5)*sin(5*t) + a(6)*sin(6*t);
    a_valid = rand(6,1);
    % simultion for validation
    t = 0:1e-2:10;
    x0 = [-pi/2, 0, 0, 0]';
    x_sim = x0;
    tau = [];
    for i = 1:length(t)-1
        % compute control signal
        u = kp*(q_ref(t(i), q0, a_valid) - x_sim(1,i)) - kd*x_sim(3,i);
        tau = vertcat(tau, u);

        % simulate
        [~, x] = ode45(@(t,x) robot.ode(x,u), [t(i) t(i+1)], x_sim(:,i));
        x_sim = horzcat(x_sim, x(end,:)');
    end
    q = x_sim(1:2,:);
    q_dot = x_sim(3:4,:);
    
    %estimate accelerations using central difference
    q_2dot = zeros(size(q_dot));
    for k = 2:size(q_dot,2)-1
        delta_qdot = q_dot(:,k+1) - q_dot(:,k-1);
        delta_t = t(k+1) - t(k-1);
        q_2dot(:,k) = delta_qdot./delta_t;
    end
    
    Wb = robot.get_rigid_body_observation_matrix(q(:,2:end-1), q_dot(:,2:end-1), q_2dot(:,2:end-1), 'base');
    Wf = robot.get_friction_observation_matrix(q_dot(:,2:end-1), 'continuous');
    T_hat = Wb*pi_rgb_hat + Wf*pi_frcn_hat;
    
    figure
    plot(t(2:end-1), tau(1:end-1), 'LineWidth', 1.5)
    hold on
    plot(t(2:end-1), T_hat(1:2:end), 'LineWidth', 1.5)
    title('validation', 'FontName', 'Times', 'FontSize', 13, 'Interpreter', 'latex');
    legend('$\tau$', '$\hat{\tau}$', 'FontName', 'Times', 'FontSize', 13, 'Interpreter', 'latex');
    xlabel('t (sec)', 'FontName', 'Times', 'FontSize', 13, 'Interpreter', 'latex')
    grid on
end
