clc; clear all; close all;

%% Creating pendubot instance
path_to_urdf = 'planar_manip.urdf';

robot = Pendubot(path_to_urdf);


%% Identification

% choose file with experimental data
path_to_csv = 'data1.csv';

% parse data
[t,q,tau] = parse_data(path_to_csv);

% estimate velocities and acceleration
[q_dot, q_2dot] = process_data(t, q);

% plot_data(t(3:end-2), q(3:end-2,:), q_dot(3:end-2,:), q_2dot(3:end-2,:), tau(3:end-2))

[pi_rgb_hat, pi_frcn_hat] = robot.identify_parameters(tau(3:end-2), q(3:end-2,:)', ...
                                                      q_dot(3:end-2,:)', q_2dot(3:end-2,:)');


%% Validation

% choose file with experimental data
path_to_csv = 'data3.csv';

% parse data
[t,q,tau] = parse_data(path_to_csv);

% estimate velocities and acceleration
[q_dot, q_2dot] = process_data(t, q);


% ---------------------------------------------------------------


T = robot.get_aggregated_torque_vector(tau(3:end-2));
Ws = robot.get_rigid_body_observation_matrix(q(3:end-2,:)', q_dot(3:end-2,:)',...
                                             q_2dot(3:end-2,:)', 'standard');
Wb = robot.get_rigid_body_observation_matrix(q(3:end-2,:)', q_dot(3:end-2,:)',...
                                             q_2dot(3:end-2,:)', 'base');
Wf = robot.get_friction_observation_matrix(q_dot(3:end-2,:)', 'continuous');

% prediction by CAD
T_hat_CAD = Ws*robot.get_dynamic_parameters_from_urdf('standard');
T_hat_OLS = Wb*pi_rgb_hat + Wf*pi_frcn_hat;

figure
subplot(2,1,1)
plot(t(3:end-2), tau(3:end-2), 'LineWidth', 1.5)
hold on
plot(t(3:end-2), T_hat_CAD(1:2:end))
plot(t(3:end-2), T_hat_OLS(1:2:end))
legend('$\tau$ measured', '$\tau$ predicted CAD', 'Interpreter', 'latex', 'FontSize', 13) 
grid on
subplot(2,1,2)
plot(t(3:end-2), tau(3:end-2) - T_hat_CAD(1:2:end), 'LineWidth', 1.5)
title('Prediction error')
grid on       




















%% Local Functions

function [q_dot, q_2dot] = process_data(t, q)
    % estimate velocties and accelerations
    % no filtering is being peformed on data
    no_observations = numel(t);

    % estimate velocities using central difference
    q_dot = zeros(size(q));
    for k = 2:no_observations-1
        delta_q = q(k+1,:) - q(k-1,:);
        delta_t = t(k+1) - t(k-1);
        q_dot(k,:) = delta_q./delta_t; 
    end

    % estimate accelerations using central difference
    q_2dot = zeros(size(q));
    for k = 2:no_observations-1
        delta_q_dot = q_dot(k+1,:) - q_dot(k-1,:);
        delta_t = t(k+1) - t(k-1);
        q_2dot(k,:) = delta_q_dot./delta_t; 
    end
end


function [t,q,tau] = parse_data(path_to_file)
    % load file
    data = load(path_to_file);

    t = data(:,1) - data(1,1);
    tau = -data(:,6)*0.123;
    q = data(:,2:3);
end


function plot_data(t, q, q_dot, q_2dot, tau)
    figure
    plot(t, q, 'LineWidth', 1.5)
    legend('$q_1$', '$q_2$', 'Interpreter', 'latex', 'FontSize', 13)
    xlabel('$t$ (sec)', 'Interpreter', 'latex', 'FontSize', 13)
    grid on

    figure
    plot(t, q_dot, 'LineWidth', 1.5)
    legend('$\dot{q}_1$', '$\dot{q}_2$', 'Interpreter', 'latex', 'FontSize', 13)
    xlabel('$t$ (sec)', 'Interpreter', 'latex', 'FontSize', 13)
    grid on

    figure
    plot(t, q_2dot, 'LineWidth', 1.5)
    legend('$\ddot{q}_1$', '$\ddot{q}_2$', 'Interpreter', 'latex', 'FontSize', 13)
    xlabel('$t$ (sec)', 'Interpreter', 'latex', 'FontSize', 13)
    grid on

    figure
    plot(t, tau, 'LineWidth', 1.5)
    ylabel('$\tau$ (Nm)', 'Interpreter', 'latex', 'FontSize', 13)
    xlabel('$t$ (sec)', 'Interpreter', 'latex', 'FontSize', 13)
    grid on
end