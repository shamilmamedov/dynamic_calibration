clc; clear all; close all;

% get robot description
plnr = parse_urdf('planar_manip.urdf');

% get inertial parameters from CAD to compare with identification
% 0 corresponds to the reflected motor inertia
pi_CAD = [plnr.pi(:,1); 0; plnr.pi(:,2)];

% load and process pendubot data for verification
data_files = {'harmonic_A_0.3927_v_1.mat', ...
              'harmonic_A_1.5708_v_0.5.mat', ...
              'harmonic_A_0.7854_v_0.5.mat'};
idntfcnData = pendubotDataProcessing(data_files{3});


% based on generalized positions, velocities and accelerations and using
% regressor form predict torques
tau_prdctd_CAD = []; 
Dlta_tau = [];
for i = 1:length(idntfcnData.time)
    qi = [idntfcnData.shldr_position(i), idntfcnData.elbw_position(i)]';
    qdi = [idntfcnData.shldr_velocity(i), idntfcnData.elbw_velocity(i)]';
    q2di = [idntfcnData.shldr_acceleration_filtered(i), idntfcnData.elbow_acceleration_filtered(i)]';
    
    Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    Yfrctni = frictionRegressor(qdi);
    
    tau_prdctd_CAD = horzcat(tau_prdctd_CAD, Yi*pi_CAD);
    Dlta_tau = horzcat(Dlta_tau, -tau_prdctd_CAD(:,end) + ...
                                    [idntfcnData.torque_filtered(i); 0]);
%     tau_prdctd_idnt = horzcat(tau_prdctd_idnt, Yi*pi_stnd + Yfrctni*pi_frctn);
end

%% Find friction parameters of the first joint
% figure
% plot(idntfcnData.time - idntfcnData.time(1), Dlta_tau(1,:)')
% hold on
% plot(idntfcnData.time - idntfcnData.time(1), tau_prdctd_CAD(1,:))
% plot(idntfcnData.time, idntfcnData.torque_filtered, 'LineWidth', 1.25)

figure
plot(idntfcnData.shldr_velocity, Dlta_tau(1,:))
grid on

W_frcn = friction_observation_matrix(idntfcnData.shldr_velocity);
pi_frcn_1 = mldivide(W_frcn'*W_frcn, W_frcn'*Dlta_tau(1,:)')



%% Validate with friction model
vldtnData = pendubotDataProcessing(data_files{2});

tau_prdctd_CAD = [];
tau_prdctd_frcn = [];
Dlta_tau = [];
for i = 1:length(vldtnData.time)
    qi = [vldtnData.shldr_position(i), vldtnData.elbw_position(i)]';
    qdi = [vldtnData.shldr_velocity(i), vldtnData.elbw_velocity(i)]';
    q2di = [vldtnData.shldr_acceleration(i), vldtnData.elbow_acceleration(i)]';
    
    Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    Yfi = friction_regressor(vldtnData.shldr_velocity(i));
    tau_prdctd_frcn = horzcat(tau_prdctd_frcn, Yfi*pi_frcn_1);
    
    tau_prdctd_CAD = horzcat(tau_prdctd_CAD, Yi*pi_CAD);
    Dlta_tau = horzcat(Dlta_tau, -tau_prdctd_CAD(:,end) + ...
                                    [vldtnData.current(i)*0.123; 0]);
%     tau_prdctd_idnt = horzcat(tau_prdctd_idnt, Yi*pi_stnd + Yfrctni*pi_frctn);
end

%%
% figure
% plot(vldtnData.time, Dlta_tau, 'LineWidth', 1.25)
% grid on
% legend('\Delta \tau_1', '\Delta \tau_2')

figure
plot(vldtnData.time, vldtnData.current*0.123, 'LineWidth', 1.25)
hold on
plot(vldtnData.time, tau_prdctd_CAD(1,:), 'LineWidth', 1.25)
% plot(vldtnData.time, vldtnData.current*0.123 - tau_prdctd_CAD(1,:)', 'LineWidth', 1.25)
plot(vldtnData.time, tau_prdctd_CAD(1,:) + tau_prdctd_frcn, 'LineWidth', 1.25)
grid on
ylim([-4 4])
legend('$i \times k$', '$\hat{\tau}$', '$\Delta \tau$', 'Interpreter', 'latex')




function Y_frcn = friction_regressor(q_dot)
    Y_frcn = [q_dot, sign(q_dot)];
end

function W_frcn = friction_observation_matrix(q_dot)
    no_observations = length(q_dot);
    W_frcn = zeros(no_observations, 2);    
    for i = 1:no_observations
        W_frcn(i,:) = friction_regressor(q_dot(i));
    end
end