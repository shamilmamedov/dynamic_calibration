clc; clear all; close all;

% get robot description
plnr = parse_urdf('planar_manip.urdf');

% get inertial parameters from CAD to compare with identification
% 0 corresponds to the reflected motor inertia
pi_CAD = [plnr.pi(:,1); 0; plnr.pi(:,2)];

return
% load and process pendubot data for verification
data_files = {'harmonic_A_0.3927_v_1.mat', ...
              'harmonic_A_1.5708_v_0.5.mat', ...
              'harmonic_A_0.7854_v_0.5.mat'};
vldtnData = pendubotDataProcessing(data_files{1});


% based on generalized positions, velocities and accelerations and using
% regressor form predict torques
tau_prdctd_CAD = []; tau_prdctd_idnt = [];
for i = 1:length(vldtnData.time)
    qi = [vldtnData.shldr_position(i), vldtnData.elbw_position(i)]';
    qdi = [vldtnData.shldr_velocity(i), vldtnData.elbw_velocity(i)]';
    q2di = [vldtnData.shldr_acceleration(i), vldtnData.elbow_acceleration(i)]';
    
    Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    Yfrctni = frictionRegressor(qdi);
    
    tau_prdctd_CAD = horzcat(tau_prdctd_CAD, Yi*pi_CAD);
%     tau_prdctd_idnt = horzcat(tau_prdctd_idnt, Yi*pi_stnd + Yfrctni*pi_frctn);
end


%%
figure
plot(vldtnData.time, vldtnData.current*0.123, 'LineWidth', 1.25)
hold on
plot(vldtnData.time, tau_prdctd_CAD(1,:), 'LineWidth', 1.25)
plot(vldtnData.time, vldtnData.current*0.123 - tau_prdctd_CAD(1,:)', 'LineWidth', 1.25)
grid on
ylim([-4 4])
legend('$i \times k$', '$\hat{\tau}$', '$\Delta \tau$', 'Interpreter', 'latex')

