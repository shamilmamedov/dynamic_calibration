% -----------------------------------------------------------------------
% In this script I carry out validation of identified inertial parameters 
% of the pendubot. 
% -----------------------------------------------------------------------

% load and process pendubot data for valication
% vldtnData = pendubotDataProcessing('position_A_0.3141_v_0.5.mat');
vldtnData = pendubotDataProcessing('position_A_0.3141_v_1.mat');
% vldtnData = pendubotDataProcessing('position_A_0.3141_v_2.mat');

% get inertial parameters from CAD to compare with identification
pi_CAD = [plnr.pi(:,1); 0; plnr.pi(:,2)];

% based on generalized positions, velocities and accelerations and using
% regressor form predict torques
vldtnRange = 1:500 ;%size(vldtnData.time,1);
tau_prdctd_OLS = []; tau_prdctd_SDP = []; tau_prdctd_CAD = [];
for i = vldtnRange
    qi = [vldtnData.shldr_position(i), vldtnData.elbw_position(i)]';
    qdi = [vldtnData.shldr_velocity_filtered(i), vldtnData.elbw_velocity_filtered(i)]';
    q2di = [vldtnData.shldr_acceleration_filtered(i), vldtnData.elbow_acceleration_filtered(i)]';
        
    Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    Ybi = Yi*fullRegressor2BaseRegressor;
    Yfrctni = frictionRegressor(qdi);
    
    tau_prdctd_OLS = horzcat(tau_prdctd_OLS, [Ybi, Yfrctni]*pi_hat_OLS);
    tau_prdctd_SDP = horzcat(tau_prdctd_SDP, [Ybi, Yfrctni]*[pi_b; pi_frctn]);
%     tau_prdctd_CAD = horzcat(tau_prdctd_CAD, [Yi, Yfrctni] * [pi_CAD; pi_frctn]);
    tau_prdctd_CAD = horzcat(tau_prdctd_CAD, Yi*pi_CAD);
end

% plot real torque and predictions
figure
plot(vldtnData.time(vldtnRange), vldtnData.torque(vldtnRange),'r')
hold on
plot(vldtnData.time(vldtnRange), tau_prdctd_OLS(1,:),'b-')
plot(vldtnData.time(vldtnRange), tau_prdctd_SDP(1,:),'k-')
plot(vldtnData.time(vldtnRange), tau_prdctd_CAD(1,:),'g-')
legend('measured', 'predicted OLS', 'predicted SDP', 'predicted CAD')
grid on
