% load pendubot data
% rawData = load('position_A_0.7_v_1.0.mat');
% rawData = load('position_A_0.7_v_0.5.mat');
% rawData = load('position_A_1.2_v_0.05.mat');
% rawData = load('position_A_1.1_v_0.01.mat');
% rawData = load('position_A_1.2_v_0.1.mat');

pendubot = pendubotDataProcessing('position_A_1.2_v_0.5.mat');
% pendubot = pendubotDataProcessing('ramp_A_1.2_v_1.mat');


%%

% Validation
pi_CAD = [plnr.pi(:,1); 0; plnr.pi(:,2)];
vldtnRange = 1:1500;
tau_prdctd_OLS = []; tau_prdctd_SDP = []; tau_prdctd_CAD = [];
for i = vldtnRange
%     qi = [pendubot.shldr_position(i), pendubot.elbw_position(i)]';
%     qdi = [pendubot.shldr_velocity_filtered(i), pendubot.elbw_velocity_filtered(i)]';
%     q2di = [pendubot.shldr_acceleration_filtered(i), pendubot.elbow_acceleration_filtered(i)]';
    
    qi = [pendubot.shldr_position(i), pendubot.elbw_position(i)]';
    qdi = [pendubot.shldr_velocity_estimated(i), pendubot.elbw_velocity_estimated(i)]';
    q2di = [pendubot.shldr_acceleration_filtered2(i), pendubot.elbow_acceleration_filtered2(i)]';
    
    if plnrBaseQR.motorDynamicsIncluded
        Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    else
        Yi = full_regressor_plnr(qi, qdi, q2di);
    end
    Ybi = Yi*plnrBaseQR.permutationMatrix(:,1:plnrBaseQR.numberOfBaseParameters);
    Yfrctni = frictionRegressor(qdi);
    tau_prdctd_OLS = horzcat(tau_prdctd_OLS, [Ybi, Yfrctni]*pi_hat);
    tau_prdctd_SDP = horzcat(tau_prdctd_SDP, [Ybi, Yfrctni]*[pi_b; pi_frctn]);
    tau_prdctd_CAD = horzcat(tau_prdctd_CAD, Yi*pi_CAD);
end

%%
figure
plot(pendubot.time(vldtnRange), pendubot.torque(vldtnRange),'r')
hold on
plot(pendubot.time(vldtnRange), tau_prdctd_OLS(1,:),'b-')
plot(pendubot.time(vldtnRange), tau_prdctd_SDP(1,:),'k-')
plot(pendubot.time(vldtnRange), tau_prdctd_CAD(1,:),'g-')
legend('measured', 'predicted OLS', 'predicted SDP', 'predicted CAD')
grid on
