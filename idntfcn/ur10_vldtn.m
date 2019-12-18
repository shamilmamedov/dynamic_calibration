% ------------------------------------------------------------------------
% Load validation trajectory
% ------------------------------------------------------------------------
vldtn_data = csvread('ur-19_10_01-13_51_41.csv');
% vldtn_data = csvread('ur-19_10_01-14_04_13.csv');

t_msrd = vldtn_data(:,1) - vldtn_data(1,1); %subtract offset
q_msrd = vldtn_data(:,2:7);
qd_msrd = vldtn_data(:,8:13);
i_msrd = vldtn_data(:,14:19);
i_des = vldtn_data(:,20:25);
tau_des = vldtn_data(:,26:31);

% ------------------------------------------------------------------------
% Filtering Velocities
% ------------------------------------------------------------------------
% wf = 2*pi/traj_par.T; % main frequency of the data
% wc = 10*wf*traj_par.N/2/pi;

vel_filt = designfilt('lowpassiir','FilterOrder',3, ...
        'HalfPowerFrequency',0.2,'DesignMethod','butter');

qd_fltrd = zeros(size(qd_msrd));
for i = 1:6
    qd_fltrd(:,i) = filtfilt(vel_filt,qd_msrd(:,i));
end

% ------------------------------------------------------------------------
% Estimating accelerations
% ------------------------------------------------------------------------
q2d_est = zeros(size(qd_fltrd));

% Three point central difference
for i = 2:length(qd_fltrd)-1
   dlta_qd_fltrd =  qd_fltrd(i+1,:) -  qd_fltrd(i-1,:);
   dlta_t_msrd = t_msrd(i+1) - t_msrd(i-1);
   q2d_est(i,:) = dlta_qd_fltrd/dlta_t_msrd;
end

% Zeros phase filtering acceleration obtained by finite difference
accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.05,'DesignMethod','butter');
for i = 1:6
    q2d_est(:,i) = filtfilt(accel_filt,q2d_est(:,i));
end

% -----------------------------------------------------------------------
% Predicting torques
% -----------------------------------------------------------------------
%Constracting regressor matrix
Wb_uldd = []; tau_msrd = []; tau_prdctd = [];
for i = 1:length(t_msrd)
    Yb_ulddi = base_regressor_UR10E(q_msrd(i,:)',...
                qd_fltrd(i,:)',q2d_est(i,:)');
    Yfrctni = ur10_frctn_rgsr(qd_fltrd(i,:)');
    Ydrvi = ur10_drv_rgsr(q2d_est(i,:)');
    Wb_uldd = vertcat(Wb_uldd,[Yb_ulddi, Ydrvi, Yfrctni]);
    tau_msrd = horzcat(tau_msrd, diag(drv_gns)*i_msrd(i,:)');
    tau_prdctd = horzcat(tau_prdctd, [Yb_ulddi, Ydrvi, Yfrctni]*...
                    [pi_b; pi_rtr; pi_frctn]);
end

%%
fig = figure;
subplot(3,2,1)
    plot(t_msrd,tau_msrd(1,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(1,:),'k-')
    ylabel('\tau_1')
    grid on
subplot(3,2,2)
    plot(t_msrd,tau_msrd(2,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(2,:),'k-')
    ylabel('\tau_2')
    grid on
subplot(3,2,3)
    plot(t_msrd,tau_msrd(3,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(3,:),'k-')
    ylabel('\tau_3')
    grid on
subplot(3,2,4)
    plot(t_msrd,tau_msrd(4,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(4,:),'k-')
    ylabel('\tau_4')
    grid on
subplot(3,2,5)
    plot(t_msrd,tau_msrd(5,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(5,:),'k-')
    ylabel('\tau_5')
    xlabel('t')
    grid on
subplot(3,2,6)
    plot(t_msrd,tau_msrd(6,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(6,:),'k-')
    ylabel('\tau_6')
    xlabel('t')
    grid on


return
% ------------------------------------------------------------------------
% Predicting currents
% -----------------------------------------------------------------------
I = cell(1,6);

for i = 1:length(t_msrd)
    Yi = base_regressor(q_msrd(i,:)',qd_fltrd(i,:)',q2d_est(i,:)');
% Complementing regressor with friction terms
%     drvi = ur10_drv_rgsr(qd_fltrd(i,:)');
    drvi = ur10_drv_rgsr(q2d_est(i,:)');
    frctni = ur10_frctn_rgsr(qd_fltrd(i,:)');
   
    Yi_l1 = Yi(:,1);
    Yi_l2 = Yi(:,2:8);
    Yi_l3 = Yi(:,9:15);
    Yi_l4 = Yi(:,16:22);
    Yi_l5 = Yi(:,23:29);
    Yi_l6 = Yi(:,30:36);
           
%     Yi_j2 = [Yi_l2(2,:), drvi(2), Yi_l3(2,:), Yi_l4(2,:), Yi_l5(2,:), Yi_l6(2,:)];
    Yi_j2 = [Yi_l2(2,:), Yi_l3(2,:), Yi_l4(2,:), Yi_l5(2,:), Yi_l6(2,:)];
    Yi_j3 = [Yi_l3(3,:), drvi(3), Yi_l4(3,:), Yi_l5(3,:), Yi_l6(3,:)];
    Yi_j4 = [Yi_l4(4,:), drvi(4), Yi_l5(4,:), Yi_l6(4,:)];
    Yi_j5 = [Yi_l5(5,:), drvi(5), Yi_l6(5,:)];
    Yi_j6 = [Yi_l6(6,:), drvi(6)];
    
% Computing currents    
%     I{1} = vertcat(I{1},[Yi(1,n1:4),Yi(1,6:11),Yi(1,13:18),Yi(1,20:end) frctn1i]*pi1_hat);
%     I{2} = vertcat(I{2},[Yi(2,n2:end),frctn2i]*pi2_hat);
%     I{3} = vertcat(I{3},[Yi(3,n3:end),frctn3i]*pi3_hat);
%     I{4} = vertcat(I{4},[Yi(4,n4:end),frctn4i]*pi4_hat);
%     I{5} = vertcat(I{5},[Yi(5,n5:end),frctn5i]*pi5_hat);
%     I{6} = vertcat(I{6},[Yi(6,n6:end),frctn6i]*pi6_hat);
    
    I{2} = vertcat(I{2},[Yi_j2,frctni(2,:)]*pi2_hat);
    I{3} = vertcat(I{3},[Yi_j3,frctni(3,:)]*pi3_hat);
    I{4} = vertcat(I{4},[Yi_j4,frctni(4,:)]*pi4_hat);
    I{5} = vertcat(I{5},[Yi_j5,frctni(5,:)]*pi5_hat);
    I{6} = vertcat(I{6},[Yi_j6,frctni(6,:)]*pi6_hat);
    
end

for i = 2:6
    figure
    plot(t_msrd,i_msrd(:,i))
    hold on
    plot(t_msrd,I{i})
end
