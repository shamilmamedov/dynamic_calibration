% ------------------------------------------------------------------------
% Load validation trajectory
% ------------------------------------------------------------------------
% vldtn_data = csvread('ur-19_10_01-13_51_41.csv');
vldtn_data = csvread('ur-19_10_01-14_04_13.csv');

t_msrd = vldtn_data(:,1) - vldtn_data(1,1); %subtract offset
q_msrd = vldtn_data(:,2:7);
qd_msrd = vldtn_data(:,8:13);
i_msrd = vldtn_data(:,14:19);
i_des = vldtn_data(:,20:25);
tau_des = vldtn_data(:,26:31);

% ------------------------------------------------------------------------
% Filtering Velocities
% ------------------------------------------------------------------------
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
    Y_ulddi = regressorWithMotorDynamics(q_msrd(i,:)',...
                                         qd_fltrd(i,:)',q2d_est(i,:)');
%     Yfrctni = frictionRegressor(qd_fltrd(i,:)');
%     Yb_ulddi = [Y_ulddi*E1, Yfrctni];
%     
%     Wb_uldd = vertcat(Wb_uldd, Yb_ulddi);
%     tau_msrd = horzcat(tau_msrd, diag(drvGains)*i_msrd(i,:)');
%     tau_prdctd = horzcat(tau_prdctd, Yb_ulddi*[pi_b; pi_frctn]);
    
    Yb_ulddi = Y_ulddi*E1;
    Wb_uldd = vertcat(Wb_uldd, Yb_ulddi);
    tau_msrd = horzcat(tau_msrd, diag(drvGains)*i_msrd(i,:)');
    tau_prdctd = horzcat(tau_prdctd, Yb_ulddi*pi_b);
end

%%
for i = 1:6
    figure
    hold on
    plot(t_msrd, tau_msrd(i,:),'r-')
    plot(t_msrd,tau_prdctd(i,:),'k-')
    grid on
end

return
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



