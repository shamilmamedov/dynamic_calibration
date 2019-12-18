clc; clear all; close all;

% ------------------------------------------------------------------------
% Load and calculate desired trajectory
% ------------------------------------------------------------------------
load('opt_sltn_N5T20.mat');
[q,qd,q2d] = mixed_traj(traj_par.t,c_pol,a,b,traj_par.wf,traj_par.N);


% ------------------------------------------------------------------------
% Load the real trajectory of the robot without load
% ------------------------------------------------------------------------
traj_data = csvread('ur-19_10_29-20_sec.csv');

int_idx = 256; % get the point when the trajectory begins
fnl_idx = 2158;

t_msrd = traj_data(int_idx:fnl_idx,1) - traj_data(int_idx,1); %subtract offset
q_msrd = traj_data(int_idx:fnl_idx,2:7);
qd_msrd = traj_data(int_idx:fnl_idx,8:13);
i_msrd = traj_data(int_idx:fnl_idx,14:19);
i_des = traj_data(int_idx:fnl_idx,20:25);
tau_des = traj_data(int_idx:fnl_idx,26:31);


% -----------------------------------------------------------------------
% Load the real trajectory of the robot with load
% -----------------------------------------------------------------------
traj_ldd = csvread('ur-19_11_05-20_sec_mass_2.csv');

int_idx_ldd = 8;
fnl_idx_ldd = 1877;

t_msrd_ldd = traj_ldd(int_idx_ldd:fnl_idx_ldd,1) - traj_ldd(int_idx_ldd,1); %subtract offset
q_msrd_ldd = traj_ldd(int_idx_ldd:fnl_idx_ldd,2:7);
qd_msrd_ldd = traj_ldd(int_idx_ldd:fnl_idx_ldd,8:13);
i_msrd_ldd = traj_ldd(int_idx_ldd:fnl_idx_ldd,14:19);
i_des_ldd = traj_ldd(int_idx_ldd:fnl_idx_ldd,20:25);
tau_des_ldd = traj_ldd(int_idx_ldd:fnl_idx_ldd,26:31);


% ------------------------------------------------------------------------
% Filtering Velocities
% ------------------------------------------------------------------------
% Design filter
vel_filt = designfilt('lowpassiir','FilterOrder',3, ...
        'HalfPowerFrequency',0.2,'DesignMethod','butter');
    
qd_fltrd = zeros(size(qd_msrd));
for i = 1:6
    qd_fltrd(:,i) = filtfilt(vel_filt,qd_msrd(:,i));
end

qd_fltrd_ldd = zeros(size(qd_msrd_ldd));
for i = 1:6
    qd_fltrd_ldd(:,i) = filtfilt(vel_filt,qd_msrd_ldd(:,i));
end


% ------------------------------------------------------------------------
% Estimating accelerations
% ------------------------------------------------------------------------
% Three point central difference
q2d_est = zeros(size(qd_fltrd));
for i = 2:length(qd_fltrd)-1
   dlta_qd_fltrd =  qd_fltrd(i+1,:) -  qd_fltrd(i-1,:);
   dlta_t_msrd = t_msrd(i+1) - t_msrd(i-1);
   q2d_est(i,:) = dlta_qd_fltrd/dlta_t_msrd;
end

q2d_est_ldd = zeros(size(qd_fltrd_ldd));
for i = 2:length(qd_fltrd_ldd)-1
   dlta_qd_fltrd_ldd =  qd_fltrd_ldd(i+1,:) -  qd_fltrd_ldd(i-1,:);
   dlta_t_msrd_ldd = t_msrd_ldd(i+1) - t_msrd_ldd(i-1);
   q2d_est_ldd(i,:) = dlta_qd_fltrd_ldd/dlta_t_msrd_ldd;
end

% Zeros phase filtering acceleration obtained by finite difference
accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.05,'DesignMethod','butter');
for i = 1:6
    q2d_est(:,i) = filtfilt(accel_filt,q2d_est(:,i));
end

for i = 1:6
    q2d_est_ldd(:,i) = filtfilt(accel_filt,q2d_est_ldd(:,i));
end

% ------------------------------------------------------------------------
% Filtering current
% ------------------------------------------------------------------------
% Zeros phase filtering acceleration obtained by finite difference
curr_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.1,'DesignMethod','butter');

for i = 1:6
    i_fltrd(:,i) = filtfilt(curr_filt,i_msrd(:,i));
end

for i = 1:6
    i_fltrd_ldd(:,i) = filtfilt(curr_filt,i_msrd_ldd(:,i));
end






return
% ------------------------------------------------------------------------
% Construncting regressor and performing least squares
% ------------------------------------------------------------------------

W1 = []; W2 = []; W3 = []; W4 = []; W5 = []; W6 = [];
I1 = []; I2 = []; I3 = []; I4 = []; I5 = []; I6 = [];

n1 = 1;
n2 = 2;
n3 = 9;
n4 = 16;
n5 = 23;
n6 = 30;
xi = 5e-4;

for i = 1:length(t_msrd)
    Yi = base_regressor(q_msrd(i,:)',qd_fltrd(i,:)',q2d_est(i,:)');
% Complementing regressor with friction terms

%     drvi = ur10_drv_rgsr(qd_fltrd(i,:)');
    drvi = ur10_drv_rgsr(q2d_est(i,:)');
    frctni = ur10_frctn_rgsr(qd_fltrd(i,:)');
    
% As drive gains are unknown we want to build regressor for each joint in
% order to identify paramters as seen from this joint point of view
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
    
%     W1 = vertcat(W1,[Yi(1,n1:4),Yi(1,6:11),Yi(1,13:18),Yi(1,20:end), frctni(1,:)]);
%     W2 = vertcat(W2,[Yi(2,n2:end),frctni(2,:)]);
%     W3 = vertcat(W3,[Yi(3,n3:end),frctni(3,:)]);
%     W4 = vertcat(W4,[Yi(4,n4:end),frctni(4,:)]);
%     W5 = vertcat(W5,[Yi(5,n5:end),frctni(5,:)]);
%     W6 = vertcat(W6,[Yi(6,n6:end),frctni(6,:)]);
    
    W2 = vertcat(W2,[Yi_j2, frctni(2,:)]);
    W3 = vertcat(W3,[Yi_j3, frctni(3,:)]);
    W4 = vertcat(W4,[Yi_j4, frctni(4,:)]);
    W5 = vertcat(W5,[Yi_j5, frctni(5,:)]);
    W6 = vertcat(W6,[Yi_j6, frctni(6,:)]);

%     I1 = vertcat(I1,i_fltrd(i,1));
    I2 = vertcat(I2,i_fltrd(i,2));
    I3 = vertcat(I3,i_fltrd(i,3));
    I4 = vertcat(I4,i_fltrd(i,4));
    I5 = vertcat(I5,i_fltrd(i,5));
    I6 = vertcat(I6,i_fltrd(i,6));
end

% pi1_hat = (W1'*W1)\(W1'*I1);
pi2_hat = (W2'*W2)\(W2'*I2);
pi3_hat = (W3'*W3)\(W3'*I3);
pi4_hat = (W4'*W4)\(W4'*I4);
pi5_hat = (W5'*W5)\(W5'*I5);
pi6_hat = (W6'*W6)\(W6'*I6);








