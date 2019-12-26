clc; clear all; close all;

% ------------------------------------------------------------------------
% Load and calculate desired trajectory
% ------------------------------------------------------------------------
load('ptrnSrch_N5T20QR.mat');
[q,qd,q2d] = mixed_traj(traj_par.t,c_pol,a,b,traj_par.wf,traj_par.N);


% ------------------------------------------------------------------------
% Load the real trajectory of the robot without load
% ------------------------------------------------------------------------
traj_data = csvread('ur-19_12_23_free.csv');

int_idx = 1; % get the point when the trajectory begins
fnl_idx = 2016;

unloadedTrajectory = struct; % structure with trajectory data

unloadedTrajectory.t = traj_data(int_idx:fnl_idx,1) - traj_data(int_idx,1);
unloadedTrajectory.q = traj_data(int_idx:fnl_idx,2:7);
unloadedTrajectory.qd = traj_data(int_idx:fnl_idx,8:13);
unloadedTrajectory.i = traj_data(int_idx:fnl_idx,14:19);
unloadedTrajectory.i_des = traj_data(int_idx:fnl_idx,20:25);
unloadedTrajectory.tau_des = traj_data(int_idx:fnl_idx,26:31);


% -----------------------------------------------------------------------
% Load the real trajectory of the robot with load
% -----------------------------------------------------------------------
traj_ldd = csvread('ur-19_12_23_load.csv');

int_idx_ldd = 1;
fnl_idx_ldd = 2040;

loadedTrajectory = struct;

loadedTrajectory.t = traj_ldd(int_idx_ldd:fnl_idx_ldd,1) - traj_ldd(int_idx_ldd,1); 
loadedTrajectory.q = traj_ldd(int_idx_ldd:fnl_idx_ldd,2:7);
loadedTrajectory.qd = traj_ldd(int_idx_ldd:fnl_idx_ldd,8:13);
loadedTrajectory.i = traj_ldd(int_idx_ldd:fnl_idx_ldd,14:19);
loadedTrajectory.i_des = traj_ldd(int_idx_ldd:fnl_idx_ldd,20:25);
loadedTrajectory.tau_des = traj_ldd(int_idx_ldd:fnl_idx_ldd,26:31);


% ------------------------------------------------------------------------
% Filtering Velocities
% ------------------------------------------------------------------------
% Design filter
vel_filt = designfilt('lowpassiir','FilterOrder',3, ...
        'HalfPowerFrequency',0.2,'DesignMethod','butter');
    
unloadedTrajectory.qd_fltrd = zeros(size(unloadedTrajectory.qd));
for i = 1:6
    unloadedTrajectory.qd_fltrd(:,i) = filtfilt(vel_filt,unloadedTrajectory.qd(:,i));
end

loadedTrajectory.qd_fltrd = zeros(size(loadedTrajectory.qd));
for i = 1:6
    loadedTrajectory.qd_fltrd(:,i) = filtfilt(vel_filt,loadedTrajectory.qd(:,i));
end


% ------------------------------------------------------------------------
% Estimating accelerations
% ------------------------------------------------------------------------
% Three point central difference
unloadedTrajectory.q2d_est = zeros(size(unloadedTrajectory.qd_fltrd));
for i = 2:length(unloadedTrajectory.qd_fltrd)-1
   dlta_qd_fltrd =  unloadedTrajectory.qd_fltrd(i+1,:) -  ...
                        unloadedTrajectory.qd_fltrd(i-1,:);
   dlta_t_msrd = unloadedTrajectory.t(i+1) - unloadedTrajectory.t(i-1);
   unloadedTrajectory.q2d_est(i,:) = dlta_qd_fltrd/dlta_t_msrd;
end

loadedTrajectory.q2d_est = zeros(size(loadedTrajectory.qd_fltrd));
for i = 2:length(loadedTrajectory.qd_fltrd)-1
   dlta_qd_fltrd_ldd =  loadedTrajectory.qd_fltrd(i+1,:) - ...
                            loadedTrajectory.qd_fltrd(i-1,:);
   dlta_t_msrd_ldd = loadedTrajectory.t(i+1) - loadedTrajectory.t(i-1);
   loadedTrajectory.q2d_est(i,:) = dlta_qd_fltrd_ldd/dlta_t_msrd_ldd;
end

% Zeros phase filtering acceleration obtained by finite difference
accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.05,'DesignMethod','butter');
for i = 1:6
    unloadedTrajectory.q2d_est(:,i) = filtfilt(accel_filt,unloadedTrajectory.q2d_est(:,i));
end

for i = 1:6
    loadedTrajectory.q2d_est(:,i) = filtfilt(accel_filt,loadedTrajectory.q2d_est(:,i));
end


% ------------------------------------------------------------------------
% Filtering current
% ------------------------------------------------------------------------
% Zeros phase filtering acceleration obtained by finite difference
curr_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.1,'DesignMethod','butter');

for i = 1:6
    unloadedTrajectory.i_fltrd(:,i) = filtfilt(curr_filt,unloadedTrajectory.i(:,i));
end

for i = 1:6
    loadedTrajectory.i_fltrd(:,i) = filtfilt(curr_filt,loadedTrajectory.i(:,i));
end

