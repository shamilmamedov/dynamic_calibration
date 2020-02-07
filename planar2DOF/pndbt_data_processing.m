% clc; clear all; close all;
% get robot description
run('plnr_idntfcn.m')

% load pendubot data
% rawData = load('position_A_0.7_v_0.5.mat');
rawData = load('position_A_1.2_v_0.05.mat');

% parse pendubot data
pendubot.time = rawData.data(:,1) - rawData.data(1,1);
pendubot.current = rawData.data(:,2);
pendubot.current_desired = rawData.data(:,4);
pendubot.torque = rawData.data(:,3);

pendubot.shldr_position = rawData.data(:,7) - pi/2; % to fit model
pendubot.shldr_velocity = rawData.data(:,9);
pendubot.elbw_position = rawData.data(:,8);
pendubot.elbw_velocity = rawData.data(:,10);

% filter velocties with zero phas filter
vlcty_fltr = designfilt('lowpassiir','FilterOrder',5, ...
                        'HalfPowerFrequency',0.25,'DesignMethod','butter');
pendubot.shldr_velocity_filtered = filtfilt(vlcty_fltr, pendubot.shldr_velocity);
pendubot.elbw_velocity_filtered = filtfilt(vlcty_fltr, pendubot.elbw_velocity);

% estimating accelerations based on filtered velocities
q2d1 = zeros(size(pendubot.shldr_velocity));
q2d2 = zeros(size(pendubot.elbw_velocity));
for i = 2:size(pendubot.shldr_velocity,1)-1
    q2d1(i) = (pendubot.shldr_velocity_filtered(i+1) - pendubot.shldr_velocity_filtered(i-1))/...
                (pendubot.time(i+1) - pendubot.time(i-1));
    q2d2(i) = (pendubot.elbw_velocity_filtered(i+1) - pendubot.elbw_velocity_filtered(i-1))/...
                (pendubot.time(i+1) - pendubot.time(i-1));
end
pendubot.shldr_acceleration = q2d1;
pendubot.elbow_acceleration = q2d2;

% filter estimated accelerations with zero phase filter
aclrtn_fltr = designfilt('lowpassiir','FilterOrder',5, ...
                        'HalfPowerFrequency',0.25,'DesignMethod','butter');
pendubot.shldr_acceleration_filtered = filtfilt(aclrtn_fltr, pendubot.shldr_acceleration);
pendubot.elbow_acceleration_filtered = filtfilt(aclrtn_fltr, pendubot.elbow_acceleration);

% filter torque measurements using zero phase filter
trq_fltr = designfilt('lowpassiir','FilterOrder',5, ...
                       'HalfPowerFrequency',0.2,'DesignMethod','butter');
pendubot.torque_filtered = filtfilt(trq_fltr, pendubot.torque);

%% 

% figure
% plot(pendubot.time, pendubot.current*0.123)
% hold on
% plot(pendubot.time, pendubot.torque)

return
plotJointPositions(pendubot)
plotJointVelocities(pendubot)
plotJointAccelerations(pendubot)

plnr_visualize([pendubot.shldr_position, pendubot.elbw_position], plnr)


%% Functions

function plotJointPositions(pendubot)
figure
subplot(2,1,1)
    plot(pendubot.time, pendubot.shldr_position)
    hold on
    plot(pendubot.time, pendubot.current_desired)
    xlim([0 1])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$q_1$, rad','interpreter', 'latex')
    grid on
    legend('measured', 'desired')
subplot(2,1,2)
    plot(pendubot.time, pendubot.elbw_position)
    xlim([0 1])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$q_2$, rad','interpreter', 'latex')
    grid on
end

function plotJointVelocities(pendubot)
figure
subplot(2,1,1)
    plot(pendubot.time, pendubot.shldr_velocity)
    hold on
    plot(pendubot.time, pendubot.shldr_velocity_filtered)
    xlim([0 5])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$\dot{q}_1$, rad/s','interpreter', 'latex')
    legend('measured', 'filtered')
    grid on
subplot(2,1,2)
    plot(pendubot.time, pendubot.elbw_velocity)
    hold on
    plot(pendubot.time, pendubot.elbw_velocity_filtered)
    xlim([0 5])
    xlabel('t, sec','interpreter', 'latex')
    ylabel('$\dot{q}_2$, rad/s','interpreter', 'latex')
    legend('measured', 'filtered')
    grid on
end

function plotJointAccelerations(pendubot)
figure
subplot(2,1,1)
    plot(pendubot.time, pendubot.shldr_acceleration)
    hold on
    plot(pendubot.time, pendubot.shldr_acceleration_filtered)
    xlim([0 2])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$\ddot{q}_1$, rad/s$^2$','interpreter', 'latex')
    legend('estimate', 'filetered estimate')
    grid on
subplot(2,1,2)
    plot(pendubot.time, pendubot.elbow_acceleration)
    hold on
    plot(pendubot.time, pendubot.elbow_acceleration_filtered)
    xlim([0 2])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$\ddot{q}_2$, rad/s$^2$','interpreter', 'latex')
    legend('estimate', 'filetered estimate')
    grid on
end