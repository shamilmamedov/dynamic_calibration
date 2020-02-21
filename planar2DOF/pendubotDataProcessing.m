function pendubot = pendubotDataProcessing(file)

clc; clear all; close all;
file = 'position_A_1.2_v_1.mat';

% load raw data
rawData = load(file);

% parse pendubot data
pendubot.time = rawData.data(:,1) - rawData.data(1,1);
pendubot.current = -rawData.data(:,2); % !!!!!!!!!!!!!!!!!
pendubot.current_desired = rawData.data(:,4);
pendubot.torque = -rawData.data(:,3); % !!!!!!!!!!!!!!

pendubot.shldr_position = rawData.data(:,7) - pi/2; % to fit model
pendubot.shldr_velocity = rawData.data(:,9);
pendubot.elbw_position = rawData.data(:,8);
pendubot.elbw_velocity = rawData.data(:,10);


% filter position to get rid of errors
pstn_fltr = designfilt('lowpassiir','FilterOrder',5, ...
                       'HalfPowerFrequency',0.05, 'DesignMethod','butter');
                   
pendubot.shldr_position_fltrd = filtfilt(pstn_fltr, pendubot.shldr_position);
pendubot.elbw_position_fltrd = filtfilt(pstn_fltr, pendubot.elbw_position);


% velocity estimattion from position
qd1 = zeros(size(pendubot.shldr_velocity));
qd2 = zeros(size(pendubot.elbw_velocity));
for i = 2:size(pendubot.shldr_velocity,1)-1
    qd1(i) = (pendubot.shldr_position(i+1) - pendubot.shldr_position(i-1))/...
                (pendubot.time(i+1) - pendubot.time(i-1));
    qd2(i) = (pendubot.elbw_position(i+1) - pendubot.elbw_position(i-1))/...
                (pendubot.time(i+1) - pendubot.time(i-1));
end
pendubot.shldr_velocity_estimated = qd1;
pendubot.elbw_velocity_estimated = qd2;


% filter velocties with zero phase filter
vlcty_fltr_shldr = designfilt('lowpassiir','FilterOrder',5, ...
                              'HalfPowerFrequency',0.1,...
                              'DesignMethod','butter');
vlcty_fltr_elbw = designfilt('lowpassiir','FilterOrder',5, ...
                             'HalfPowerFrequency',0.15,...
                             'DesignMethod','butter');
pendubot.shldr_velocity_filtered = filtfilt(vlcty_fltr_shldr, pendubot.shldr_velocity);
pendubot.elbw_velocity_filtered = filtfilt(vlcty_fltr_elbw, pendubot.elbw_velocity);
pendubot.shldr_velocity_estimated = filtfilt(vlcty_fltr_shldr, pendubot.shldr_velocity_estimated);
pendubot.elbw_velocity_estimated = filtfilt(vlcty_fltr_elbw, pendubot.elbw_velocity_estimated);

plotJointPositions(pendubot)
plotJointVelocities(pendubot);
return

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

q2d1 = zeros(size(pendubot.shldr_velocity_estimated));
q2d2 = zeros(size(pendubot.elbw_velocity_estimated));
for i = 2:size(pendubot.shldr_velocity,1)-1
    q2d1(i) = (pendubot.shldr_velocity_estimated(i+1) - pendubot.shldr_velocity_estimated(i-1))/...
                (pendubot.time(i+1) - pendubot.time(i-1));
    q2d2(i) = (pendubot.elbw_velocity_estimated(i+1) - pendubot.elbw_velocity_estimated(i-1))/...
                (pendubot.time(i+1) - pendubot.time(i-1));
end
pendubot.shldr_acceleration2 = q2d1;
pendubot.elbow_acceleration2 = q2d2;


% filter estimated accelerations with zero phase filter
aclrtn_fltr = designfilt('lowpassiir','FilterOrder',5, ...
                        'HalfPowerFrequency',0.2,'DesignMethod','butter');
pendubot.shldr_acceleration_filtered = filtfilt(aclrtn_fltr, pendubot.shldr_acceleration);
pendubot.elbow_acceleration_filtered = filtfilt(aclrtn_fltr, pendubot.elbow_acceleration);

pendubot.shldr_acceleration_filtered2 = filtfilt(aclrtn_fltr, pendubot.shldr_acceleration2);
pendubot.elbow_acceleration_filtered2 = filtfilt(aclrtn_fltr, pendubot.elbow_acceleration2);


% filter torque measurements using zero phase filter
trq_fltr = designfilt('lowpassiir','FilterOrder',5, ...
                       'HalfPowerFrequency',0.1,'DesignMethod','butter');
pendubot.torque_filtered = filtfilt(trq_fltr, pendubot.torque);




plotJointPositions(pendubot)
plotJointVelocities(pendubot); 
plotJointAccelerations(pendubot)
plotTorque(pendubot)

plnr_visualize([pendubot.shldr_position, pendubot.elbw_position], plnr)



function plotTorque(pendubot)
figure
% plot(pendubot.time, pendubot.current*0.123)
hold on
plot(pendubot.time, pendubot.torque)
plot(pendubot.time, pendubot.torque_filtered, 'LineWidth', 1.5)
xlabel('$t$, sec','interpreter', 'latex')
ylabel('$\tau$, Nm', 'interpreter', 'latex');
legend('torque measured', 'torque filtered')
grid on
end

function plotJointPositions(pendubot)
figure
subplot(2,1,1)
    plot(pendubot.time, pendubot.shldr_position)
    hold on
    plot(pendubot.time, pendubot.shldr_position_fltrd)
%     xlim([0 5])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$q_1$, rad','interpreter', 'latex')
    grid on
    legend('measured', 'desired')
subplot(2,1,2)
    plot(pendubot.time, pendubot.elbw_position)
    hold on
    plot(pendubot.time, pendubot.elbw_position_fltrd)
%     xlim([0 5])
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
    plot(pendubot.time, pendubot.shldr_velocity_estimated)
    xlim([0 5])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$\dot{q}_1$, rad/s','interpreter', 'latex')
    legend('measured', 'filtered', 'estimated')
    grid on
subplot(2,1,2)
    plot(pendubot.time, pendubot.elbw_velocity)
    hold on
    plot(pendubot.time, pendubot.elbw_velocity_filtered)
    plot(pendubot.time, pendubot.elbw_velocity_estimated)
    xlim([0 5])
    xlabel('t, sec','interpreter', 'latex')
    ylabel('$\dot{q}_2$, rad/s','interpreter', 'latex')
    legend('measured', 'filtered', 'estimated')
    grid on
end

function plotJointAccelerations(pendubot)
figure
subplot(2,1,1)
    plot(pendubot.time, pendubot.shldr_acceleration)
    hold on
    plot(pendubot.time, pendubot.shldr_acceleration_filtered)
    plot(pendubot.time, pendubot.shldr_acceleration_filtered2)
    xlim([0 2])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$\ddot{q}_1$, rad/s$^2$','interpreter', 'latex')
    legend('estimate', 'filetered estimate')
    grid on
subplot(2,1,2)
    plot(pendubot.time, pendubot.elbow_acceleration)
    hold on
    plot(pendubot.time, pendubot.elbow_acceleration_filtered)
    plot(pendubot.time, pendubot.elbow_acceleration_filtered2)
    xlim([0 2])
    xlabel('$t$, sec','interpreter', 'latex')
    ylabel('$\ddot{q}_2$, rad/s$^2$','interpreter', 'latex')
    legend('estimate', 'filetered estimate')
    grid on
end

end