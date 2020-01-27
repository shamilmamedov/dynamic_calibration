clc; clear all; close all;

% Load trajectory and filter it
clsns = parseURData('ur-20_01_22-push_pose_2.csv', 1, 1700);
clsns = filterData(clsns);

%%
for i = 1:6
    figure
    plot(clsns.t, clsns.i_des_fltrd(:,i))
    hold on
    plot(clsns.t, clsns.i(:,i))
    xlabel('time, s')
    ylabel('current, A')
    grid on
end