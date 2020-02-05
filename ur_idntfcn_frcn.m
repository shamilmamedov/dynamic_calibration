% -------------------------------------------------------------------
% Trying to identify complex friction model
% -------------------------------------------------------------------
% clc; clear all; close all;

link1 = parseURData('ur-20_01_22-joint_1.csv', 1, 5568);
link1 = filterData(link1);

link2 = parseURData('ur-20_01_22-joint_2.csv', 1, 5000);
link2 = filterData(link2);

link3 = parseURData('ur-20_01_22-joint_3.csv', 1, 5800);
link3 = filterData(link3);

link4 = parseURData('ur-20_01_22-joint_4.csv', 1, 6000);
link4 = filterData(link4);

link5 = parseURData('ur-20_01_22-joint_5.csv', 485, 5500);
link5 = filterData(link5);

link6 = parseURData('ur-20_01_22-joint_6.csv', 371, 6000);
link6 = filterData(link6);

% Estimating torques without friction model
link1.Tau_est = torquesWithoutFriction(link1, E1, pi_b);
link2.Tau_est = torquesWithoutFriction(link2, E1, pi_b);
link3.Tau_est = torquesWithoutFriction(link3, E1, pi_b);
link4.Tau_est = torquesWithoutFriction(link4, E1, pi_b);
link5.Tau_est = torquesWithoutFriction(link5, E1, pi_b);
link6.Tau_est = torquesWithoutFriction(link6, E1, pi_b);

% Estimating friction using linear friction model
link1.Tau_frctn = linearFrictionModel(pi_frctn(1:3), link1.qd(:,1));
link2.Tau_frctn = linearFrictionModel(pi_frctn(4:6), link2.qd(:,2));
link3.Tau_frctn = linearFrictionModel(pi_frctn(7:9), link3.qd(:,3));
link4.Tau_frctn = linearFrictionModel(pi_frctn(10:12), link4.qd(:,4));
link5.Tau_frctn = linearFrictionModel(pi_frctn(13:15), link5.qd(:,5));
link6.Tau_frctn = linearFrictionModel(pi_frctn(16:18), link6.qd(:,6));

% % Finding difference between estiamted and measured torques
link1.deltaTau = link1.i_fltrd(:,1)*drvGains(1) - link1.Tau_est(:,1);
link2.deltaTau = link2.i_fltrd(:,2)*drvGains(2) - link2.Tau_est(:,2);
link3.deltaTau = link3.i_fltrd(:,3)*drvGains(3) - link3.Tau_est(:,3);
link4.deltaTau = link4.i_fltrd(:,4)*drvGains(4) - link4.Tau_est(:,4);
link5.deltaTau = link5.i_fltrd(:,5)*drvGains(5) - link5.Tau_est(:,5);
link6.deltaTau = link6.i_fltrd(:,6)*drvGains(6) - link6.Tau_est(:,6);


%% Identify nonlinear friction model
fminsrchOptns = optimset('Display','final', 'MaxFunEvals', 5e+4, ...
                         'TolFun', 1e-8, 'TolX', 1e-8, 'MaxIter', 5e+4);

x10 = [pi_frctn(1); pi_frctn(3); ones(3,1)];
x20 = [pi_frctn(4); pi_frctn(6); ones(3,1)];
x30 = [pi_frctn(7); pi_frctn(9); ones(3,1)];
x40 = [pi_frctn(10); pi_frctn(12); ones(3,1)];
x50 = [pi_frctn(13); pi_frctn(15); ones(3,1)];
x60 = [pi_frctn(16); pi_frctn(18); 2*ones(3,1)];

x1 = fminsearch(@(x)costFcn(x, link1.qd_fltrd(:,1), link1.deltaTau), x10, fminsrchOptns);
x2 = fminsearch(@(x)costFcn(x, link2.qd_fltrd(:,2), link2.deltaTau), x20, fminsrchOptns);
x3 = fminsearch(@(x)costFcn(x, link3.qd_fltrd(:,3), link3.deltaTau), x30, fminsrchOptns);
x4 = fminsearch(@(x)costFcn(x, link4.qd_fltrd(:,4), link4.deltaTau), x40, fminsrchOptns);
x5 = fminsearch(@(x)costFcn(x, link5.qd_fltrd(:,5), link5.deltaTau), x4, fminsrchOptns);
x6 = fminsearch(@(x)costFcn(x, link6.qd_fltrd(:,6), link6.deltaTau), x5, fminsrchOptns);

pi_nonlnr_frcn = [x1; x2; x3; x4; x5; x6];

identifiedUR10E.nonlinearFrictionParameters = pi_nonlnr_frcn;

%% Plot real friction torque vs estimated with nonlinear model

plotFrictionModels(link1, 1, pi_frctn(1:3), x1)
plotFrictionModels(link2, 2, pi_frctn(4:6), x2)
plotFrictionModels(link3, 3, pi_frctn(7:9), x3)
plotFrictionModels(link4, 4, pi_frctn(10:12), x4)
plotFrictionModels(link5, 5, pi_frctn(13:15), x5)
plotFrictionModels(link6, 6, pi_frctn(16:18), x6)

return
%% Plot torque prediction with different friction models
plotTroqueEstimation(link1, 1, drvGains(1), pi_frctn(1:3), x1)
plotTroqueEstimation(link2, 2, drvGains(2), pi_frctn(4:6), x2)
plotTroqueEstimation(link3, 3, drvGains(3), pi_frctn(7:9), x3)
plotTroqueEstimation(link4, 4, drvGains(4), pi_frctn(10:12), x4)
plotTroqueEstimation(link5, 5, drvGains(5), pi_frctn(13:15), x5)
plotTroqueEstimation(link6, 6, drvGains(6), pi_frctn(16:18), x6)


%% Utilized Functions

function plotTroqueEstimation(link, noLink, drvGain, linear, nonlinear)
    figure
    hold on
    plot(link.t, link.Tau_est(:,noLink) + nonlinearFrictionModel(nonlinear, link.qd_fltrd(:,noLink)))
    plot(link.t, link.Tau_est(:,noLink) + linearFrictionModel(linear, link.qd_fltrd(:,noLink)))
    plot(link.t, link.i_fltrd(:,noLink)*drvGain)
    legend('nonlinear friction', 'linear friction', 'real')
    title(strcat('Link ',num2str(noLink),' torque estimation'))
end


function plotFrictionModels(link, noLink, linear, nonlinear)
    figure
    hold on
    scatter(link.qd(:,noLink), link.deltaTau, '.')
    plot(link.qd(:,noLink), nonlinearFrictionModel(nonlinear, link.qd(:,noLink)))
    plot(link.qd(:,noLink), linearFrictionModel(linear, link.qd(:,noLink)))
    legend('real', 'nonlinear model', 'linear model','Location','northwest')
    title(strcat('Link ',num2str(noLink),' friction model'))
end


function Tau_est = torquesWithoutFriction(link, E1, pi_b)
    Tau_est = [];
    for i = 1:1:length(link.t)
         Y_ulddi = regressorWithMotorDynamics(link.q(i,:)',...
                                              link.qd_fltrd(i,:)',...
                                              link.q2d_est(i,:)');
         Ybi_uldd = Y_ulddi*E1;
         taui_est = Ybi_uldd*pi_b;
         Tau_est = vertcat(Tau_est,taui_est');
    end
end


function f = costFcn(a, qd, deltaTau)
    f = norm(nonlinearFrictionModel(a, qd) - deltaTau, 2);
end









