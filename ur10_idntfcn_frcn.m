% -------------------------------------------------------------------
% Trying to identify complex friction model
% -------------------------------------------------------------------
% clc; clear all; close all;

link1 = parseURData('ur-20_01_14-link_1.csv', 297, 2162);
link1 = filterData(link1);

link2 = parseURData('ur-20_01_14-link_2.csv', 114, 2168);
link2 = filterData(link2);

link3 = parseURData('ur-20_01_14-link_3.csv', 174, 2050);
link3 = filterData(link3);

link4 = parseURData('ur-20_01_14-link_4.csv', 161, 2208);
link4 = filterData(link4);

link5 = parseURData('ur-20_01_14-link_5.csv', 403, 2264);
link5 = filterData(link5);

link6 = parseURData('ur-20_01_14-link_6.csv', 362, 2230);
link6 = filterData(link6);


% Estimating torques without friction model
link1.Tau_est = torquesWithoutFriction(link1, E1, pi_b);
link2.Tau_est = torquesWithoutFriction(link2, E1, pi_b);
link3.Tau_est = torquesWithoutFriction(link3, E1, pi_b);
link4.Tau_est = torquesWithoutFriction(link4, E1, pi_b);
link5.Tau_est = torquesWithoutFriction(link5, E1, pi_b);
link6.Tau_est = torquesWithoutFriction(link6, E1, pi_b);


% Finding difference between estiamted and measured torques
link1.deltaTau = link1.i_fltrd(:,1)*drvGains(1) - link1.Tau_est(:,1);
link2.deltaTau = link2.i_fltrd(:,2)*drvGains(2) - link2.Tau_est(:,2);
link3.deltaTau = link3.i_fltrd(:,3)*drvGains(3) - link3.Tau_est(:,3);
link4.deltaTau = link4.i_fltrd(:,4)*drvGains(4) - link4.Tau_est(:,4);
link5.deltaTau = link5.i_fltrd(:,5)*drvGains(5) - link5.Tau_est(:,5);
link6.deltaTau = link6.i_fltrd(:,6)*drvGains(6) - link6.Tau_est(:,6);


%% Identify nonlinear friction model
fminsrchOptns = optimset('Display','iter', 'MaxFunEvals', 5e+4, ...
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
x5 = fminsearch(@(x)costFcn(x, link5.qd_fltrd(:,5), link5.deltaTau), x50, fminsrchOptns);
x6 = fminsearch(@(x)costFcn(x, link6.qd_fltrd(:,6), link6.deltaTau), x5, fminsrchOptns);


%% Plot real friction torque vs estimated with nonlinear model

plotFrictionModels(link1, 1, pi_frctn(1:3), x1)
plotFrictionModels(link2, 2, pi_frctn(4:6), x2)
plotFrictionModels(link3, 3, pi_frctn(7:9), x3)
plotFrictionModels(link4, 4, pi_frctn(10:12), x4)
plotFrictionModels(link5, 5, pi_frctn(13:15), x5)
plotFrictionModels(link6, 6, pi_frctn(16:18), x6)


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
    plot(link.t, link.Tau_est(:,noLink) + simpleFrictionModel(linear, link.qd_fltrd(:,noLink)))
    plot(link.t, link.i_fltrd(:,noLink)*drvGain)
    legend('nonlinear friction', 'linear friction', 'real')
    title(strcat('Link ',num2str(noLink),' torque estimation'))
end


function plotFrictionModels(link, noLink, linear, nonlinear)
    figure
    hold on
    scatter(link.qd_fltrd(:,noLink), link.deltaTau, '.')
    plot(link.qd_fltrd(:,noLink), nonlinearFrictionModel(nonlinear, link.qd_fltrd(:,noLink)))
    plot(link.qd_fltrd(:,noLink), simpleFrictionModel(linear, link.qd_fltrd(:,noLink)))
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


function out = parseURData(file, int_idx, fnl_idx)
    traj = load(file);

    out = struct;
    out.t = traj(int_idx:fnl_idx,1) - traj(int_idx,1);
    out.q = traj(int_idx:fnl_idx,2:7);
    out.qd = traj(int_idx:fnl_idx,8:13);
    out.i = traj(int_idx:fnl_idx,14:19);
end


function f = costFcn(a, qd, deltaTau)
    f = norm(nonlinearFrictionModel(a, qd) - deltaTau, 2);
end


function f = simpleFrictionModel(a, qd)
    f = a(1)*qd + a(2)*sign(qd) + a(3);
end


function f = nonlinearFrictionModel(a, qd)
    f = a(1)*qd + a(2) + a(3)./( 1 + exp(-a(4).*(qd + a(5)) ) );
end


function data = filterData(data)
    % ------------------------------------------------------------------------
    % Filtering Velocities
    % ------------------------------------------------------------------------
    % Design filter
    vel_filt = designfilt('lowpassiir','FilterOrder',3, ...
            'HalfPowerFrequency',0.2,'DesignMethod','butter');

    data.qd_fltrd = zeros(size(data.qd));
    for i = 1:6
        data.qd_fltrd(:,i) = filtfilt(vel_filt,data.qd(:,i));
    end

    % ------------------------------------------------------------------------
    % Estimating accelerations
    % ------------------------------------------------------------------------
    % Three point central difference
    data.q2d_est = zeros(size(data.qd_fltrd));
    for i = 2:length(data.qd_fltrd)-1
       dlta_qd_fltrd =  data.qd_fltrd(i+1,:) -  ...
                            data.qd_fltrd(i-1,:);
       dlta_t_msrd = data.t(i+1) - data.t(i-1);
       data.q2d_est(i,:) = dlta_qd_fltrd/dlta_t_msrd;
    end

    % Zeros phase filtering acceleration obtained by finite difference
    accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
            'HalfPowerFrequency',0.05,'DesignMethod','butter');
    for i = 1:6
        data.q2d_est(:,i) = filtfilt(accel_filt,data.q2d_est(:,i));
    end

    % ------------------------------------------------------------------------
    % Filtering current
    % ------------------------------------------------------------------------
    % Zeros phase filtering acceleration obtained by finite difference
    curr_filt = designfilt('lowpassiir','FilterOrder',5, ...
            'HalfPowerFrequency',0.1,'DesignMethod','butter');

    for i = 1:6
        data.i_fltrd(:,i) = filtfilt(curr_filt,data.i(:,i));
    end
end
