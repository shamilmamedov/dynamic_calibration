% ---------------------------------------------------------------------
% In this script trajectory optimization otherwise called experiment
% design for dynamic paramters identification is carried out. 
% ---------------------------------------------------------------------
run('main_ur.m'); % get robot description

% Getting limits on posistion and velocities. Moreover we get some
% constant parmeters of the robot that allow us to accelerate computation
% of the robot regressor.
q_min = zeros(6,1); q_max = zeros(6,1); qd_max = zeros(6,1);
for i = 1:6
    rot_axes(:,i) = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
    R_pj = RPY(str2num(ur10.robot.joint{i}.origin.Attributes.rpy));
    p_pj = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
    T_pj(:,:,i) = [R_pj, p_pj; zeros(1,3), 1];
    r_com(:,i) = str2num(ur10.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    
    q_min(i) = str2double(ur10.robot.joint{i}.limit.Attributes.lower);
    q_max(i) = str2double(ur10.robot.joint{i}.limit.Attributes.upper);
    qd_max(i) = str2double(ur10.robot.joint{i}.limit.Attributes.velocity);
end

ur10.rot_axes = rot_axes; % axis of rotation of each joint
ur10.T_pj = T_pj;
ur10.r_com = r_com;
ur10.qd_max = qd_max;
ur10.q2d_max = 2*ones(6,1);
% Use different limit for positions for safety
ur10.q_min = deg2rad([-180 -180 -90 -180 -90 -90]');
ur10.q_max = deg2rad([180 0 90 0 90 90]');
ur10.q0 = deg2rad([0 -90 0 -90 0 0 ]');

% Trajectory parameters
traj_par.T = 20;          % period of signal
traj_par.wf = 2*pi/traj_par.T;    % fundamental frequency
traj_par.t_smp = 1e-1;   % sampling time
traj_par.t = 0:traj_par.t_smp:traj_par.T;  % time
traj_par.N = 5;          % number of harmonics


%  ------------------------------------------------------------------------
% Otimization
% ------------------------------------------------------------------------
A = []; b = [];
Aeq = []; beq = [];
lb = []; ub = [];
x0 = rand(6*2*traj_par.N,1);

optns_pttrnSrch = optimoptions('patternsearch');
optns_pttrnSrch.Display = 'iter';

x = patternsearch(@(x)traj_cost_lgr(x,traj_par,ur10),x0,A,b,Aeq,beq,lb,ub,...
                  @(x)traj_cnstr(x,traj_par,ur10), optns_pttrnSrch);


return

options = optimoptions('ga');
options.Display = 'iter';
options.PlotFcn = 'gaplotbestf'; % {'gaplotbestf', 'gaplotscores'}
options.MaxGenerations = 50;
options.PopulationSize = 1e+3; % in each generation.
options.InitialPopulationRange = [-100; 100];
options.SelectionFcn = 'selectionroulette';

[x,fval,exitflag,output,population,scores] = ga(@(x)traj_cost(x,traj_par,ur10),...
        6*2*traj_par.N,A,b,Aeq,beq,lb,ub,@(x)traj_cnstr(x,traj_par,ur10),options);

%% ------------------------------------------------------------------------
% Verifying obtained trajectory
% ------------------------------------------------------------------------
ab = reshape(x,[12,traj_par.N]);
a = ab(1:6,:); % sin coeffs
b = ab(7:12,:); % cos coeffs
c_pol = pol_coeffs(traj_par.T,a,b,traj_par.wf,traj_par.N); % polynomial coeffs
[q,qd,q2d] = mixed_traj(traj_par.t,c_pol,a,b,traj_par.wf,traj_par.N);
    
traj_cnstr(x,traj_par,ur10)

figure
subplot(3,1,1)
    plot(traj_par.t,q)
    legend('q1','q2','q3','q4','q5','q6')
subplot(3,1,2)
    plot(traj_par.t,qd)
    legend('qd1','qd2','qd3','qd4','qd5','qd6')
subplot(3,1,3)
    plot(traj_par.t,q2d)
    legend('q2d1','q2d2','q2d3','q2d4','q2d5','q2d6')