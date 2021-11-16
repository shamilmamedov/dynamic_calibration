% ---------------------------------------------------------------------
% In this script trajectory optimization otherwise called experiment
% design for dynamic paramters identification is carried out. 
% 
% First, specify cost function (traj_cost_lgr) and constraints 
% (traj_cnstr) for the optimziation. Then choose oprimization algorithm and
% specify trajectory parameters (duration, fundamental frequency, number of 
% harmonics, initial (= final) positionin) and max/min positions,
% velocities and accelerations.
% 
% Then script runs optimization, plots obtained trajectory and saves its
% parameters into a file.
% ---------------------------------------------------------------------
% get robot description
path_to_urdf = 'ur10e.urdf';
ur10 = parse_urdf(path_to_urdf);

% get mapping from full parameters to base parameters
include_motor_dynamics = 1;
[~, baseQR] = base_params_qr(include_motor_dynamics);

% Choose optimization algorithm: 'patternsearch', 'ga'
optmznAlgorithm = 'patternsearch';

% Trajectory parameters
traj_par.T = 25;          % period of signal
traj_par.wf = 2*pi/traj_par.T;    % fundamental frequency
traj_par.t_smp = 2e-1;   % sampling time
traj_par.t = 0:traj_par.t_smp:traj_par.T;  % time
traj_par.N = 7;          % number of harmonics
traj_par.q0 = deg2rad([0 -90 0 -90 0 0 ]');
% Use different limit for positions for safety
traj_par.q_min = -deg2rad([180  180  100   180  90   90]');
traj_par.q_max =  deg2rad([180  0    100   0    90   90]');
traj_par.qd_max = qd_max;
traj_par.q2d_max = [2 1 1 1 1 2.5]';

%  ----------------------------------------------------------------------
% Otimization
% -----------------------------------------------------------------------
A = []; b = [];
Aeq = []; beq = [];
lb = []; ub = [];

if strcmp(optmznAlgorithm, 'patternsearch')
    x0 = rand(6*2*traj_par.N,1);
    optns_pttrnSrch = optimoptions('patternsearch');
    optns_pttrnSrch.Display = 'iter';
    optns_pttrnSrch.StepTolerance = 1e-1;
    optns_pttrnSrch.FunctionTolerance = 10;
    optns_pttrnSrch.ConstraintTolerance = 1e-6;
    optns_pttrnSrch.MaxTime = inf;
    optns_pttrnSrch.MaxFunctionEvaluations = 1e+6;
    
    [x,fval] = patternsearch(@(x)traj_cost_lgr(x,traj_par,baseQR), x0, ...
                             A, b, Aeq, beq, lb, ub, ...
                             @(x)traj_cnstr(x,traj_par), optns_pttrnSrch);
elseif strcmp(optmznAlgorithm, 'ga')
    optns_ga = optimoptions('ga');
    optns_ga.Display = 'iter';
    optns_ga.PlotFcn = 'gaplotbestf'; % {'gaplotbestf', 'gaplotscores'}
    optns_ga.MaxGenerations = 50;
    optns_ga.PopulationSize = 1e+3; % in each generation.
    optns_ga.InitialPopulationRange = [-100; 100];
    optns_ga.SelectionFcn = 'selectionroulette';

    [x,fval] = ga(@(x)traj_cost_lgr(x,traj_par,baseQR), 6*2*traj_par.N,...
                  A, b, Aeq, beq, lb, ub, ...
                  @(x)traj_cnstr(x,traj_par), optns_ga);
else
    error('Chosen algorithm is not found among implemented ones');
end

% ------------------------------------------------------------------------
% Plotting obtained trajectory
% ------------------------------------------------------------------------
ab = reshape(x,[12,traj_par.N]);
a = ab(1:6,:); % sin coeffs
b = ab(7:12,:); % cos coeffs
c_pol = getPolCoeffs(traj_par.T, a, b, traj_par.wf, traj_par.N, traj_par.q0);
[q,qd,q2d] = mixed_traj(traj_par.t, c_pol, a, b, traj_par.wf, traj_par.N);

figure
subplot(3,1,1)
    plot(traj_par.t,q)
    ylabel('$q$','interpreter','latex')
    grid on
    legend('q1','q2','q3','q4','q5','q6')
subplot(3,1,2)
    plot(traj_par.t,qd)
    ylabel('$\dot{q}$','interpreter','latex')
    grid on
    legend('qd1','qd2','qd3','qd4','qd5','qd6')
subplot(3,1,3)
    plot(traj_par.t,q2d)
    ylabel('$\ddot{q}$','interpreter','latex')
    grid on
    legend('q2d1','q2d2','q2d3','q2d4','q2d5','q2d6')

% ----------------------------------------------------------------------
% Saving parameters of the optimized trajectory
% ----------------------------------------------------------------------
% %{
pathToFolder = 'trajectory_optmzn/optimal_trjctrs/';
t1 = strcat('N',num2str(traj_par.N),'T',num2str(traj_par.T));
if strcmp(optmznAlgorithm, 'patternsearch')
    filename = strcat(pathToFolder,'ptrnSrch_',t1,'QR.mat');
elseif strcmp(optmznAlgorithm, 'ga')
    filename = strcat(pathToFolder,'ga_',t1,'.mat');
elseif strcmp(optmznAlgorithm, 'fmincon')
    filename = strcat(pathToFolder,'fmncn_',t1,'.mat');
end
save(filename,'a','b','c_pol','traj_par')
%}