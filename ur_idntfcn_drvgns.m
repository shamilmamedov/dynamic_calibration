% ------------------------------------------------------------------------
% The script performs drive gain identification of UR10E robot. 
% To do that several methods were used: total least squares approach; 
% ordinary least squares approach; and ordinary least squares with physical
% feasibility constraints that is solved using semidefinite programming
% -----------------------------------------------------------------------
clc; clear all; close all;

% ------------------------------------------------------------------------
% Load raw data and procces it (filter and estimate accelerations). 
% Several trajectories were recorded for unloaded and loaded cases. 
% Different combination of trajectories provide slighlty different results.
% Nonetheless, during validation they provide the more or less the same
% result. So, any unloaded trajectory from the given list can be chosen
% ------------------------------------------------------------------------
% unloadedTrajectory = parseURData('ur-20_02_12-50sec_12harm.csv', 355, 5090);
% unloadedTrajectory = parseURData('ur-20_01_31-unload.csv', 300, 2623);
unloadedTrajectory = parseURData('ur-20_02_19_14harm50sec.csv', 195, 4966);

unloadedTrajectory = filterData(unloadedTrajectory);

loadedTrajectory = parseURData('ur-20_02_19_14harm50secLoad.csv', 308, 5071);
loadedTrajectory = filterData(loadedTrajectory);

% ------------------------------------------------------------------------
% Generate Regressors based on data. 
% Here we generate base regressor, thta is obtained form the full regressor
% by multiplying it by the mapping from full standard paramters
% to base parametrs using numerical approach based on QR decomposition
% ------------------------------------------------------------------------
% Load matrices that map standard set of paratmers to base parameters
load('baseQR.mat'); % load mapping from full parameters to base parameters
E1 = baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);
m_load = 2.805; 

% Constracting regressor matrix for unloaded case
Wb_uldd = []; I_uldd = []; 
for i = 1:1:length(unloadedTrajectory.t)
    Y_ulddi = regressorWithMotorDynamics(unloadedTrajectory.q(i,:)',...
                                         unloadedTrajectory.qd_fltrd(i,:)',...
                                         unloadedTrajectory.q2d_est(i,:)');
                                     
    Yfrctni = frictionRegressor(unloadedTrajectory.qd_fltrd(i,:)');
    Ybi_uldd = [Y_ulddi*E1, Yfrctni];
    
    Wb_uldd = vertcat(Wb_uldd, Ybi_uldd);
    I_uldd = vertcat(I_uldd, diag(unloadedTrajectory.i_fltrd(i,:)));
end

% Constracting regressor matrix for loaded case
Wb_ldd = []; Wl = []; I_ldd = [];
for i = 1:1:length(loadedTrajectory.t)
    Y_lddi = regressorWithMotorDynamics(loadedTrajectory.q(i,:)',...
                                        loadedTrajectory.qd_fltrd(i,:)',...
                                        loadedTrajectory.q2d_est(i,:)');
                                    
    Yfrctni = frictionRegressor(loadedTrajectory.qd_fltrd(i,:)');
    Ybi_ldd = [Y_lddi*E1, Yfrctni];
    
    Yli = load_regressor_UR10E(loadedTrajectory.q(i,:)',...
                               loadedTrajectory.qd_fltrd(i,:)',...
                               loadedTrajectory.q2d_est(i,:)');
                           
    Wb_ldd = vertcat(Wb_ldd, Ybi_ldd);
    Wl = vertcat(Wl,Yli); 
    I_ldd = vertcat(I_ldd, diag(loadedTrajectory.i_fltrd(i,:)));
end
Wl_uknown = Wl(:,1:9);
Wl_known = Wl(:,10); % mass of the load is known 


%% Using total least squares
% TLS provides rather bad results. Even normilizing by mass and using
% weighting does not help to improve results
Wb_tls = [I_uldd   -Wb_uldd   zeros(size(I_uldd,1), size(Wl,2));
          I_ldd    -Wb_ldd    -Wl_uknown    -Wl_known*m_load];

% SVD decompostion of Wb_tls to solve total least squares
[~,~,V] = svd(Wb_tls,'econ');
% Scaling of the solution
lmda = 1/V(end,end);
pi_tls = lmda*V(:,end);
% drive gains
drvGainsTLS1 = pi_tls(1:6)

% Finding weighting matrix, joint by joint
G = zeros(6);
for i = 1:6
    Wib_tls = Wb_tls(i:6:end,:);
    [~,Si,Vi] = svd(Wib_tls,'econ');
    sgmai = Si(end,end)/sqrt((size(Wib_tls,1)-rank(Wib_tls)));
    G(i,i) = 1/sgmai^2;
end
% Weighting observation matrix
for i = 1:6:size(Wb_tls,1)
    Wb_tls(i:i+5,:) = G*Wb_tls(i:i+5,:);
end
[~,~,V] = svd(Wb_tls,'econ');
lmda = 1/V(end,end);
pi_tls = lmda*V(:,end);
drvGainsTLS2 = pi_tls(1:6)


%% Identification of parameters including drive gains
% Although according to Handbook of robotics and papers of Gautier odinary
% least squares has correlated noise for correct drive gain estimation
% it provides good results. Weighted least square does not improve the
% result.
Wb_ls = [I_uldd     -Wb_uldd    zeros(size(I_uldd,1), size(Wl_uknown,2));
         I_ldd      -Wb_ldd     -Wl_uknown];
     
Yb_ts = [zeros(size(I_uldd,1),1); Wl_known*m_load];

% Compute least squares solution
pi_ls = ((Wb_ls'*Wb_ls)\Wb_ls')*Yb_ts;
drvGainsOLS1 = pi_ls(1:6)


G = zeros(6);
for i = 1:6
    Wib_ls = Wb_ls(i:6:end,:);
    Yib_ls = Yb_ts(i:6:end);
    sgmai_sqrd = norm(Yib_ls - Wib_ls*pi_ls,2)^2/(size(Wib_ls,1)-rank(Wib_ls));
    G(i,i) = 1/sqrt(sgmai_sqrd);
end
G = diag([0.05 1 1 1 1 1]);

for i = 1:6:size(Wb_ls,1)
    Wb_ls(i:i+5,:) = G*Wb_ls(i:i+5,:);
    Yb_ts(i:i+5) = G*Yb_ts(i:i+5);
end
pi_tot = ((Wb_ls'*Wb_ls)\Wb_ls')*Yb_ts;
drvGainsOLS2 = pi_tot(1:6)


%% Set-up SDP optimization procedure
% Provides more or less the same result as OLS but with physical
% consistencty. The drive gains estimated using this appraoch is further
% used for indetification of inertial parameters
drv_gns = sdpvar(6,1); % variables for base paramters
pi_load_unknw = sdpvar(9,1); % varaibles for unknown load paramters
pi_frctn = sdpvar(18,1);
pi_b = sdpvar(baseQR.numberOfBaseParameters,1); % variables for base paramters
pi_d = sdpvar(26,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = baseQR.permutationMatrix*[ eye(baseQR.numberOfBaseParameters), ...
                                -baseQR.beta; ...
                                zeros(26,baseQR.numberOfBaseParameters), ... 
                                eye(26) ]*[pi_b; pi_d];

% Feasibility contrraints of the link paramteres and rotor inertia
cnstr = [drv_gns(1)>10]; % strong constraint on minimum value of K1
for i = 1:11:66
    link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                      pii(i+1), pii(i+3), pii(i+4); ...
                      pii(i+2), pii(i+4), pii(i+5)];
                  
    frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));
    
    Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
    cnstr = [cnstr, Di>0, pii(i+10)>0];
end

% Feasibility constraints on the load paramters
load_inertia = [pi_load_unknw(1), pi_load_unknw(2), pi_load_unknw(3); ...
                pi_load_unknw(2), pi_load_unknw(4), pi_load_unknw(5); ...
                pi_load_unknw(3), pi_load_unknw(5), pi_load_unknw(6)];                  
load_frst_mmnt = vec2skewSymMat(pi_load_unknw(7:9));    
Dl = [load_inertia, load_frst_mmnt'; load_frst_mmnt, m_load*eye(3)];

cnstr = [cnstr, Dl>0];

% Feasibility constraints on the friction prameters 
for i = 1:6
   cnstr = [cnstr, pi_frctn(3*i-2)>0, pi_frctn(3*i-1)>0];  
end

% Defining objective function
t1 = [zeros(size(I_uldd,1),1); -Wl(:,end)*m_load];

t2 = [-I_uldd, Wb_uldd, zeros(size(Wb_uldd,1), size(Wl,2)-1); ...
      -I_ldd, Wb_ldd, Wl(:,1:9) ];
  
obj = norm(t1 - t2*[drv_gns; pi_b; pi_frctn; pi_load_unknw]);

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));

% Getting values of the estimated patamters
drvGainsSDP = value(drv_gns)

%% Saving obtained drive gains
drvGains = drvGainsSDP;
filename = 'driveGains.mat';
save(filename,'drvGains')