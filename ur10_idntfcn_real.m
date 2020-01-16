clc; clear all; close all;

% ------------------------------------------------------------------------
% Load data and procces it (filter and estimate accelerations)
% ------------------------------------------------------------------------
run('data_prcsng.m')
% run('data_pltng.m')

% ------------------------------------------------------------------------
% Generate Regressors based on data
% ------------------------------------------------------------------------
% Load matrices that map standard set of paratmers to base parameters
% load('full2base_mapping.mat');
load('baseQR.mat'); % load mapping from full parameters to base parameters
E1 = baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);

% load identified drive gains
load('driveGains.mat')
% drvGains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; % deLuca gains

%Constracting regressor matrix
Wb_uldd = []; Tau_uldd = []; 
for i = 1:1:length(unloadedTrajectory.t)
     Y_ulddi = regressorWithMotorDynamics(unloadedTrajectory.q(i,:)',...
                                         unloadedTrajectory.qd_fltrd(i,:)',...
                                         unloadedTrajectory.q2d_est(i,:)');
    Yfrctni = frictionRegressor(unloadedTrajectory.qd_fltrd(i,:)');
    Ybi_uldd = [Y_ulddi*E1, Yfrctni];
    
    Wb_uldd = vertcat(Wb_uldd, Ybi_uldd);
    Tau_uldd = vertcat(Tau_uldd, diag(drvGains)*unloadedTrajectory.i_fltrd(i,:)');
end

%% Set-up SDP optimization procedure
physicalConsistency = 1;

pi_frctn = sdpvar(18,1);
pi_b = sdpvar(baseQR.numberOfBaseParameters,1); % variables for base paramters
pi_d = sdpvar(26,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = baseQR.permutationMatrix*[ eye(baseQR.numberOfBaseParameters), ...
                                -baseQR.beta; ...
                                zeros(26,baseQR.numberOfBaseParameters), ... 
                                eye(26) ]*[pi_b; pi_d];

% Feasibility contrraints of the link paramteres and rotor inertia
cnstr = [];

if physicalConsistency
    for i = 1:11:66
        link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];          
        frst_mmnt_i = pii(i+6:i+8);

        Di = [0.5*trace(link_inertia_i)*eye(3) - link_inertia_i, ...
                frst_mmnt_i; frst_mmnt_i', pii(i+9)];

        cnstr = [cnstr, Di>0, pii(i+10)>0];
    end
else
    for i = 1:11:66
        link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];

        frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));

        Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
        cnstr = [cnstr, Di>0, pii(i+10)>0];
    end
end

% Feasibility constraints on the friction prameters 
for i = 1:6
   cnstr = [cnstr, pi_frctn(3*i-2)>0, pi_frctn(3*i-1)>0];  
end

% Defining pbjective function
obj = norm(Tau_uldd - Wb_uldd*[pi_b; pi_frctn]);

% Solving sdp problem
sol2 = optimize(cnstr, obj, sdpsettings('solver','sdpt3'));

pi_frctn = value(pi_frctn);
pi_b = value(pi_b); % variables for base paramters


%% Statisitical analysis
% unbiased estimation of the standard deviation
sqrd_sgma_e = norm(Tau_uldd - Wb_uldd*[pi_b; pi_frctn], 2)^2/...
                (size(Wb_uldd, 1) - size(Wb_uldd, 2));
            
% the covariance matrix of the estimation error
Cpi = sqrd_sgma_e*inv(Wb_uldd'*Wb_uldd);
sgma_pi = sqrt(diag(Cpi));

% relative standard deviation
sgma_pi_rltv = sgma_pi./abs([pi_b; pi_frctn]);

