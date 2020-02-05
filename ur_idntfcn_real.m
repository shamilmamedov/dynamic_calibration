clc; clear all; close all;

% ------------------------------------------------------------------------
% Load data and procces it (filter and estimate accelerations)
% ------------------------------------------------------------------------
unloadedTrajectory = parseURData('ur-19_12_23_free.csv', 1, 2005);
% unloadedTrajectory = parseURData('ur-20_01_31-unload.csv', 300, 2623);
unloadedTrajectory = filterData(unloadedTrajectory);

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
% drvGains = [11.1272; 11.83; 9.53; 12.64; 10.24; 5.53];
% drvGains = [15.87; 11.83; 9.53; 11.92; 15.24; 18.47];

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

%% Usual least squares
pib_hat = (Wb_uldd'*Wb_uldd)\(Wb_uldd'*Tau_uldd);

pi_b = pib_hat(1:40); % variables for base paramters
pi_frctn = pib_hat(41:end);




%% Set-up SDP optimization procedure
physicalConsistency = 1;

pi_frctn = sdpvar(18,1);
pi_b = sdpvar(baseQR.numberOfBaseParameters,1); % variables for base paramters
pi_d = sdpvar(26,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = baseQR.permutationMatrix*[eye(baseQR.numberOfBaseParameters), ...
                                -baseQR.beta; ...
                                zeros(26,baseQR.numberOfBaseParameters), ... 
                                eye(26) ]*[pi_b; pi_d];

% Feasibility contrraints of the link paramteres and rotor inertia
mass_indexes = 10:11:66;
massValuesURDF = [7.778 12.93 3.87 1.96 1.96 0.202]';
errorRange = 0.25;
massUpperBound = massValuesURDF*(1 + errorRange);
massLowerBound = massValuesURDF*(1 - errorRange);

cnstr = [];
for i = 1:6
    cnstr = [cnstr, pii(mass_indexes(i))> massLowerBound(i), ...
                pii(mass_indexes(i)) < massUpperBound(i)];
        
end



if physicalConsistency
    for i = 1:11:66
        link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];          
        frst_mmnt_i = pii(i+6:i+8);

        Di = [0.5*trace(link_inertia_i)*eye(3) - link_inertia_i, ...
                frst_mmnt_i; frst_mmnt_i', pii(i+9)];

        cnstr = [cnstr, Di>0, pii(i+10)>0,  pii(i+9)<15];
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


%% Saving identified parameters
pi_full = baseQR.permutationMatrix*[eye(baseQR.numberOfBaseParameters), ...
                                    -baseQR.beta; ...
                                    zeros(26,baseQR.numberOfBaseParameters), ... 
                                    eye(26) ]*[value(pi_b); value(pi_d)];
t1 = reshape(pi_full, [11, 6]);
                                
identifiedUR10E = struct;
identifiedUR10E.baseParameters = pi_b;
identifiedUR10E.standardParameters = pi_full;
identifiedUR10E.linearFrictionParameters = pi_frctn;



%% Statisitical analysis
% unbiased estimation of the standard deviation
sqrd_sgma_e = norm(Tau_uldd - Wb_uldd*[pi_b; pi_frctn], 2)^2/...
                (size(Wb_uldd, 1) - size(Wb_uldd, 2));
            
% the covariance matrix of the estimation error
Cpi = sqrd_sgma_e*inv(Wb_uldd'*Wb_uldd);
sgma_pi = sqrt(diag(Cpi));

% relative standard deviation
sgma_pi_rltv = sgma_pi./abs([pi_b; pi_frctn]);

