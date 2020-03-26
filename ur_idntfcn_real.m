% -----------------------------------------------------------------------
% In this script identification of inertial parameters of the UR10E
% is carried out. Two approaches are implemented: ordinary least squares
% and ordinary least squares with physical feasibility constraint.
% -----------------------------------------------------------------------
clc; clear all; close all;

% ------------------------------------------------------------------------
% Load raw data and procces it (filter and estimate accelerations).
% A lot of different trajectories were recorded for identificatio. Each of
% them result in slightly different dynamic parameters. Some of them
% describe the dynamics better than others.
% ------------------------------------------------------------------------

% idntfcnTrjctry = parseURData('ur-19_12_23_free.csv', 1, 2005);
% idntfcnTrjctry = parseURData('ur-20_01_31-unload.csv', 300, 2623);
% idntfcnTrjctry = parseURData('ur-20_02_06-20sec_5harm.csv', 545, 2417); 
% idntfcnTrjctry = parseURData('ur-20_02_10-20sec_7harm.csv', 544, 2500); 
% idntfcnTrjctry = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);
% idntfcnTrjctry = parseURData('ur-20_02_06-20sec_10harm.csv', 713, 2800); 
% idntfcnTrjctry = parseURData('ur-20_02_05-20sec_12harm.csv', 605, 2710);
% idntfcnTrjctry = parseURData('ur-20_02_10-30sec_12harm.csv', 635, 3510); 
% idntfcnTrjctry = parseURData('ur-20_02_12-40sec_12harm.csv', 500, 4460); 
% idntfcnTrjctry = parseURData('ur-20_02_12-50sec_12harm.csv', 355, 5090);

% idntfcnTrjctry = filterData(idntfcnTrjctry);


% idntfcnTrjctry{1} = parseURData('ur-19_12_23_free.csv', 1, 2005);
% idntfcnTrjctry{2} = parseURData('ur-20_01_31-unload.csv', 300, 2623);
% idntfcnTrjctry{3} = parseURData('ur-20_02_06-20sec_5harm.csv', 545, 2417);
% idntfcnTrjctry{4} = parseURData('ur-20_02_10-20sec_7harm.csv', 544, 2500); 
% idntfcnTrjctry{5} = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);
% idntfcnTrjctry{6} = parseURData('ur-20_02_06-20sec_10harm.csv', 713, 2800);
% idntfcnTrjctry{7} = parseURData('ur-20_02_05-20sec_12harm.csv', 605, 2710);
% idntfcnTrjctry{8} = parseURData('ur-20_02_10-30sec_12harm.csv', 635, 3510);
% idntfcnTrjctry{9} = parseURData('ur-20_02_12-40sec_12harm.csv', 500, 4460);
% idntfcnTrjctry{10} = parseURData('ur-20_02_12-50sec_12harm.csv', 355, 5090);

idntfcnTrjctry{1} = parseURData('ur-20_02_10-30sec_12harm.csv', 635, 3510);
idntfcnTrjctry{2} = parseURData('ur-20_02_12-40sec_12harm.csv', 500, 4460);
idntfcnTrjctry{3} = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);
idntfcnTrjctry{4} = parseURData('ur-20_02_12-50sec_12harm.csv', 355, 5090);

for i = 1:length(idntfcnTrjctry)
    idntfcnTrjctry{i} = filterData(idntfcnTrjctry{i});
end


%{
idntfcnTrjctry{1} = parseURData('ur-20_02_10-30sec_12harm.csv', 635, 3510);
idntfcnTrjctry{2} = parseURData('ur-20_02_12-40sec_12harm.csv', 500, 4460);
idntfcnTrjctry{3} = parseURData('ur-20_02_12-50sec_12harm.csv', 355, 5090);

idntfcnTrjctry{1} = filterData(idntfcnTrjctry{1});
idntfcnTrjctry{2} = filterData(idntfcnTrjctry{2});
idntfcnTrjctry{3} = filterData(idntfcnTrjctry{3});

idntfcnTrjctry{4}.t = [idntfcnTrjctry{1}.t;... 
                       idntfcnTrjctry{1}.t(end) + idntfcnTrjctry{2}.t;... 
                       idntfcnTrjctry{1}.t(end) + idntfcnTrjctry{2}.t(end) + idntfcnTrjctry{3}.t];
idntfcnTrjctry{4}.q = [idntfcnTrjctry{1}.q; 
                       idntfcnTrjctry{2}.q; 
                       idntfcnTrjctry{3}.q];
idntfcnTrjctry{4}.qd_fltrd = [idntfcnTrjctry{1}.qd_fltrd; 
                              idntfcnTrjctry{2}.qd_fltrd; 
                              idntfcnTrjctry{3}.qd_fltrd];
idntfcnTrjctry{4}.q2d_est = [idntfcnTrjctry{1}.q2d_est; 
                             idntfcnTrjctry{2}.q2d_est; 
                             idntfcnTrjctry{3}.q2d_est];
idntfcnTrjctry{4}.i_fltrd = [idntfcnTrjctry{1}.i_fltrd; 
                             idntfcnTrjctry{2}.i_fltrd; 
                             idntfcnTrjctry{3}.i_fltrd];
%}

% -------------------------------------------------------------------
% Generate Regressors based on data
% ------------------------------------------------------------------------
% Load matrices that map standard set of paratmers to base parameters
load('baseQR.mat'); % load mapping from full parameters to base parameters

% load identified drive gains
load('driveGains.mat')
% drvGains2 = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; % deLuca gains
% some comments about drive gains. Overall, identified gains results in
% better validation, than gains that are given in deLuca's work.

Tau = {}; Wb = {};

for i = 1:length(idntfcnTrjctry)
    [Tau{i}, Wb{i}] = buildObservationMatrices(idntfcnTrjctry{i}, baseQR, drvGains);
end

% Usual least squares
for i = 1:length(idntfcnTrjctry)
    [pib_OLS(:,i), pifrctn_OLS(:,i)] = ordinaryLeastSquareEstimation(Tau{i}, Wb{i});
end

% Set-up SDP optimization procedure
for i = 1:length(idntfcnTrjctry)
    [pib_SDP(:,i), pifrctn_SDP(:,i), pi_full] = physicallyConsistentEstimation(Tau{i}, Wb{i}, baseQR);
end



return
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


% 

%% Statisitical analysis
% unbiased estimation of the standard deviation
sqrd_sgma_e = norm(Tau_uldd - Wb_uldd*[pi_b; pi_frctn], 2)^2/...
                (size(Wb_uldd, 1) - size(Wb_uldd, 2));
            
% the covariance matrix of the estimation error
Cpi = sqrd_sgma_e*inv(Wb_uldd'*Wb_uldd);
sgma_pi = sqrt(diag(Cpi));

% relative standard deviation
sgma_pi_rltv = sgma_pi./abs([pi_b; pi_frctn]);


%% Functions
function [Tau, Wb] = buildObservationMatrices(idntfcnTrjctry, baseQR, drvGains)
% --------------------------------------------------------------------
% The function builds observation matrix for UR10E
% --------------------------------------------------------------------

E1 = baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);

Wb = []; Tau = []; 
for i = 1:1:length(idntfcnTrjctry.t)
     Yi = regressorWithMotorDynamics(idntfcnTrjctry.q(i,:)',...
                                          idntfcnTrjctry.qd_fltrd(i,:)',...
                                          idntfcnTrjctry.q2d_est(i,:)');
    Yfrctni = frictionRegressor(idntfcnTrjctry.qd_fltrd(i,:)');
    Ybi = [Yi*E1, Yfrctni];
    
    Wb = vertcat(Wb, Ybi);
    Tau = vertcat(Tau, diag(drvGains)*idntfcnTrjctry.i_fltrd(i,:)');
end
end



function [pib_OLS, pifrctn_OLS] = ordinaryLeastSquareEstimation(Tau, Wb)

pi_OLS = (Wb'*Wb)\(Wb'*Tau);

pib_OLS = pi_OLS(1:40); % variables for base paramters
pifrctn_OLS = pi_OLS(41:end);

end



function [pib_SDP, pifrctn_SDP, pi_full] = physicallyConsistentEstimation(Tau, Wb, baseQR)

physicalConsistency = 1;

pi_frctn = sdpvar(18,1); % variables for dependent parameters
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
errorRange = 0.10;
massUpperBound = massValuesURDF*(1 + errorRange);

cnstr = [];
for i = 1:6
    cnstr = [cnstr, pii(mass_indexes(i))> 0, ...
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
obj = norm(Tau - Wb*[pi_b; pi_frctn]);

% Solving sdp problem
sol2 = optimize(cnstr, obj, sdpsettings('solver','sdpt3'));

pib_SDP = value(pi_b); % variables for base paramters
pifrctn_SDP = value(pi_frctn);

pi_full = baseQR.permutationMatrix*[eye(baseQR.numberOfBaseParameters), ...
                                    -baseQR.beta; ...
                                    zeros(26,baseQR.numberOfBaseParameters), ... 
                                    eye(26) ]*[value(pi_b); value(pi_d)];

end