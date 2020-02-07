clc; clear all; close all;

% ------------------------------------------------------------------------
% Load data and procces it (filter and estimate accelerations)
% ------------------------------------------------------------------------

% idntfcnTrjctry = parseURData('ur-19_12_23_free.csv', 1, 2005);
% idntfcnTrjctry = parseURData('ur-20_01_31-unload.csv', 300, 2623);
% idntfcnTrjctry = parseURData('ur-20_02_06-20sec_5harm.csv', 545, 2417); % 713
idntfcnTrjctry = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);
% idntfcnTrjctry = parseURData('ur-20_02_06-20sec_10harm.csv', 713, 2800); % 713
% idntfcnTrjctry = parseURData('ur-20_02_05-20sec_12harm.csv', 605, 2710);

idntfcnTrjctry = filterData(idntfcnTrjctry);

%{
idntfcnTrjctry1 = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);
idntfcnTrjctry2 = parseURData('ur-20_02_06-20sec_10harm.csv', 713, 2800);


idntfcnTrjctry1 = filterData(idntfcnTrjctry1);
idntfcnTrjctry2 = filterData(idntfcnTrjctry2);

idntfcnTrjctry3.t = [idntfcnTrjctry1.t; idntfcnTrjctry2.t + idntfcnTrjctry1.t(end)];
idntfcnTrjctry3.q = [idntfcnTrjctry1.q; idntfcnTrjctry2.q];
idntfcnTrjctry3.qd_fltrd = [idntfcnTrjctry1.qd_fltrd; idntfcnTrjctry2.qd_fltrd];
idntfcnTrjctry3.q2d_est = [idntfcnTrjctry1.q2d_est; idntfcnTrjctry2.q2d_est];
idntfcnTrjctry3.i_fltrd = [idntfcnTrjctry1.i_fltrd; idntfcnTrjctry2.i_fltrd];
%}

% -------------------------------------------------------------------
% Generate Regressors based on data
% ------------------------------------------------------------------------
% Load matrices that map standard set of paratmers to base parameters
% load('full2base_mapping.mat');
load('baseQR.mat'); % load mapping from full parameters to base parameters


% load identified drive gains
% load('driveGains.mat')
drvGains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; % deLuca gains


Tau = {}; Wb = {};

[Tau{1}, Wb{1}] = buildObservationMatrices(idntfcnTrjctry, baseQR, drvGains);
[Tau{2}, Wb{2}] = buildObservationMatrices(idntfcnTrjctry, baseQR, drvGains2);
[Tau{3}, Wb{3}] = buildObservationMatrices(idntfcnTrjctry, baseQR, drvGains3);

% Usual least squares
[pib_OLS(:,1), pifrctn_OLS(:,1)] = ordinaryLeastSquareEstimation(Tau{1}, Wb{1});
[pib_OLS(:,2), pifrctn_OLS(:,2)] = ordinaryLeastSquareEstimation(Tau{2}, Wb{2});
[pib_OLS(:,3), pifrctn_OLS(:,3)] = ordinaryLeastSquareEstimation(Tau{3}, Wb{3});

% Set-up SDP optimization procedure
[pib_SDP(:,1), pifrctn_SDP(:,1)] = physicallyConsistentEstimation(Tau{1}, Wb{1}, baseQR);
[pib_SDP(:,2), pifrctn_SDP(:,2)] = physicallyConsistentEstimation(Tau{2}, Wb{2}, baseQR);
[pib_SDP(:,3), pifrctn_SDP(:,3)] = physicallyConsistentEstimation(Tau{3}, Wb{3}, baseQR);

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



function [pib_SDP, pifrctn_SDP] = physicallyConsistentEstimation(Tau, Wb, baseQR)

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

end