function sol = estimate_dynamic_params(path_to_data, idx, drvGains, baseQR, method)
% -----------------------------------------------------------------------
% In this script identification of inertial parameters of the UR10E
% is carried out. Two approaches are implemented: ordinary least squares
% and ordinary least squares with physical feasibility constraint.
% Moreover, statistical analysis of the estimated parmaeters is performed
%
% Inputs: 
%   path_to_data - measured data from the robot
%   idx - specifies indeces for data to be used in estimation. Used
%         to remove garbage data
%   drvGains - drive gains
%   baseQR - QR decomposition of the observation matrix used for 
%            calculatung base regressor
%   method - method for estimation. Could be OLS or PC-OLS
%
% Ouputs:
%   sol - structre with the estimate and its statistical abalysis
%   sol.pi_b - estimated base parameters
%   sol.pi_fr - estimated friction parameters
%   sol.pi_s - estimated standard parameters only if PC-OLS was used
%   sol.std - standard deviation of the estimated parameters
%   sol.rel_std - relative standatrd deviation of the estimated parameters
% -----------------------------------------------------------------------

% ------------------------------------------------------------------------
% Load raw data and procces it (filter and estimate accelerations).
% A lot of different trajectories were recorded for identificatio. Each of
% them result in slightly different dynamic parameters. Some of them
% describe the dynamics better than others.
% ------------------------------------------------------------------------
idntfcnTrjctry = parseURData(path_to_data, idx(1), idx(2));
idntfcnTrjctry = filterData(idntfcnTrjctry);

% -------------------------------------------------------------------
% Generate Regressors based on data
% ------------------------------------------------------------------------
[Tau, Wb] = buildObservationMatrices(idntfcnTrjctry, baseQR, drvGains);

% ---------------------------------------------------------------------
% Estimate parameters
% ---------------------------------------------------------------------
sol = struct;
if strcmp(method, 'OLS')
    % Usual least squares
    [sol.pi_b, sol.pi_fr] = ordinaryLeastSquareEstimation(Tau, Wb);
elseif strcmp(method, 'PC-OLS')
    % Physically consistent OLS using SDP optimization
    [sol.pi_b, sol.pi_fr, sol.pi_s] = physicallyConsistentEstimation(Tau, Wb, baseQR);
else
    error("Chosen method for dynamic parameter estimation does not exist");
end

% ------------------------------------------------------------------
% Statistical analysis
% -----------------------------------------------------------------
% unbiased estimation of the standard deviation
sqrd_sgma_e = norm(Tau - Wb*[sol.pi_b; sol.pi_fr], 2)^2/...
                (size(Wb, 1) - size(Wb, 2));
            
% the covariance matrix of the estimation error
Cpi = sqrd_sgma_e*inv(Wb'*Wb);
sol.std = sqrt(diag(Cpi));

% relative standard deviation
sol.rel_std = 100*sol.std./abs([sol.pi_b; sol.pi_fr]);
end


% Local unctions
function [Tau, Wb] = buildObservationMatrices(idntfcnTrjctry, baseQR, drvGains)
    % The function builds observation matrix for UR10E
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
    % Function perfroms ordinary least squares estimation of parameters
    pi_OLS = (Wb'*Wb)\(Wb'*Tau);

    pib_OLS = pi_OLS(1:40); % variables for base paramters
    pifrctn_OLS = pi_OLS(41:end);
end


function [pib_SDP, pifrctn_SDP, pi_full] = physicallyConsistentEstimation(Tau, Wb, baseQR)
    % Function estimation physically consistent parameters
    % Ideally the user can choose between physical consistency and 
    % so called semi-physical consistency, but right now it is hardcoded
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
                                        eye(26)]*[value(pi_b); value(pi_d)];
end
