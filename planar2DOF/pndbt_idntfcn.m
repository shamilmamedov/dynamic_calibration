clc; clear all; close all;

% get robot description
plnr = parse_urdf('planar_manip.urdf');

% load and process data
% pendubot = pendubotDataProcessing('step_A_1.57.mat');
% pendubot = pendubotDataProcessing('interp5_2.mat');
pendubot = pendubotDataProcessing('harmonic_A_0.7854_v_0.25.mat');

idntfcnData = {};
idntfcnData{1} = pendubotDataProcessing('harmonic_A_0.7854_v_0.5.mat');
idntfcnData{2} = pendubotDataProcessing('step_A_1.57.mat');
idntfcnData{3} = pendubotDataProcessing('interp5_0.5.mat');

% load mapping from standard parameters to base parameters
load('pndbtBaseQR.mat')
fullRegressor2BaseRegressor = pndbtBaseQR.permutationMatrix(:, ...
                                    1:pndbtBaseQR.numberOfBaseParameters);

Tau = {}; W = {};
[Tau{1}, ~, W{1}] = computeObservationMatricesForPendubot(idntfcnData{1}, fullRegressor2BaseRegressor);
[Tau{2}, ~, W{2}] = computeObservationMatricesForPendubot(idntfcnData{2}, fullRegressor2BaseRegressor);
[Tau{3}, ~, W{3}] = computeObservationMatricesForPendubot(idntfcnData{3}, fullRegressor2BaseRegressor);

% % Ordinary Least Squares Approach
% pi_hat_OLS = (Wb'*Wb)\(Wb'*Tau);

% Set-up SDP optimization procedure
pi_stnd = {}; pi_frctn = {};
[pi_stnd{1}, pi_frctn{1}] = OLS_SDP(Tau{1}, W{1}, pndbtBaseQR, plnr);
[pi_stnd{2}, pi_frctn{2}] = OLS_SDP(Tau{2}, W{2}, pndbtBaseQR, plnr);
[pi_stnd{3}, pi_frctn{3}] = OLS_SDP(Tau{3}, W{3}, pndbtBaseQR, plnr);


return
%%
pi_rgd_1 = pi_stnd{1}(1:10);
pi_rgd_2 = pi_stnd{1}(12:end);

m1 = pi_rgd_1(10);
com1 = pi_rgd_1(7:9)/m1;
link_inertia_1 = [pi_rgd_1(1), pi_rgd_1(2), pi_rgd_1(3); ...
                  pi_rgd_1(2), pi_rgd_1(4), pi_rgd_1(5); ...
                  pi_rgd_1(3), pi_rgd_1(5), pi_rgd_1(6)] + m1*vec2skewSymMat(com1)*vec2skewSymMat(com1) ;

m2 = pi_rgd_2(10);
com2 = pi_rgd_2(7:9)/m2;
link_inertia_2 = [pi_rgd_2(1), pi_rgd_2(2), pi_rgd_2(3); ...
                  pi_rgd_2(2), pi_rgd_2(4), pi_rgd_2(5); ...
                  pi_rgd_2(3), pi_rgd_2(5), pi_rgd_2(6)] + m2*vec2skewSymMat(com2)*vec2skewSymMat(com2);
              
%% Local Functions
function [pi_stnd, pi_frctn] = OLS_SDP(Tau, W, pndbtBaseQR, plnr)
    % It is possible to choose between physical consistency that was introduced
    % by Cortesao (generalized momentum is positive semidefinite) or physical
    % consistency that was introduced by Wensing and Slotine and then by
    % Cartesao. 
    % Then it is possible to assign realizability of the first and second
    % moments on the ellipsoid. For that more or less accurate CAD parameter
    % required.
    physicalConsistency = 1; % if 1 phsycal, if 0 then semiphysical
    firstMomentRealizability = 1; % DOESN"T WORK FOR NOW!!!!!!!!!
    secondMomentRealizability = 0;

    pi_frctn = sdpvar(6,1);
    pi_b = sdpvar(pndbtBaseQR.numberOfBaseParameters, 1); % variables for base paramters
    pi_d = sdpvar(15, 1); % variables for dependent paramters

    % Bijective mapping from [pi_b; pi_d] to standard parameters pi
    pii = pndbtBaseQR.permutationMatrix*[eye(pndbtBaseQR.numberOfBaseParameters), ...
                                        -pndbtBaseQR.beta; ...
                                        zeros(15, pndbtBaseQR.numberOfBaseParameters), ... 
                                        eye(15)]*[pi_b; pi_d];

    % Density realizability of the first momentum (in ellipsoid)
    % Ellipsoid is described by the equation
    % (x - xc)^2/xr^2 + (y - yc)^2/yr^2 + (z - zc)^2/zr^2 = 1 or 
    % (v - vc)'Qs^-1(v - vc), Qs = diag([xc^2 yc^2 zc^2])
    % where xs = [xc yc zc]' is vector defining center of ellipsoid, 
    % abc = [xr yr zr]' is semiaxis length
    xs(:,1) = [0.13 0 0]'; % center of the ellipsoid
    xs(:,2) = [0.13 0 0]';
    abc(:,1) = [0.13 0.05 0.005]'; % semiaxis length
    abc(:,2) = [0.13 0.01 0.005]';
    Qs(:,:,1) = diag(abc(:,1).^2); % matrix of reciprocals of the squares of the semi-axes
    Qs(:,:,2) = diag(abc(:,2).^2);


    % Density realizability of the second momentum (in an ellipsoid)
    sclng = 1.8;
    Sigma_c_1 = 0.5*trace(sclng*plnr.I(:,:,1))*eye(3) - sclng*plnr.I(:,:,1); % density weighted covariance
    Epsilon_pi_1 = inv(Sigma_c_1/plnr.m(1)); % covariance ellipsoid
    Q(:,:,1) = blkdiag(-Epsilon_pi_1, 1);

    Sigma_c_2 = 0.5*trace(sclng*plnr.I(:,:,2))*eye(3) - sclng*plnr.I(:,:,2); % density weighted covariance
    Epsilon_pi_2 = inv(Sigma_c_2/plnr.m(2)); % covariance ellipsoid
    Q(:,:,2) = blkdiag(-Epsilon_pi_2, 1);

    pi_CAD = [plnr.pi(:,1); 0; plnr.pi(:,2)]; % parameters from the CAD
    w_pi = 1e-6; % regulization term

    cnstr = [pii(10) < 2., pii(21) < 1.]; % constraints on the mass


    k = 1; % iterates over rigid bodies
    for i = 1:11:21
        second_moment_k = [pii(i),   pii(i+1), pii(i+2); ...
                           pii(i+1), pii(i+3), pii(i+4); ...
                           pii(i+2), pii(i+4), pii(i+5)]; 
        first_moment_k = pii(i+6:i+8);
        mass_k = pii(i+9);

        % Physical or semi-physical consistency 
        if  physicalConsistency
            Jk = [0.5*trace(second_moment_k)*eye(3) - second_moment_k, first_moment_k;
                  first_moment_k', mass_k];
            cnstr = [cnstr, Jk > 0];
        else
            Dk = [second_moment_k, vec2skewSymMat(first_moment_k)';
                  vec2skewSymMat(first_moment_k), mass_k*eye(3)];
            cnstr = [cnstr, Dk > 0];
        end

        % First moment realizability on the ellipsoid
        if firstMomentRealizability
            Ck = [mass_k, first_moment_k' - mass_k*xs(:,k)';
                  first_moment_k - mass_k*xs(:,k), mass_k*Qs(:,:,k)];
            cnstr = [cnstr, Ck > 0];
        end

        % Second moment realizability on the ellipsoid
        if secondMomentRealizability
            Pk = trace(Jk*Q(:,:,k));
            cnstr = [cnstr, Pk >= 0];
        end
        k = k + 1;
    end

    % first motor inertia constraint
    cnstr = [cnstr, pii(11) > 0]; 

    % Feasibility constraints on the friction prameters 
    % Columb and viscous friction coefficients are positive
    for i = 1:2
       cnstr = [cnstr, pi_frctn(3*i-2) > 0, pi_frctn(3*i-1) > 0];  
    end

    %!!!!!!!!!!!!!!!!
    % cnstr = [cnstr, pi_frctn(3) == 0, pi_frctn(6) == 0];

    % Defining pbjective function
    % obj = norm(Tau - Wb*[pi_b; pi_frctn], 2)^2 + w_pi*norm(pii - pi_CAD, 2)^2;
    obj = norm(Tau - W*[pii; pi_frctn], 2)^2 + w_pi*norm(pii - pi_CAD, 2)^2;

    % Solving sdp problem
    optns = sdpsettings;
    optns.solver = 'sdpt3';
    optns.verbose = 1;
    optns.showprogress = 1;
    diagnostics = optimize(cnstr, obj, optns);

    if diagnostics.problem ~= 0
        disp('Solver was not able to solve the problem')
        pi_frctn = [];
        pi_stnd = [];
        return
    end

    pi_b = value(pi_b); % variables for base paramters
    pi_frctn = value(pi_frctn);

    pi_stnd = pndbtBaseQR.permutationMatrix*[eye(pndbtBaseQR.numberOfBaseParameters), ...
                                            -pndbtBaseQR.beta; ...
                                            zeros(15,pndbtBaseQR.numberOfBaseParameters), ... 
                                            eye(15)]*[value(pi_b); value(pi_d)];
end

