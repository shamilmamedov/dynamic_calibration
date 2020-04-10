clc; clear all; close all;

% get robot description
plnr = parse_urdf('planar_manip.urdf');

% load and process data
% pendubot = pendubotDataProcessing('step_A_1.57.mat');
pendubot = pendubotDataProcessing('interp5_2.mat');

% load mapping from standard parameters to base parameters
load('pndbtBaseQR.mat')
fullRegressor2BaseRegressor = pndbtBaseQR.permutationMatrix(:, ...
                                    1:pndbtBaseQR.numberOfBaseParameters);

% compose observation matrix and torque vector
noObservations = length(pendubot.time);
W = []; Wb = []; Tau = [];
for i = 1:1:noObservations
    % generalized positions velocities and accelerations
    qi = [pendubot.shldr_position(i), pendubot.elbw_position(i)]';
    qdi = [pendubot.shldr_velocity_filtered(i), pendubot.elbw_velocity_filtered(i)]';
    q2di = [pendubot.shldr_acceleration_filtered(i), pendubot.elbow_acceleration_filtered(i)]';
    
    % regressor
    Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    
    % get base regressor from full regressor
    Ybi = Yi*fullRegressor2BaseRegressor;
    
    % friction regressor
    Yfrctni = frictionRegressor(qdi);
    
    % compose base observation matrix
    Wb = vertcat(Wb, [Ybi, Yfrctni]);
    
    % compose full observation matrix
    W = vertcat(W, [Yi, Yfrctni]);
    
    % compose torque vector
    taui = [pendubot.torque_filtered(i), 0]';
%     taui = [pendubot.current(i)*0.123, 0]';
    Tau = vertcat(Tau, taui);
end


%% Usual Least Squares Approach
pi_hat_OLS = (Wb'*Wb)\(Wb'*Tau);


%% Set-up SDP optimization procedure
% It is possible to choose between physical consistency that was introduced
% by Cortesao (generalized momentum is positive semidefinite) or physical
% consistency that was introduced by Wensing and Slotine and then by
% Cartesao. 
% Then it is possible to assign realizability of the first and second
% moments on the ellipsoid. For that more or less accurate CAD parameter
% required.
physicalConsistency = 1; % if 1 phsycal, if 0 then semiphysical
firstMomentRealizability = 0; % DOESN"T WORK FOR NOW!!!!!!!!!
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
xs(:,1) = [0.15 0 0]'; % center of the ellipsoid
xs(:,2) = [0.15 0 0]';
abc(:,1) = [0.13 0.01 0.005]'; % semiaxis length
abc(:,2) = [0.13 0.01 0.005]';
Qs(:,:,1) = diag(abc(:,1).^2); % matrix of reciprocals of the squares of the semi-axes
Qs(:,:,2) = diag(abc(:,2).^2);


% Density realizability of the second momentum (in an ellipsoid)
Sigma_c_1 = 0.5*trace(plnr.I(:,:,1))*eye(3) - plnr.I(:,:,1); % density weighted covariance
Epsilon_pi_1 = inv(Sigma_c_1/plnr.m(1)); % covariance ellipsoid
Q(:,:,1) = blkdiag(-Epsilon_pi_1, 1);

Sigma_c_2 = 0.5*trace(plnr.I(:,:,2))*eye(3) - plnr.I(:,:,2); % density weighted covariance
Epsilon_pi_2 = inv(Sigma_c_2/plnr.m(2)); % covariance ellipsoid
Q(:,:,2) = blkdiag(-Epsilon_pi_2, 1);

pi_CAD = [plnr.pi(:,1); 0; plnr.pi(:,2)]; % parameters from the CAD
w_pi = 1e-6; % regulization term

cnstr = [pii(10) < 1.5, pii(21) < 0.75]; % constraints on the mass
if physicalConsistency
    k = 1; % iterates over rigid bodies
    for i = 1:11:21
        link_inertia_i = [pii(i),   pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];  
                      
        frst_mmnt_i = pii(i+6:i+8);
        
        % Positive definiteness of the generalized mass matrix
        Ji = [trace(link_inertia_i)/2*eye(3) - link_inertia_i, ...
                frst_mmnt_i; frst_mmnt_i', pii(i+9)];
            
        cnstr = [cnstr, Ji > 0];
        
        % First moment realizability on the ellipsoid
        if firstMomentRealizability
            Ci = [pii(i+9), frst_mmnt_i' - pii(i+9)*xs(:,k)';
                  frst_mmnt_i - pii(i+9)*xs(:,k), pii(i+9)*Qs(:,:,k)];
            cnstr = [cnstr, Ci > 0];
        end
              
        % Second moment realizability on the ellipsoid
        if secondMomentRealizability
            Pi = trace(Ji*Q(:,:,k));
            cnstr = [cnstr, Pi >= 0];
        end
        k = k + 1;
    end
else
    for i = 1:11:21
        link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];

        frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));

        Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
        cnstr = [cnstr, Di>0];
    end
end
% cnstr = [cnstr, pii(11) > 0, pii(10)> 0, pii(21) > 0];
cnstr = [cnstr, pii(11) > 0]; % first motor inertia constraint

% Feasibility constraints on the friction prameters 
% Columb and viscous friction coefficients are positive
for i = 1:2
   cnstr = [cnstr, pi_frctn(3*i-2) > 0, pi_frctn(3*i-1) > 0];  
end

%!!!!!!!!!!!!!!!!
% cnstr = [cnstr, pi_frctn(3) == 0, pi_frctn(6) == 0];

% Defining pbjective function
obj = norm(Tau - Wb*[pi_b; pi_frctn], 2)^2 + w_pi*norm(pii - pi_CAD)^2;

% Solving sdp problem
optns = sdpsettings;
optns.solver = 'sdpt3';
optns.verbose = 1;
optns.showprogress = 1;
diagnostics = optimize(cnstr, obj, optns);

if diagnostics.problem ~= 0
    disp('Solver was not able to solve the problem')
    return
end

pi_b = value(pi_b) % variables for base paramters
pi_frctn = value(pi_frctn)

pi_stnd = pndbtBaseQR.permutationMatrix*[eye(pndbtBaseQR.numberOfBaseParameters), ...
                                        -pndbtBaseQR.beta; ...
                                        zeros(15,pndbtBaseQR.numberOfBaseParameters), ... 
                                        eye(15)]*[value(pi_b); value(pi_d)];


%%
pi_rgd_1 = pi_stnd(1:10);
pi_rgd_2 = pi_stnd(12:end);

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
