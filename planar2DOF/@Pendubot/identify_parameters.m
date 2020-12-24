function [pi_rgb_hat, pi_frcn_hat] = identify_parameters(obj, tau, q, q_dot, q_2dot)
    % It is possible to choose between OLS, physical consistency that was introduced
    % by Cortesao (generalized momentum is positive semidefinite) or physical
    % consistency that was introduced by Wensing and Slotine and then by
    % Cartesao. 
    % Also it is possible to assign realizability of the first and second
    % moments on the ellipsoid. For that more or less accurate CAD parameter
    % required.
    regression_types = {'OLS', 'semiphysical_consistency', 'physical_consistency'};
    regr = regression_types{2};
    first_moment_realizability = 1;
    second_moment_realizability = 0;
    regulization = 0;
    % -----------------------------------------------------------------
    
    T = obj.get_aggregated_torque_vector(tau);
    Ws = obj.get_rigid_body_observation_matrix(q, q_dot, q_2dot, 'standard');
    Wb = obj.get_rigid_body_observation_matrix(q, q_dot, q_2dot, 'base');
    Wf = obj.get_friction_observation_matrix(q_dot, 'continuous');
    W = [Wb Wf];
    fprintf('Condition number of the observation matrix = %d \n', cond(W));
    pi_hat = (W'*W)\(W'*T);
    pi_rgb_hat = pi_hat(1:obj.qr_decomposition.no_base_parameters);
    pi_frcn_hat = pi_hat(obj.qr_decomposition.no_base_parameters+1:end);
    
    return
    % -----------------------------------------------------------------
    pi_frctn = sdpvar(4,1);
    pi_b = sdpvar(obj.qr_decomposition.no_base_parameters, 1); % variables for base paramters
    pi_d = sdpvar(14, 1); % variables for dependent paramters
    
    % Bijective mapping from [pi_b; pi_d] to standard parameters pi
    pii = obj.qr_decomposition.permutation_matrix*[eye(obj.qr_decomposition.no_base_parameters), ...
                                                    -obj.qr_decomposition.beta; ...
                                                    zeros(14, obj.qr_decomposition.no_base_parameters), ... 
                                                    eye(14)]*[pi_b; pi_d];
                                                
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
%     sclng = 1.8;
%     Sigma_c_1 = 0.5*trace(sclng*plnr.I(:,:,1))*eye(3) - sclng*plnr.I(:,:,1); % density weighted covariance
%     Epsilon_pi_1 = inv(Sigma_c_1/plnr.m(1)); % covariance ellipsoid
%     Q(:,:,1) = blkdiag(-Epsilon_pi_1, 1);
% 
%     Sigma_c_2 = 0.5*trace(sclng*plnr.I(:,:,2))*eye(3) - sclng*plnr.I(:,:,2); % density weighted covariance
%     Epsilon_pi_2 = inv(Sigma_c_2/plnr.m(2)); % covariance ellipsoid
%     Q(:,:,2) = blkdiag(-Epsilon_pi_2, 1);
    
    % Constraints
    cnstr = [pii(10) < 2., pii(20) < 1.]; % constraints on the mass
    
    k = 1; % iterates over rigid bodies
    for i = 1:10:20
        second_moment_k = [pii(i),   pii(i+1), pii(i+2); ...
                           pii(i+1), pii(i+3), pii(i+4); ...
                           pii(i+2), pii(i+4), pii(i+5)]; 
        first_moment_k = pii(i+6:i+8);
        mass_k = pii(i+9);

        % Physical or semi-physical consistency 
        if  (strcmp(regr, 'physical_consistency'))
            Jk = [0.5*trace(second_moment_k)*eye(3) - second_moment_k, first_moment_k;
                  first_moment_k', mass_k];
            cnstr = [cnstr, Jk > 0];
        elseif (strcmp(regr, 'semiphysical_consistency'))
            Dk = [second_moment_k, vec2skewSymMat(first_moment_k)';
                  vec2skewSymMat(first_moment_k), mass_k*eye(3)];
            cnstr = [cnstr, Dk > 0];
        end

        % First moment realizability on the ellipsoid
        if first_moment_realizability
            Ck = [mass_k, first_moment_k' - mass_k*xs(:,k)';
                  first_moment_k - mass_k*xs(:,k), mass_k*Qs(:,:,k)];
            cnstr = [cnstr, Ck > 0];
        end

        % Second moment realizability on the ellipsoid
        if second_moment_realizability
            Pk = trace(Jk*Q(:,:,k));
            cnstr = [cnstr, Pk >= 0];
        end
        k = k + 1;
    end
    
    % Feasibility constraints on the friction prameters 
    % Columb and viscous friction coefficients are positive
    for i = 1:2
       cnstr = [cnstr, pi_frctn(2*i-1) > 0, pi_frctn(2*i) > 0];  
    end
    
    if regulization
        w_pi = 1e-6; % regulization term
        pi_CAD = obj.get_dynamic_parameters_from_urdf('standard'); % parameters from the CAD
        obj = norm(T - W*[pii; pi_frctn], 2)^2 + w_pi*norm(pii - pi_CAD, 2)^2;
    else
        obj = norm(T - W*[pii; pi_frctn], 2)^2;
    end
    
    % Solving sdp problem
    optns = sdpsettings;
    optns.solver = 'sdpt3';
    optns.verbose = 1;
    optns.showprogress = 1;
    diagnostics = optimize(cnstr, obj, optns);

    if diagnostics.problem ~= 0
        pi_frctn = [];
        pi_stnd = [];
        error('Solver was not able to solve the problem')
    end

    pi_rgb_hat = value(pi_b); % variables for base paramters
    pi_frcn_hat = value(pi_frctn);
end