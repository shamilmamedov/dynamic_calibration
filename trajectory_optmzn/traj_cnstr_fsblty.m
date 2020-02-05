function [c,ceq] = traj_cnstr_fsblty(opt_vars, traj_par, baseQR)
% --------------------------------------------------------------------
% The function computes constraints on trajectory for trajectoty
% optimization needed for dynamic parameter identification more 
% precisely for searching feasibly solutions with condition number
% less than 100 that guarantees good estimate
% -------------------------------------------------------------------
% Trajectory parameters
N = traj_par.N;
wf = traj_par.wf;
T = traj_par.T;
t = traj_par.t;

% As paramters of the trajectory are in a signle vector we reshape them as
% to feed the function that computes the trajectory
ab = reshape(opt_vars,[12,N]);
a = ab(1:6,:); % sin coeffs
b = ab(7:12,:); % cos coeffs

% To guarantee that positions, velocities and accelerations are zero in the
% beginning and at time T, we add fifth order polynomial to fourier
% series. The parameters of the polynomial depends on the parameters of
% fourier series. Here we compute them.
c_pol = getPolCoeffs(T, a, b, wf, N, traj_par.q0);   

% Compute trajectory (Fouruer series + fifth order polynomail)
[q,qd,q2d] = mixed_traj(t, c_pol, a, b, wf, N);

% Obtain observation matrix by computing regressor for each sampling time
E1 = baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);
W = [];    
for i = 1:length(t)
%     Y = base_regressor_UR10E(q(:,i),qd(:,i),q2d(:,i));
    if baseQR.motorDynamicsIncluded
        Y = [regressorWithMotorDynamics(q(:,i),qd(:,i),q2d(:,i))*E1, ...
             frictionRegressor(qd(:,i))];
    else
        Y = [full_regressor_UR10E(q(:,i),qd(:,i),q2d(:,i))*E1, ...
             frictionRegressor(qd(:,i))];
    end
    W = vertcat(W,Y);
end


% Inequality constraints
c(1:6) = traj_par.q_min - min(q,[],2); % upper joint limit constraint
c(7:12) = max(q,[],2) - traj_par.q_max; % lower joint limit constraint
c(13:18) = max(abs(qd),[],2) - traj_par.qd_max; % max joint velocity const
c(19:24) = max(abs(q2d),[],2) - traj_par.q2d_max; % max joint acceleration constr
c(25) = cond(W) - 100;

% Equality contrsints
ceq = [];