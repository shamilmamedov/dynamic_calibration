function out = traj_cost_lgr(opt_vars,traj_par,baseQR)
% -------------------------------------------------------------------
% This function computes cost in terms of condition number for 
% trajectory optimization needed for dynamic parameter identification
% The computation of regressor matrix is obtained using screw 
% theory methods.
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
   
out = cond(W);
