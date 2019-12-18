function out = traj_cost_scrw(opt_vars,traj_par,ur10)
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
c_pol = getPolCoeffs(T, a, b, wf, N, ur10.q0);

% Compute trajectory (Fouruer series + fifth order polynomail)
[q,qd,q2d] = mixed_traj(t, c_pol, a, b, wf, N);
    
% Obtain observation matrix by computing regressor for each sampling time
W = [];    
for i = 1:length(t)
    Y = screw_regressor2(q(:,i),qd(:,i),q2d(:,i),ur10);
    W = vertcat(W,Y);
end

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% should be changed
% ------------------------------------------------------------------------
% Observation matrix is not full rank in full vector of parameters. In
% order to extract submatrix of full rank, we perform QR decomposition.
% ------------------------------------------------------------------------
    [Q,R,E] = qr(W); 
    
    % matrix W has rank bb which is number number of base parameters and 
    % cc unidentifiable parameters
    bb = rank(W);
    
    WE = W*E; % permuted observation matrix
    W1 = WE(:,1:bb); % full rank observation matrix
    
    out = cond(W1);
end