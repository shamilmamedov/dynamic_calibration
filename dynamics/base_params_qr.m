function [pi_lgr_base, baseQR] = base_params_qr(includeMotorDynamics)
% ----------------------------------------------------------------------
% In this function QR decomposition is applied to regressor in closed
% form obtained from Lagrange formulation of dynamics.
%
% You should already have a function to compute the regressor matrix of the
% robot full_regressor_UR10E.m 
%
% In the beginning you need to choose if you want to include motor dynamics
% (reflected inertia of the motor). A rule of thumb is to include it.
% 
% After finding base parametrs the test is performed which compares the
% output torques with base and full parameters for randomly generated data.
% 
% Finally, a structure is generated and saved with parameters necessary to 
% find base  parameters from standard ones, and base regressor from a
% strandard one.
% ----------------------------------------------------------------------
% Seed the random number generator based on the current time
rng('shuffle');

% ------------------------------------------------------------------------
% Set limits on posistion and velocities
% ------------------------------------------------------------------------
q_min = -pi*ones(6,1);
q_max = pi*ones(6,1);
qd_max = 3*pi*ones(6,1);
q2d_max = 6*pi*ones(6,1); 

% -----------------------------------------------------------------------
% Standard dynamics paramters of the robot in symbolic form
% -----------------------------------------------------------------------
m = sym('m%d',[6,1],'real');
hx = sym('h%d_x',[6,1],'real');
hy = sym('h%d_y',[6,1],'real');
hz = sym('h%d_z',[6,1],'real');
ixx = sym('i%d_xx',[6,1],'real');
ixy = sym('i%d_xy',[6,1],'real');
ixz = sym('i%d_xz',[6,1],'real');
iyy = sym('i%d_yy',[6,1],'real');
iyz = sym('i%d_yz',[6,1],'real');
izz = sym('i%d_zz',[6,1],'real');
im = sym('im%d',[6,1],'real');

% Load parameters attached to the end-effector
syms ml hl_x hl_y hl_z il_xx il_xy il_xz il_yy il_yz il_zz      real 

% Vector of symbolic parameters
for i = 1:6
    if includeMotorDynamics
        pi_lgr_sym(:,i) = [ixx(i),ixy(i),ixz(i),iyy(i),iyz(i),izz(i),...
                           hx(i),hy(i),hz(i),m(i),im(i)]';
    else
        pi_lgr_sym(:,i) = [ixx(i),ixy(i),ixz(i),iyy(i),iyz(i),izz(i),...
                           hx(i),hy(i),hz(i),m(i)]';
    end
end
[nLnkPrms, nLnks] = size(pi_lgr_sym);
pi_lgr_sym = reshape(pi_lgr_sym, [nLnkPrms*nLnks, 1]);


% -----------------------------------------------------------------------
% Find relation between independent columns and dependent columns
% -----------------------------------------------------------------------
% Get observation matrix of identifiable paramters
W = [];    
for i = 1:25
    q_rnd = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(6,1);
    
    if includeMotorDynamics
        Y = regressorWithMotorDynamics(q_rnd,qd_rnd,q2d_rnd);
    else
        Y = full_regressor_UR10E(q_rnd,qd_rnd,q2d_rnd);
    end
    W = vertcat(W,Y);
end

% QR decomposition with pivoting: W*E = Q*R
%   R is upper triangular matrix
%   Q is unitary matrix
%   E is permutation matrix
[Q, R, E] = qr(W);

% matrix W has rank bb which is number number of base parameters 
bb = rank(W);

% R = [R1 R2; 
%      0  0]
% R1 is bbxbb upper triangular and reguar matrix
% R2 is bbx(c-bb) matrix where c is number of standard parameters
R1 = R(1:bb,1:bb);
R2 = R(1:bb,bb+1:end);
beta = R1\R2; % the zero rows of K correspond to independent columns of WP
beta(abs(beta)<sqrt(eps)) = 0; % get rid of numerical errors
% W2 = W1*beta

% Make sure that the relation holds
W1 = W*E(:,1:bb);
W2 = W*E(:,bb+1:end);
assert(norm(W2 - W1*beta) < 1e-6,... 
        'Found realationship between W1 and W2 is not correct\n');

% -----------------------------------------------------------------------
% Find base parmaters
% -----------------------------------------------------------------------
pi1 = E(:,1:bb)'*pi_lgr_sym; % independent paramters
pi2 = E(:,bb+1:end)'*pi_lgr_sym; % dependent paramteres

% all of the expressions below are equivalent
pi_lgr_base = pi1 + beta*pi2;
% pi_lgr_base = [eye(bb) beta]*[pi1;pi2];
% pi_lgr_base = [eye(bb) beta]*E'*pi_lgr_sym;

% ---------------------------------------------------------------------
% Create structure with the result of QR decompositon a
% ---------------------------------------------------------------------
baseQR = struct;
baseQR.numberOfBaseParameters = bb;
baseQR.permutationMatrix = E;
baseQR.beta = beta;
baseQR.motorDynamicsIncluded = includeMotorDynamics;


