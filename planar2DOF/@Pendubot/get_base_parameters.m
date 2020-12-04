function get_base_parameters(obj)
% -------------------------------------------------------------------
% This script finds base parameters of the of the pendubot robot.
% Base parameters are found using numerica methods, namely 
% QR decomposition.
% -------------------------------------------------------------------

% Seed the random number generator based on the current time
rng('shuffle');

% limits on positions, velocities, accelerations
q_min = -2*pi; 
q_max = 2*pi;
qd_min = -10;
qd_max = 10;
q2d_min = -100;
q2d_max = 100;


% Find relation between independent columns and dependent columns
W = [];    
for i = 1:20
    q_rnd = q_min + (q_max - q_min).*rand(2,1);
    qd_rnd = -qd_min + (qd_max - qd_min).*rand(2,1);
    q2d_rnd = -q2d_min + (q2d_max - q2d_min).*rand(2,1);
    
    Yi = obj.get_rigid_body_regressor(q_rnd, qd_rnd, q2d_rnd, 'standard');
    W = vertcat(W,Yi);
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
R1 = R(1:bb, 1:bb);
R2 = R(1:bb, bb+1:end);
% mapping between independent and dependent columns of W
beta = R1\R2; % the zero rows of K correspond to independent columns of WP
beta(abs(beta)<sqrt(eps)) = 0; % get rid of numerical errors

% Make sure that the relation holds
W1 = W*E(:,1:bb);
W2 = W*E(:,bb+1:end);
assert(norm(W2 - W1*beta) < 1e-6, 'Found realationship between W1 and W2 is not correct\n');


% Create structure with the result of QR decompositon and save it
% for further use.
no_base_parameters = bb;
permutation_matrix = E;


filename = 'planar2DOF/@Pendubot/qr_decomposition.mat';
save(filename,'no_base_parameters', 'permutation_matrix', 'beta');
