% get robot description
run('plnr_idntfcn.m')

% Seed the random number generator based on the current time
rng('shuffle');

% some parameters
includeMotorDynamics = 1;


% limits on positions, velocities, accelerations
q_min = -2*pi;
q_max = 2*pi;
qd_min = -10;
qd_max = 10;
q2d_min = -100;
q2d_max = 100;

% -----------------------------------------------------------------------
% Find relation between independent columns and dependent columns
% -----------------------------------------------------------------------
% Get observation matrix of identifiable paramters
W = [];    
for i = 1:20
    q_rnd = q_min + (q_max - q_min).*rand(2,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(2,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(2,1);
    
    if includeMotorDynamics
        Yi = regressorWithMotorDynamicsPndbt(q_rnd, qd_rnd, q2d_rnd);
    else
        Yi = full_regressor_plnr(q_rnd, qd_rnd, q2d_rnd);
    end
    W = vertcat(W,Yi);
end

% QR decomposition with pivoting: W*E = Q*R
%   R is upper triangular matrix
%   Q is unitary matrix
%   E is permutation matrix
[Q,R,E] = qr(W);

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
if norm(W2 - W1*beta) > 1e-6
   fprintf('Found realationship between W1 and W2 is not correct\n');
   return
end


% Defining parameters symbolically
m = sym('m%d',[2,1],'real');
hx = sym('h%d_x',[2,1],'real');
hy = sym('h%d_y',[2,1],'real');
hz = sym('h%d_z',[2,1],'real');
ixx = sym('i%d_xx',[2,1],'real');
ixy = sym('i%d_xy',[2,1],'real');
ixz = sym('i%d_xz',[2,1],'real');
iyy = sym('i%d_yy',[2,1],'real');
iyz = sym('i%d_yz',[2,1],'real');
izz = sym('i%d_zz',[2,1],'real');
im = sym('im',[1,1],'real');

% Vector of symbolic parameters
pi_pndbt_sym = {};
for i = 1:2
    pi_pndbt_sym{i} = [ixx(i),ixy(i),ixz(i),iyy(i),iyz(i),izz(i),...
                           hx(i),hy(i),hz(i),m(i)]';
end

if includeMotorDynamics
   pi_pndbt_sym{1} = [pi_pndbt_sym{1}; im]; 
end
pi_pndbt_sym = [pi_pndbt_sym{1}; pi_pndbt_sym{2}];


% Find base parmaters
pi1 = E(:,1:bb)'*pi_pndbt_sym; % independent paramters
pi2 = E(:,bb+1:end)'*pi_pndbt_sym; % dependent paramteres

% all of the expressions below are equivalent
pi_lgr_base = pi1 + beta*pi2;
pi_lgr_base2 = [eye(bb) beta]*[pi1;pi2];
pi_lgr_base3 = [eye(bb) beta]*E'*pi_pndbt_sym;

% -----------------------------------------------------------------------
% Validation of obtained mappings
% -----------------------------------------------------------------------
fprintf('Validation of mapping from standard parameters to base ones\n')
if includeMotorDynamics
    plnr.pi = [plnr.pi(:,1); rand; plnr.pi(:,2)];
else
    plnr.pi = [plnr.pi(:,1); plnr.pi(:,2)];
end

% On random positions, velocities, aceeleations
for i = 1:100
    q_rnd = q_min + (q_max - q_min).*rand(2,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(2,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(2,1);
    
    if includeMotorDynamics
        Yi = regressorWithMotorDynamicsPndbt(q_rnd,qd_rnd,q2d_rnd);
    else
        Yi = full_regressor_plnr(q_rnd,qd_rnd,q2d_rnd);
    end
    tau_full = Yi*plnr.pi;
    
    pi_lgr_base = [eye(bb) beta]*E'*plnr.pi;
    Y_base = Yi*E(:,1:bb);
    tau_base = Y_base*pi_lgr_base;
    nrm_err1(i) = norm(tau_full - tau_base);
end
figure
plot(nrm_err1)
ylabel('||\tau - \tau_b||')
grid on

if ~all(nrm_err1<1e-6)
    fprintf('Validation failed')
    return
end

% ---------------------------------------------------------------------
% Create structure with the result of QR decompositon and save it
% for further use.
% ---------------------------------------------------------------------
plnrBaseQR = struct;
plnrBaseQR.numberOfBaseParameters = bb;
plnrBaseQR.permutationMatrix = E;
plnrBaseQR.beta = beta;
plnrBaseQR.motorDynamicsIncluded = includeMotorDynamics;

filename = 'planar2DOF/plnrBaseQR.mat';
save(filename,'plnrBaseQR')