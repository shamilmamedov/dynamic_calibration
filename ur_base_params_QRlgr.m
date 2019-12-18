% Get robot description
run('main_ur.m')

% ------------------------------------------------------------------------
% Getting limits on posistion and velocities
% ------------------------------------------------------------------------
q_min = zeros(6,1);
q_max = zeros(6,1);
qd_max = zeros(6,1);
q2d_max = 2*ones(6,1); % it is chosen by us as it is not given in URDF
for i = 1:6
    q_min(i) = str2double(ur10.robot.joint{i}.limit.Attributes.lower);
    q_max(i) = str2double(ur10.robot.joint{i}.limit.Attributes.upper);
    qd_max(i) = str2double(ur10.robot.joint{i}.limit.Attributes.velocity);
end


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
    pi_lgr_sym(:,i) = [ixx(i),ixy(i),ixz(i),iyy(i),iyz(i),izz(i),...
                    hx(i),hy(i),hz(i),m(i)]';
end
pi_lgr_sym = reshape(pi_lgr_sym,[60,1]);


% -----------------------------------------------------------------------
% Finding paramters that do not affect dynamics i.e. unidentifiable params
% -----------------------------------------------------------------------
rng('shuffle');%seed the random number generator based on the current time
%{
% Obtain observation matrix by computing regressor for each sampling time
W = [];    
for i = 1:10
    q_rnd = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(6,1);
    
    Y = full_regressor_UR10E(q_rnd,qd_rnd,q2d_rnd);
    W = vertcat(W,Y);
end

% Find matrix that maps full regressor and full parameter vector
% into low dimensional paramter space
Prmt = [];
fprintf('List of unidentifiable parameters:\n')
for i = 1:60
    if all(W(:,i)==0)
        disp(pi_lgr_sym(i))
    else
        prmti = zeros(60,1);
        prmti(i) = 1;
        Prmt(:,end+1) = prmti;
    end
end

clear W q_rnd qd_rnd q2d_rnd
%}

% -----------------------------------------------------------------------
% Find relation between independent columns and dependent columns
% -----------------------------------------------------------------------
% Get observation matrix of identifiable paramters
W = [];    
for i = 1:20
    q_rnd = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(6,1);
    
    Y = full_regressor_UR10E(q_rnd,qd_rnd,q2d_rnd);
    W = vertcat(W,Y);
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
% R2 is bbx(c-bb) matrix where c is number of identifiable parameters
R1 = R(1:bb,1:bb);
R2 = R(1:bb,bb+1:end);
beta = R1\R2; % the zero rows of K correspond to independent columns of WP
beta(abs(beta)<sqrt(eps)) = 0; % get rid of numerical errors
% W2 = W1*beta

% Make sure that the relation holds
W1 = W*E(:,1:bb);
W2 = W*E(:,bb+1:end);
norm(W2 - W1*beta)

return
% -----------------------------------------------------------------------
% Find base parmaters
% -----------------------------------------------------------------------
% parameters that identifiable seperately or in combination
pi_lgr_sym_tlde = Prmt'*pi_lgr_sym; 

pi1 = E(:,1:bb)'*pi_lgr_sym_tlde; % independent paramters
pi2 = E(:,bb+1:end)'*pi_lgr_sym_tlde; % dependent paramteres

% all of the expressions below are equivalent
pi_lgr_base = pi1 + beta*pi2;
pi_lgr_base2 = [eye(bb) beta]*[pi1;pi2];
pi_lgr_base3 = [eye(bb) beta]*E'*pi_lgr_sym_tlde;
pi_lgr_base4 = [eye(bb) beta]*E'*Prmt'*pi_lgr_sym;

return
% -----------------------------------------------------------------------
% Validation of obtained mappings
% -----------------------------------------------------------------------
ur10.pi = reshape(ur10.pi,[60,1]);

% On random positions, velocities, aceeleations
for i = 1:100
    q_rnd = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(6,1);
       
    Yi = full_regressor_UR10E(q_rnd, qd_rnd, q2d_rnd);
    tau_full = Yi*ur10.pi;
    
    pi_lgr_base = [eye(bb) beta]*E'*Prmt'*ur10.pi;
    Y_base = Yi*Prmt*E(:,1:bb);
    tau_base = Y_base*pi_lgr_base;
    nrm_err1(i) = norm(tau_full - tau_base);
end
figure
plot(nrm_err1)
ylabel('||\tau - \tau_b||')





% -----------------------------------------------------------------------
% Additional functions
% -----------------------------------------------------------------------
function Y = regressorWithMotorDynamics(q,qd,q2d)
% ----------------------------------------------------------------------
% This function adds motor dynamics to rigid body regressor.
% It is simplified model of motor dynamics, it adds only reflected
% inertia i.e. I_rflctd = Im*N^2 where N is reduction ratio - I_rflctd*q_2d
% parameter is added to existing vector of each link [pi_i I_rflctd_i]
% so that each link has 11 parameters
% ----------------------------------------------------------------------
    Y_rgd_bdy = full_regressor_UR10E(q,qd,q2d);
    Y_mtrs = diag(q2d);
    Y = [Y_rgd_bdy(:,1:10), Y_mtrs(:,1), Y_rgd_bdy(:,11:20), Y_mtrs(:,2),...
         Y_rgd_bdy(:,21:30), Y_mtrs(:,3), Y_rgd_bdy(:,31:40), Y_mtrs(:,4),...
         Y_rgd_bdy(:,41:50), Y_mtrs(:,5), Y_rgd_bdy(:,51:60), Y_mtrs(:,6)];
end