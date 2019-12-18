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
% Full set of parameters of the robot
m = sym('m%d',[6,1],'real');
hx = sym('hx%d',[6,1],'real');
hy = sym('hy%d',[6,1],'real');
hz = sym('hz%d',[6,1],'real');
ixx = sym('ixx%d',[6,1],'real');
iyy = sym('iyy%d',[6,1],'real');
izz = sym('izz%d',[6,1],'real');
ixy = sym('ixy%d',[6,1],'real');
ixz = sym('ixz%d',[6,1],'real');
iyz = sym('iyz%d',[6,1],'real');

for i = 1:6
   pi_sym_scrw(:,i) = [m(i), hx(i), hy(i), hz(i), ixx(i), iyy(i),...
                izz(i), ixy(i), ixz(i), iyz(i)]'; 
end
pi_sym_scrw = reshape(pi_sym_scrw,[60,1]);


% -----------------------------------------------------------------------
% Finding paramters that do not affect dynamics i.e. unidentifiable params
% -----------------------------------------------------------------------
rng('shuffle');%seed the random number generator based on the current time
for i = 1:10
    q_rnd(:,i) = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd(:,i) = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd(:,i) = -q2d_max + 2*q2d_max.*rand(6,1);
end

% Obtain observation matrix by computing regressor for each sampling time
W = [];    
for i = 1:10
    Y = screw_regressor(q_rnd(:,i),qd_rnd(:,i),q2d_rnd(:,i),ur10);
    W = vertcat(W,Y);
end

% Find matrix that maps full regressor and full parameter vector
% into low dimensional paramter space
Prmt = [];
fprintf('List of unidentifiable parameters:\n')
for i = 1:60
    if all(W(:,i)==0)
        disp(pi_sym_scrw(i))
    else
        prmti = zeros(60,1);
        prmti(i) = 1;
        Prmt(:,end+1) = prmti;
    end
end

clear W q_rnd qd_rnd q2d_rnd


% -----------------------------------------------------------------------
% Find relation between independent columns and dependent columns
% -----------------------------------------------------------------------
for i = 1:20
    q_rnd(:,i) = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd(:,i) = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd(:,i) = -q2d_max + 2*q2d_max.*rand(6,1);
end

% Get observation matrix of identifiable paramters
W = [];    
for i = 1:20
    Y = screw_regressor(q_rnd(:,i),qd_rnd(:,i),q2d_rnd(:,i),ur10)*Prmt;
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


% -----------------------------------------------------------------------
% Find base parmaters
% -----------------------------------------------------------------------
% parameters that identifiable seperately or in combination
pi_scrw_sym_tlde = Prmt'*pi_sym_scrw; 

pi1 = E(:,1:bb)'*pi_scrw_sym_tlde; % independent paramters
pi2 = E(:,bb+1:end)'*pi_scrw_sym_tlde; % dependent paramteres

% all of the expressions below are equivalent
pi_scrw_base = pi1 + beta*pi2;
pi_scrw_base2 = [eye(bb) beta]*[pi1;pi2];
pi_scrw_base3 = [eye(bb) beta]*E'*pi_scrw_sym_tlde;
pi_scrw_base4 = [eye(bb) beta]*E'*Prmt'*pi_sym_scrw;


% -----------------------------------------------------------------------
% Validation of obtained mappings
% -----------------------------------------------------------------------
ur10.pi_scrw = reshape(ur10.pi_scrw,[60,1]);

% On random positions, velocities, aceeleations
for i = 1:100
    q_rnd = q_min + (q_max - q_min).*rand(6,1);
    qd_rnd = -qd_max + 2*qd_max.*rand(6,1);
    q2d_rnd = -q2d_max + 2*q2d_max.*rand(6,1);
    
    Yi = screw_regressor(q_rnd, qd_rnd, q2d_rnd, ur10);
    tau_full = Yi*ur10.pi_scrw;
    
    pi_scrw_base = [eye(bb) beta]*E'*Prmt'*ur10.pi_scrw;
    Y_base = Yi*Prmt*E(:,1:bb);
    tau_base = Y_base*pi_scrw_base;
    nrm_err1(i) = norm(tau_full - tau_base);
end
figure
plot(nrm_err1)
ylabel('||\tau - \tau_b||')


% On periodic trajectory
T = 5;          % period of signal
wf = 2*pi/T;    % fundamental frequency
t_smp = 1e-2;   % sampling time
t = 0:t_smp:T;  % time
N = 3;          % number of harmonics

q0 = zeros(6,1); % initial offset
a = rand(6,N); % sin coeffs
b = rand(6,N); % cos coeffs

% Compute trajectory (Fouruer series)
[q,qd,q2d] = fourier_series_traj(t,q0,a,b,wf,N);

for i = 1:100   
    Yi = screw_regressor(q(:,i), qd(:,i), q2d(:,i), ur10);
    tau_full = Yi*ur10.pi_scrw;
    
    pi_scrw_base = [eye(bb) beta]*E'*Prmt'*ur10.pi_scrw;
    Y_base = Yi*Prmt*E(:,1:bb);
    tau_base = Y_base*pi_scrw_base;
    nrm_err2(i) = norm(tau_full - tau_base);
end

figure
plot(nrm_err2)
ylabel('||\tau - \tau_b||')
