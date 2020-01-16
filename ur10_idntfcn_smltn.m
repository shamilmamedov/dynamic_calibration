% ---------------------------------------------------------------------
% In this script identification of dynamic parameters as well as
% drive gains are performed on ideal trajectory found from
% optimization procedure.
% ---------------------------------------------------------------------
clc; clear all; close all;

% get robot parameters from urdf
run('main_ur.m');

% add reflected rotor inertia to parameters
reflectedRotorInertia = rand([1,6]);
ur10.pi = [ur10.pi; reflectedRotorInertia];
ur10.pi = reshape(ur10.pi,[size(ur10.pi,1)*size(ur10.pi,2), 1]);

% get base parameters
load('baseQR.mat'); 
E1 = baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);
ur10.pi_base = [eye(baseQR.numberOfBaseParameters) baseQR.beta]*...
                    baseQR.permutationMatrix'*ur10.pi;
                
% add friction parmeters to the vector of base parameters
frictionParameters = rand([18,1]);
ur10.pi_base = [ur10.pi_base; frictionParameters];

% optimized trajectory
load('ptrnSrch_N5T20QR.mat'); 
traj_par.t_smp = 1e-1; % sampling time
traj_par.t = 0:traj_par.t_smp:traj_par.T; % time

% generilized corrdinates, velocities and acelerations of the trajectory
[q,qd,q2d] = mixed_traj(traj_par.t,c_pol,a,b,traj_par.wf,traj_par.N);

% add noise to data
noiseVariance = 0.1;
q = q + sqrt(noiseVariance)*rand(size(q));
qd = qd + sqrt(noiseVariance)*rand(size(qd));
q2d = q2d + sqrt(2*noiseVariance)*rand(size(q2d));

% load and drive paramters
m_load = 5;
pi_load =  [0.2,0.05,0.07,0.15,0.01,0.4, 0.05, 0, 0, m_load]';
K_drv = [20 16 13 10 9 8]';

% Constracting regressor matrix for unloaded case
Wb_uldd = []; I_uldd = []; 
for i = 1:1:length(traj_par.t)
    Y_ulddi = regressorWithMotorDynamics(q(:,i),qd(:,i),q2d(:,i));                              
    Yfrctni = frictionRegressor(qd(:,i));
    Ybi_uldd = [Y_ulddi*E1, Yfrctni];
    taui_uldd = Ybi_uldd*ur10.pi_base + sqrt(noiseVariance)*rand(6,1);
        
    Wb_uldd = vertcat(Wb_uldd, Ybi_uldd);
    I_uldd = vertcat(I_uldd, diag(diag(K_drv)\taui_uldd));
end

% Constracting regressor matrix for loaded case
Wb_ldd = []; Wl = []; I_ldd = [];
for i = 1:1:length(traj_par.t)
    Y_lddi = regressorWithMotorDynamics(q(:,i),qd(:,i),q2d(:,i));                                
    Yfrctni = frictionRegressor(qd(:,i));
    Ybi_ldd = [Y_lddi*E1, Yfrctni];
    Yli = load_regressor_UR10E(q(:,i),qd(:,i),q2d(:,i));
    taui_ldd = [Ybi_ldd Yli]*[ur10.pi_base; pi_load] + ...
                                        sqrt(noiseVariance)*rand(6,1);
    
    Wb_ldd = vertcat(Wb_ldd, Ybi_ldd);
    Wl = vertcat(Wl, Yli); 
    I_ldd = vertcat(I_ldd, diag(diag(K_drv)\taui_ldd));
end
Wl_uknown = Wl(:,1:9); % regressor matrix of the unknown load params
Wl_known = Wl(:,10); % regressor matrix of the known load params
nmbrRwsUnldd = size(Wb_uldd,1);
nmbrRwsLdd = size(Wb_ldd,1);


%% Using total least squares
Wb_tls = [I_uldd   -Wb_uldd   zeros(size(I_uldd,1), size(Wl,2));
          I_ldd    -Wb_ldd    -Wl_uknown    -Wl_known*m_load];

% SVD decompostion of Wb_tls to solve total least squares
[~,~,V] = svd(Wb_tls,'econ');
% Scaling of the solution
lmda = 1/V(end,end);
pi_tls = lmda*V(:,end);
% drive gains
drvGains = [pi_tls(1:6) K_drv]

% Finding weighting matrix, joint by joint
G = zeros(6);
for i = 1:6
    Wib_tls = Wb_tls(i:6:end,:);
    [~,Si,Vi] = svd(Wib_tls,'econ');
    sgmai = Si(end,end)/sqrt((size(Wib_tls,1)-rank(Wib_tls))); %size(Wib_tls,2)));
    G(i,i) = 1/sgmai^2;
end

for i = 1:6:size(Wb_tls,1)
    Wb_tls(i:i+5,:) = G*Wb_tls(i:i+5,:);
end
[~,~,V] = svd(Wb_tls,'econ');
lmda = 1/V(end,end);
pi_tls = lmda*V(:,end);
drvGains = pi_tls(1:6)

% Finding weighting matrix, joint by joint
G = zeros(6);
for i = 1:6
    Wib_tls = Wb_tls(i:6:end,:);
    [~,Si,Vi] = svd(Wib_tls,'econ');
    sgmai = Si(end,end)/sqrt((size(Wib_tls,1)-rank(Wib_tls))); %size(Wib_tls,2)));
    G(i,i) = 1/sgmai^2;
end

for i = 1:6:size(Wb_tls,1)
    Wb_tls(i:i+5,:) = G*Wb_tls(i:i+5,:);
end
[~,~,V] = svd(Wb_tls,'econ');
lmda = 1/V(end,end);
pi_tls = lmda*V(:,end);
drvGains = pi_tls(1:6)


return
%% Identification of parameters including drive gains
Wb_ls = [I_uldd     -Wb_uldd    zeros(size(I_uldd,1), size(Wl_uknown,2));
         I_ldd      -Wb_ldd     -Wl_uknown];
     
Yb_ts = [zeros(size(I_uldd,1),1); Wl_known*m_load];

% Compute least squares solution
pi_ls = ((Wb_ls'*Wb_ls)\Wb_ls')*Yb_ts;
drvGains = [pi_ls(1:6) K_drv]


% Find weighting matrix
G = zeros(6);
for i = 1:6
    Wib_ls = Wb_ls(i:6:end,:);
    Yib_ls = Yb_ts(i:6:end);
    sgmai_sqrd = norm(Yib_ls - Wib_ls*pi_ls,2)^2/(size(Wib_ls,1)-size(Wib_ls,2));
    G(i,i) = 1/sqrt(sgmai_sqrd);
end

% Weight data with weighting matrix
for i = 1:6:size(Wb_ls,1)
    Wb_ls(i:i+5,:) = G*Wb_ls(i:i+5,:);
    Yb_ts(i:i+5) = G*Yb_ts(i:i+5);
end
pi_tot = ((Wb_ls'*Wb_ls)\Wb_ls')*Yb_ts;
drvGains = pi_tot(1:6)



G_uldd = zeros(6);
G_ldd = zeros(6);
for i = 1:6
    Wib_uldd = Wb_ls(i:6:nmbrRwsUnldd,:);
    Yib_uldd = Yb_ts(i:6:nmbrRwsUnldd);
    sgmai_sqrd = norm(Yib_uldd - Wib_uldd*pi_ls,2)^2/(size(Wib_uldd,1)-rank(Wib_uldd));
    G_uldd(i,i) = 1/sqrt(sgmai_sqrd);
    
    Wib_ldd = Wb_ls(nmbrRwsUnldd+i:6:end,:);
    Yib_ldd = Yb_ts(nmbrRwsUnldd+i:6:end);
    sgmai_sqrd = norm(Yib_ldd - Wib_ldd*pi_ls,2)^2/(size(Wib_ldd,1)-rank(Wib_ldd));
    G_ldd(i,i) = 1/sqrt(sgmai_sqrd);
end


for i = 1:6:size(Wb_uldd,1)
    Wb_ls(i:i+5,:) = G_uldd*Wb_ls(i:i+5,:);
    Yb_ts(i:i+5) = G_uldd*Yb_ts(i:i+5);
end

for i = nmbrRwsUnldd+1:6:nmbrRwsUnldd+size(Wb_ldd,1)   
    Wb_ls(i:i+5,:) = G_ldd*Wb_ls(i:i+5,:);
    Yb_ts(i:i+5) = G_ldd*Yb_ts(i:i+5);
end

pi_tot = ((Wb_ls'*Wb_ls)\Wb_ls')*Yb_ts;
drvGains = pi_tot(1:6)



return

%% Set-up SDP optimization procedure
drv_gns = sdpvar(6,1); % variables for base paramters
pi_load_unknw = sdpvar(9,1); % varaibles for unknown load paramters
pi_frctn = sdpvar(18,1);
pi_b = sdpvar(baseQR.numberOfBaseParameters,1); % variables for base paramters
pi_d = sdpvar(26,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = baseQR.permutationMatrix*[ eye(baseQR.numberOfBaseParameters), ...
                                -baseQR.beta; ...
                                zeros(26,baseQR.numberOfBaseParameters), ... 
                                eye(26) ]*[pi_b; pi_d];

% Feasibility contrraints of the link paramteres and rotor inertia
cnstr = diag(drv_gns)>0;
for i = 1:11:66
    link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                      pii(i+1), pii(i+3), pii(i+4); ...
                      pii(i+2), pii(i+4), pii(i+5)];
                  
    frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));
    
    Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
    cnstr = [cnstr, Di>0, pii(i+10)>0];
end

% Feasibility constraints on the load paramters
load_inertia = [pi_load_unknw(1), pi_load_unknw(2), pi_load_unknw(3); ...
                pi_load_unknw(2), pi_load_unknw(4), pi_load_unknw(5); ...
                pi_load_unknw(3), pi_load_unknw(5), pi_load_unknw(6)];                  
load_frst_mmnt = vec2skewSymMat(pi_load_unknw(7:9));    
Dl = [load_inertia, load_frst_mmnt'; load_frst_mmnt, m_load*eye(3)];

cnstr = [cnstr, Dl>0];

% Feasibility constraints on the friction prameters 
for i = 1:6
   cnstr = [cnstr, pi_frctn(3*i-2)>0, pi_frctn(3*i-1)>0];  
end

% Defining pbjective function
t1 = [zeros(size(I_uldd,1),1); -Wl(:,end)*m_load];

t2 = [-I_uldd, Wb_uldd, zeros(size(Wb_uldd,1), size(Wl,2)-1); ...
      -I_ldd, Wb_ldd, Wl(:,1:9) ];
  
obj = norm(t1 - t2*[drv_gns; pi_b; pi_frctn; pi_load_unknw]);

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));

% Getting values of the estimated patamters
drv_gns = [value(drv_gns) K_drv]


return

% ------------------------------------------------------------------------
% Using SDP to find load parmeters along with drive gains
% ------------------------------------------------------------------------
%{
drv_gns = sdpvar(6,1); % variables for base paramters
pi_load_unknw = sdpvar(9,1); % varaibles for unknown load paramters
% u = sdpvar(1,1); % variable for cost
% 
% % Writing |Tau - W*pi_b| using Schur compliment
% U1 = [u, (-Wl(:,end)*m_load - [-(I_ldd-I_uldd) Wl(:,1:9)]*[drv_gns;pi_load_unknw])'; ...
%       -Wl(:,end)*m_load - [-(I_ldd-I_uldd) Wl(:,1:9)]*[drv_gns;pi_load_unknw], sparse(eye(size(Wl,1)))];

% Feasibility contrraints
% U2 = diag(drv_gns);
load_inertia = [pi_load_unknw(1), pi_load_unknw(2), pi_load_unknw(3); ...
                pi_load_unknw(2), pi_load_unknw(4), pi_load_unknw(5); ...
                pi_load_unknw(3), pi_load_unknw(5), pi_load_unknw(6)];
                  
load_frst_mmnt = vec2skewSymMat(pi_load_unknw(7:9));
    
D = [load_inertia, load_frst_mmnt'; load_frst_mmnt, m_load*eye(3)];
% U2 = blkdiag(U2,D);
% Overall constraints
% cnstr = blkdiag(U1,U2)>0;
cnstr = [drv_gns>0, D>0];

% Objective function
obj = norm(-Wl(:,end)*m_load - [-(I_ldd-I_uldd) Wl(:,1:9)]*[drv_gns;pi_load_unknw]);

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));

%}

% ------------------------------------------------------------------------
% Computing drive gains using second method
% ------------------------------------------------------------------------
drv_gns = sdpvar(6,1); % variables for base paramters
pi_load_unknw = sdpvar(9,1); % varaibles for unknown load paramters
pi_b = sdpvar(36,1); % variables for base paramters
pi_d = sdpvar(24,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = [Pb' Pd']*[ eye(36) -double(Kd); zeros(24,36) eye(24)]*[pi_b; pi_d];

% Feasibility contrraints
cnstr = drv_gns>0;
for i = 1:10:60
    link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                      pii(i+1), pii(i+3), pii(i+4); ...
                      pii(i+2), pii(i+4), pii(i+5)];
                  
    frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));
    
    Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
    cnstr = [cnstr, Di>0];
end

load_inertia = [pi_load_unknw(1), pi_load_unknw(2), pi_load_unknw(3); ...
                pi_load_unknw(2), pi_load_unknw(4), pi_load_unknw(5); ...
                pi_load_unknw(3), pi_load_unknw(5), pi_load_unknw(6)];
                  
load_frst_mmnt = vec2skewSymMat(pi_load_unknw(7:9));
    
Dl = [load_inertia, load_frst_mmnt'; load_frst_mmnt, m_load*eye(3)];
cnstr = [cnstr, Dl>0];

t1 = [zeros(size(I_uldd,1),1); -Wl(:,end)*m_load];
t2 = [-I_uldd, W, zeros(size(W,1), size(Wl,2)-1); ...
      -I_ldd, W, Wl(:,1:9) ];
obj = norm(t1 - t2*[drv_gns;pi_b;pi_load_unknw]);

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));
return
% ------------------------------------------------------------------------
% Least square
% ------------------------------------------------------------------------
pir_hat = (W'*W)\(W'*Tau);

% ------------------------------------------------------------------------
% Ridge regression
% ------------------------------------------------------------------------
% pi_hat = mldivide(W'*W + 0.001*eye(size(W'*W)),W'*tau);

% ------------------------------------------------------------------------
% Using SDP to find inertial parameters
% ------------------------------------------------------------------------
pi_b = sdpvar(36,1); % variables for base paramters
pi_d = sdpvar(24,1); % variables for dependent paramters
u = sdpvar(1,1); % variable for cost

% Writing |Tau - W*pi_b| using Schur compliment
U1 = [u, (Tau - W*pi_b)'; Tau - W*pi_b, eye(size(W,1))];

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = [Pb' Pd']*[ eye(36) -double(Kd); zeros(24,36) eye(24)]*[pi_b; pi_d];

% Feasibility contrraints
U2 = [];
for i = 1:10:60
    link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                      pii(i+1), pii(i+3), pii(i+4); ...
                      pii(i+2), pii(i+4), pii(i+5)];
                  
    frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));
    
    Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
    U2 = blkdiag(U2,Di);
end
% Overall constraints
cnstr = blkdiag(U1,U2)>0;

% Objective function
obj = u;

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));

% Getting numerical values of optimization paramters
base_prms = value(pi_b);
dpndn_prms = value(pi_d);





return
% ------------------------------------------------------------------------
% DREM
% ------------------------------------------------------------------------
alpha = [1, 1.3, 1.6, 1.3, 1.15, 1.5];
beta = [1, 1.25, 1.5, 1.75, 2, 2.25];
dly = [3, 6, 9, 12, 15];

% tau(:,1) = base_regressor(q(:,1),qd(:,1),q2d(:,1))*pir_base_vctr;
% tau_f1(:,1) = tau(:,1); tau_f2(:,1) = tau(:,1); tau_f3(:,1) = tau(:,1);
% tau_f4(:,1) = tau(:,1); tau_f5(:,1) = tau(:,1); tau_f6(:,1) = tau(:,1);
% 
% for i = 2:length(traj_par.t)
%     tau(:,i) = base_regressor(q(:,i),qd(:,i),q2d(:,i))*pir_base_vctr;
%     for j = 1:6
%        tau_f1(j,i) = low_pass_filter(tau(j,i),tau_f1(j,i-1),alpha(1),beta(1),dt); 
%        tau_f2(j,i) = low_pass_filter(tau(j,i),tau_f2(j,i-1),alpha(2),beta(2),dt); 
%        tau_f3(j,i) = low_pass_filter(tau(j,i),tau_f3(j,i-1),alpha(3),beta(3),dt); 
%        tau_f4(j,i) = low_pass_filter(tau(j,i),tau_f4(j,i-1),alpha(4),beta(4),dt); 
%        tau_f5(j,i) = low_pass_filter(tau(j,i),tau_f5(j,i-1),alpha(5),beta(5),dt); 
%     end
% end
% tau_e1 = vertcat(tau,tau_f1,tau_f2,tau_f3,tau_f4,tau_f5);

for i = 1:length(traj_par.t)
    tau(:,i) = base_regressor(q(:,i),qd(:,i),q2d(:,i))*pir_base_vctr;
    if i>dly(1); tau_f1(:,i) = tau(:,i-dly(1)); else; tau_f1(:,i) = zeros(6,1); end
    if i>dly(2); tau_f2(:,i) = tau(:,i-dly(2)); else; tau_f2(:,i) = zeros(6,1); end
    if i>dly(3); tau_f3(:,i) = tau(:,i-dly(3)); else; tau_f3(:,i) = zeros(6,1); end
    if i>dly(4); tau_f4(:,i) = tau(:,i-dly(4)); else; tau_f4(:,i) = zeros(6,1); end
    if i>dly(5); tau_f5(:,i) = tau(:,i-dly(5)); else; tau_f5(:,i) = zeros(6,1); end
end
tau_e2 = vertcat(tau,tau_f1,tau_f2,tau_f3,tau_f4,tau_f5);

%{
jnt = 4;
figure
plot(traj_par.t,tau(jnt,:))
hold on
plot(traj_par.t,tau_f1(jnt,:))
plot(traj_par.t,tau_f2(jnt,:))
plot(traj_par.t,tau_f3(jnt,:))
plot(traj_par.t,tau_f4(jnt,:))
plot(traj_par.t,tau_f5(jnt,:))
%}

% Y(:,:,1) = base_regressor(q(:,1),qd(:,1),q2d(:,1));
% Y_f1(:,:,1) = Y(:,:,1); Y_f2(:,:,1) = Y(:,:,1); Y_f3(:,:,1) = Y(:,:,1);
% Y_f4(:,:,1) = Y(:,:,1); Y_f5(:,:,1) = Y(:,:,1); Y_f6(:,:,1) = Y(:,:,1);
% for i = 2:length(traj_par.t)
%    Y(:,:,i) = base_regressor(q(:,i),qd(:,i),q2d(:,i));
%    for j = 1:6
%        for k = 1:36
%            Y_f1(j,k,i) = low_pass_filter(Y(j,k,i),Y_f1(j,k,i-1),alpha(1),beta(1),dt);
%            Y_f2(j,k,i) = low_pass_filter(Y(j,k,i),Y_f2(j,k,i-1),alpha(2),beta(2),dt);
%            Y_f3(j,k,i) = low_pass_filter(Y(j,k,i),Y_f3(j,k,i-1),alpha(3),beta(3),dt);
%            Y_f4(j,k,i) = low_pass_filter(Y(j,k,i),Y_f4(j,k,i-1),alpha(4),beta(4),dt);
%            Y_f5(j,k,i) = low_pass_filter(Y(j,k,i),Y_f5(j,k,i-1),alpha(5),beta(5),dt);
%        end
%    end
%     
% end
% Y_e1 = vertcat(Y, Y_f1, Y_f2, Y_f3, Y_f4, Y_f5);

for i = 1:length(traj_par.t)
   Y(:,:,i) = base_regressor(q(:,i),qd(:,i),q2d(:,i));
   if i>dly(1); Y_f1(:,:,i) = Y(:,:,i-dly(1)); else; Y_f1(:,:,i) = zeros(6,36); end
   if i>dly(2); Y_f2(:,:,i) = Y(:,:,i-dly(2)); else; Y_f2(:,:,i) = zeros(6,36); end
   if i>dly(3); Y_f3(:,:,i) = Y(:,:,i-dly(3)); else; Y_f3(:,:,i) = zeros(6,36); end
   if i>dly(4); Y_f4(:,:,i) = Y(:,:,i-dly(4)); else; Y_f4(:,:,i) = zeros(6,36); end
   if i>dly(5); Y_f5(:,:,i) = Y(:,:,i-dly(5)); else; Y_f5(:,:,i) = zeros(6,36); end
end
Y_e2 = vertcat(Y, Y_f1, Y_f2, Y_f3, Y_f4, Y_f5);

%{
row_Y = 1;
column_Y = 8;
figure
plot(traj_par.t, reshape(Y(row_Y,column_Y,:),[size(Y,3),1]))
hold on
plot(traj_par.t, reshape(Y_f1(row_Y,column_Y,:),[size(Y_f1,3),1]))
plot(traj_par.t, reshape(Y_f2(row_Y,column_Y,:),[size(Y_f2,3),1]))
plot(traj_par.t, reshape(Y_f3(row_Y,column_Y,:),[size(Y_f3,3),1]))
plot(traj_par.t, reshape(Y_f4(row_Y,column_Y,:),[size(Y_f4,3),1]))
plot(traj_par.t, reshape(Y_f5(row_Y,column_Y,:),[size(Y_f5,3),1]))
%}

gma1 = 1e-2;
pi_drem(:,1) = zeros(36,1);
for i = 2:length(traj_par.t)
   Yei = Y_e2(:,:,i);
   tauei = tau_e2(:,i);
   
   adjYei =  adjoint(Yei);
   detYei = det(Yei);
   YYi = adjYei*tauei;
   erri = YYi - detYei*pi_drem(:,i-1);
   gradi = detYei/(gma1 + detYei^2)*erri;
   pi_drem(:,i) = pi_drem(:,i-1) + gma1*gradi;
end

figure
hold on
for i = 1:5
    plot(traj_par.t,pi_drem(i,1:end))
end

return


% ------------------------------------------------------------------------
% Low pass filter for DREM
% ------------------------------------------------------------------------
function y_cur = low_pass_filter(u_cur,y_prev,alpha,beta,T)
    t1 = 1+beta*T;
    t2 = alpha*T;
    y_cur = 1/t1*y_prev + t2/t1*u_cur;
end




