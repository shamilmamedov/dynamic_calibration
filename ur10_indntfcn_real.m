% ------------------------------------------------------------------------
% Load data and procces it (filter and estimate accelerations)
% ------------------------------------------------------------------------
run('data_prcsng.m')
% run('data_pltng.m')

% ------------------------------------------------------------------------
% Generate Regressors based on data
% ------------------------------------------------------------------------
% Load matrices that map standard set of paratmers to base parameters
load('full2base_mapping.mat');
m_load = 1.069;

%Constracting regressor matrix
Wb_uldd = []; I_uldd = []; 
for i = 1:2:length(t_msrd)
    Yb_ulddi = base_regressor_UR10E(q_msrd(i,:)',...
                qd_fltrd(i,:)',q2d_est(i,:)');
    Yfrctni = ur10_frctn_rgsr(qd_fltrd(i,:)');
    Ydrvi = ur10_drv_rgsr(q2d_est(i,:)');
    Wb_uldd = vertcat(Wb_uldd,[Yb_ulddi, Ydrvi, Yfrctni]);
    I_uldd = vertcat(I_uldd, diag(i_fltrd(i,:)));
end

Wb_ldd = []; Wl = []; I_ldd = [];
for i = 1:2:length(t_msrd_ldd)
    Yb_lddi = base_regressor_UR10E(q_msrd_ldd(i,:)',...
                qd_fltrd_ldd(i,:)',q2d_est_ldd(i,:)');
    Yfrctni = ur10_frctn_rgsr(qd_fltrd_ldd(i,:)');
    Ydrvi = ur10_drv_rgsr(q2d_est_ldd(i,:)');
    
    Yli = load_regressor_UR10E(q_msrd_ldd(i,:)',...
                qd_fltrd_ldd(i,:)',q2d_est_ldd(i,:)');
    
    Wb_ldd = vertcat(Wb_ldd,[Yb_lddi, Ydrvi, Yfrctni]);
    Wl = vertcat(Wl,Yli); 
    I_ldd = vertcat(I_ldd, diag(i_fltrd_ldd(i,:)));
end

% ----------------------------------------------------------------------
% Set-up SDP optimization procedure
% -----------------------------------------------------------------------
drv_gns = sdpvar(6,1); % variables for base paramters
pi_load_unknw = sdpvar(9,1); % varaibles for unknown load paramters
pi_frctn = sdpvar(18,1);
pi_rtr = sdpvar(4,1);
pi_b = sdpvar(36,1); % variables for base paramters
pi_d = sdpvar(24,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = [Pb' Pd']*[ eye(36) -Kd; zeros(24,36) eye(24) ]*[pi_b; pi_d];

% Feasibility contrraints of the link paramteres
cnstr = diag(drv_gns)>0;
for i = 1:10:60
    link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                      pii(i+1), pii(i+3), pii(i+4); ...
                      pii(i+2), pii(i+4), pii(i+5)];
                  
    frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));
    
    Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
    cnstr = [cnstr, Di>0];
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

% Feasibiliy of the rotor inertia
cnstr = [cnstr, diag(pi_rtr)>0];

% Defining pbjective function
t1 = [zeros(size(I_uldd,1),1); -Wl(:,end)*m_load];

t2 = [-I_uldd, Wb_uldd, zeros(size(Wb_uldd,1), size(Wl,2)-1); ...
      -I_ldd, Wb_ldd, Wl(:,1:9) ];
  
obj = norm(t1 - t2*[drv_gns; pi_b; pi_rtr; pi_frctn; pi_load_unknw]);

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));

% Getting values of the estimated patamters
drv_gns = value(drv_gns);

% -----------------------------------------------------------------------
% When drive gains are known we optimize for paramters
% -----------------------------------------------------------------------
%Constracting regressor matrix
Wb_uldd = []; Tau_uldd = []; 
for i = 1:6:length(t_msrd)
    Yb_ulddi = base_regressor_UR10E(q_msrd(i,:)',...
                qd_fltrd(i,:)',q2d_est(i,:)');
    Yfrctni = ur10_frctn_rgsr(qd_fltrd(i,:)');
    Ydrvi = ur10_drv_rgsr(q2d_est(i,:)');
    Wb_uldd = vertcat(Wb_uldd,[Yb_ulddi, Ydrvi, Yfrctni]);
    Tau_uldd = vertcat(Tau_uldd, diag(drv_gns)*i_fltrd(i,:)');
end

pi_frctn = sdpvar(18,1);
pi_rtr = sdpvar(4,1);
pi_b = sdpvar(36,1); % variables for base paramters
pi_d = sdpvar(24,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = [Pb' Pd']*[ eye(36) -Kd; zeros(24,36) eye(24) ]*[pi_b; pi_d];

% Feasibility contrraints of the link paramteres
cnstr = [];
for i = 1:10:60
    link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                      pii(i+1), pii(i+3), pii(i+4); ...
                      pii(i+2), pii(i+4), pii(i+5)];
                  
    frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));
    
    Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
    cnstr = [cnstr, Di>0];
end

% Feasibility constraints on the friction prameters 
for i = 1:6
   cnstr = [cnstr, pi_frctn(3*i-2)>0, pi_frctn(3*i-1)>0];  
end

% Feasibiliy of the rotor inertia
cnstr = [cnstr, diag(pi_rtr)>0];

% Defining pbjective function
obj = norm(Tau_uldd - Wb_uldd*[pi_b; pi_rtr; pi_frctn]);

% Solving sdp problem
sol2 = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));


pi_frctn = value(pi_frctn);
pi_rtr = value(pi_rtr);
pi_b = value(pi_b); % variables for base paramters

return

% ------------------------------------------------------------------------
% Using SDP to find load parmeters along with drive gains
% ------------------------------------------------------------------------
%{
%Constracting regressor matrix
Wl = []; I_uldd = []; I_ldd = [];
for i = 1:length(t_msrd_ldd)
    Yli = load_regressor_UR10E(q_msrd_ldd(i,:)',...
                qd_fltrd_ldd(i,:)',q2d_est_ldd(i,:)');
    Wl = vertcat(Wl,Yli);
    I_uldd = vertcat(I_uldd, diag(i_fltrd(i,:)));
    I_ldd = vertcat(I_ldd, diag(i_fltrd_ldd(i,:)));
end

m_load = 1.069;
drv_gns = sdpvar(6,1); % variables for base paramters
pi_load_unknw = sdpvar(9,1); % varaibles for unknown load paramters

% Feasibility contrraints
load_inertia = [pi_load_unknw(1), pi_load_unknw(2), pi_load_unknw(3); ...
                pi_load_unknw(2), pi_load_unknw(4), pi_load_unknw(5); ...
                pi_load_unknw(3), pi_load_unknw(5), pi_load_unknw(6)];
            
load_frst_mmnt = vec2skewSymMat(pi_load_unknw(7:9));
    
D = [load_inertia, load_frst_mmnt'; load_frst_mmnt, m_load*eye(3)];

% Overall Constraints
cnstr = [drv_gns>0, D>0];

% Objective function
t1 = I_ldd - I_uldd(1:length(I_ldd),:);
t2 = [-t1, Wl(:,1:9)]*[drv_gns;pi_load_unknw];
obj = norm(-Wl(:,end)*m_load - t2);

% Solving sdp problem
sol = optimize(cnstr,obj,sdpsettings('solver','sdpt3'));
%}