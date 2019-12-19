% -----------------------------------------------------------------------
% 
% Getting base parameters of the UR10 manipulator based on the 
% 
% Gautier, M., & Khalil, W. (1990). Direct calculation of minimum set 
% of inertial parameters of serial robots. IEEE Transactions on Robotics 
% and Automation, 6(3), 368–373. https://doi.org/10.1109/70.56655
% 
% Here we tried to generilize what was given there to more general
% parametrization compared to their modified DH parameters.
% ------------------------------------------------------------------------
% Get robot description
run('main_ur.m')

generateBaseRegressorFunction = 0;
generateBaseDynamicsFunctions = 0;

% Create symbolic parameters
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
    pi_ur10(:,i) = [ixx(i),ixy(i),ixz(i),iyy(i),iyz(i),izz(i),...
                        hx(i),hy(i),hz(i),m(i)]';
end

% Symbolic generilized coordiates, their first and second deriatives
q_sym = sym('q%d',[6,1],'real');
qd_sym = sym('qd%d',[6,1],'real');
q2d_sym = sym('q2d%d',[6,1],'real');

% ------------------------------------------------------------------------
% Getting energy functions, to derive dynamics
% ------------------------------------------------------------------------
T_pk = sym(zeros(4,4,6)); % transformation between links
w_kk(:,1) = sym(zeros(3,1)); % angular velocity k in frame k
v_kk(:,1) = sym(zeros(3,1)); % linear velocity of the origin of frame k in frame k
g_kk(:,1) = sym([0,0,9.81])'; % vector of graviatational accelerations in frame k
p_kk(:,1) = sym(zeros(3,1)); % origin of frame k in frame k
DK(:,1) = sym(zeros(10,1)); % gradient of kinetic energy
DP(:,1) = sym([zeros(1,6),[0,0,9.81],0]'); % gradient of gravitational energy

for i = 1:6
    jnt_axs_k = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
% Transformation from parent link frame p to current joint frame
    rpy_k = sym(str2num(ur10.robot.joint{i}.origin.Attributes.rpy));
    R_pj = RPY(rpy_k);
    R_pj(abs(R_pj)<sqrt(eps)) = sym(0); % to avoid numerical errors
    p_pj = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
    T_pj = sym([R_pj, p_pj; zeros(1,3), 1]); % to avoid numerical errors
% Tranformation from joint frame of the joint that rotaties body k to
% link frame. The transformation is pure rotation
    R_jk = Rot(q_sym(i),sym(jnt_axs_k));
    p_jk = sym(zeros(3,1));
    T_jk = [R_jk, p_jk; sym(zeros(1,3)),sym(1)];
% Transformation from parent link frame p to current link frame k
    T_pk(:,:,i) = T_pj*T_jk;
    z_kk(:,i) = sym(jnt_axs_k);
        
    w_kk(:,i+1) = T_pk(1:3,1:3,i)'*w_kk(:,i) + sym(jnt_axs_k)*qd_sym(i);
    v_kk(:,i+1) = T_pk(1:3,1:3,i)'*(v_kk(:,i) + cross(w_kk(:,i),sym(p_pj)));
    g_kk(:,i+1) = T_pk(1:3,1:3,i)'*g_kk(:,i);
    p_kk(:,i+1) = T_pk(1:3,1:3,i)'*(p_kk(:,i) + sym(p_pj));
                           
    lmda(:,:,i) = getLambda(T_pk(1:3,1:3,i),sym(p_pj));
    f(:,i) = getF(v_kk(:,i+1),w_kk(:,i+1),sym(jnt_axs_k),qd_sym(i));
    DK(:,i+1) = lmda(:,:,i)*DK(:,i) + qd_sym(i)*f(:,i);
    DP(:,i+1) = lmda(:,:,i)*DP(:,i);
end


% -----------------------------------------------------------------------
% Regrouping parameters of the links
% -----------------------------------------------------------------------
pir_ur10_num = zeros(10,6);
pir_ur10_num(:,6) = ur10.pi(:,6);

pir_ur10_sym = sym(zeros(10,6));
pir_ur10_sym(:,6) = pi_ur10(:,6);
for i = 6:-1:2
    pi_prv_num = ur10.pi(:,i-1);
    [pir_ur10_num(:,i-1),pir_ur10_num(:,i)] = ...
            group(pi_prv_num,pir_ur10_num(:,i),lmda(:,:,i),z_kk(:,i));
    
    pi_prv_sym = pi_ur10(:,i-1);
    [pir_ur10_sym(:,i-1),pir_ur10_sym(:,i)] = ...
            group(pi_prv_sym, pir_ur10_sym(:,i),lmda(:,:,i),z_kk(:,i));
end
pir_ur10_num = double(pir_ur10_num);


% -----------------------------------------------------------------------
% Getting base parameters
% -----------------------------------------------------------------------
grad_Lagr = [];
pir_base_vctr = [];
nb = 0; % number of independent parameters
nd = 0; % number of non-identifiable or indeitifiable in combination
Pb = zeros(36,60);
Pd = zeros(24,60);
for i = 1:6
    DK_base{i} = sym([]);
    DP_base{i} = sym([]);
    pir_base_sym{i} = sym([]);
    pir_base_num{i} = [];
    for j = 1:10
%       We try to obtain base parameters, parameters tha affect energy
%       of the system, by anlayzing kinetic energy and potenial energy
%       as well as vector of regrouped parameters
% 
%       first condition says that if DK(j,i+1) and DP(j,i+1) are zero
%       or DK(j,i+1) is zero and DP(j,i+1) is constant
%       the second condition says that if we have regrouped some parmeter
%       with parameters of previous link so it is zero now
        if (DK(j,i+1) == 0 && (DP(j,i+1)==0 || isempty(symvar(DP(j,i+1)))))...
                || pir_ur10_sym(j,i) == 0
            str = strcat('\nlink\t ',num2str(i), ',\tparameter\t',num2str(j));
            fprintf(str)
            nd = nd + 1;
            Pd(nd,(i-1)*10 + j) = 1;
        else
            nb = nb + 1;
            DK_base{i} = vertcat(DK_base{i},DK(j,i+1));
            DP_base{i} = vertcat(DP_base{i},DP(j,i+1));
            
            pir_base_sym{i} = vertcat(pir_base_sym{i},pir_ur10_sym(j,i));
            pir_base_num{i} = vertcat(pir_base_num{i},pir_ur10_num(j,i));
            Pb(nb,(i-1)*10 + j) = 1;
        end
    end
%     -------------------------------------------------------------------
%     DK_base{i} = vertcat(DK_base{i}, 0.5*qd_sym(i)^2);
%     DP_base{i} = vertcat(DP_base{i}, 0);
%     pir_base_sym{i} = vertcat(pir_base_sym{i}, im(i));
%     -------------------------------------------------------------------
    fprintf('\n\n')
    grad_Lagr = horzcat(grad_Lagr, (DK_base{i} - DP_base{i})');
    pir_base_vctr = vertcat(pir_base_vctr,pir_base_num{i});
end


% -----------------------------------------------------------------------
% Computing Regressor
% -----------------------------------------------------------------------
dLagr_dq = jacobian(grad_Lagr,q_sym)';
dLagr_dqd = jacobian(grad_Lagr,qd_sym)';
t1 = sym(zeros(6,length(pir_base_vctr)));
for i = 1:6
   t1 = t1 + diff(dLagr_dqd,q_sym(i))*qd_sym(i)+...
                diff(dLagr_dqd,qd_sym(i))*q2d_sym(i);
end
Y_hat = t1 - dLagr_dq;

if generateBaseRegressorFunction
    matlabFunction(Y_hat,'File','autogen/base_regressor_UR10E',...
                   'Vars',{q_sym,qd_sym,q2d_sym});
end

% ------------------------------------------------------------
% Finding mapping from standard parameters to base parameters
% ------------------------------------------------------------
pi_ur10_full = reshape(pi_ur10,[60,1]);
pi_ur10_rdcd = reshape(pir_ur10_sym,[60,1]);
pi_ur10_base = [pir_base_sym{1};pir_base_sym{2};pir_base_sym{3};...
                pir_base_sym{4};pir_base_sym{5};pir_base_sym{6}];
dbase_dfull = jacobian(pi_ur10_base,pi_ur10_full);
Kd = jacobian(pi_ur10_base - Pb*pi_ur10_full, Pd*pi_ur10_full);


% --------------------------------------------------------------------
% Computing matrices of dynamic equations of motion 
% --------------------------------------------------------------------
% Complement gradient Lagrangian with rotor contribution
grad_Lagr = [grad_Lagr, 0.5*qd_sym(3)^2, 0.5*qd_sym(4)^2,...
                0.5*qd_sym(5)^2, 0.5*qd_sym(6)^2];
% Gradient of potential energy to find gravity vector
grad_P = [DP_base{1}', DP_base{2}', DP_base{3}', DP_base{4}',...
            DP_base{5}', DP_base{6}', 0, 0, 0, 0];
% Create symbolic variable with base parameters of the robot
xi = sym('xi%d',[40,1],'real');
% Lagrangian
Lagr = grad_Lagr*xi;
%  Potential energy
Pot_enrgy = grad_P*xi;

% Solving Lagrange equations to find
dLagr_dq = jacobian(Lagr,q_sym)';
dLagr_dqd = jacobian(Lagr,qd_sym)';
M_mtrx_sym = jacobian(dLagr_dqd, qd_sym);
G_vctr_sym = jacobian(Pot_enrgy,q_sym)';

% Finding matrix of Centrifugal and Coriolis forces
cs1 = sym(zeros(6,6,6)); % Christoffel symbols of the first kind
for i = 1:1:6
    for j = 1:1:6
       for k = 1:1:6
          cs1(i,j,k) = 0.5*(diff(M_mtrx_sym(i,j), q_sym(k)) + ...
                          diff(M_mtrx_sym(i,k), q_sym(j)) - ...
                          diff(M_mtrx_sym(j,k), q_sym(i)));
       end
    end
end

C_mtrx_sym = sym(zeros(6, 6));
for i = 1:1:6
    for j = 1:1:6
        for k = 1:1:6
            C_mtrx_sym(i,j) = C_mtrx_sym(i,j)+cs1(i,j,k)*qd_sym(k);
        end
    end
end

if generateBaseDynamicsFunctions
    fprintf('Generating dynamic equation elements:\n');

    fprintf('\t Mass matrix\n');
    matlabFunction(M_mtrx_sym,'File','autogen/M_mtrx_fcn','Vars',{q_sym,xi});

    fprintf('\t Matrix of Coriolis and Centrifugal Forces\n');
    matlabFunction(C_mtrx_sym,'File','autogen/C_mtrx_fcn','Vars',{q_sym,qd_sym,xi});

    fprintf('\t Vector of gravitational forces\n');
    matlabFunction(G_vctr_sym,'File','autogen/G_vctr_fcn','Vars',{q_sym,xi});
end

return
% --------------------------------------------------------------------
% Tests
% --------------------------------------------------------------------
q = rand(6,1);
qd = rand(6,1);
q2d = rand(6,1);

rbt = importrobot('ur10e.urdf');
rbt.DataFormat = 'column';
rbt.Gravity = [0 0 -9.81];

id_matlab = inverseDynamics(rbt,q,qd,q2d);
id_gutier = base_regressor(q,qd,q2d)*pir_base_vctr;
id_screwreg = screw_regressor(q,qd,q2d,ur10)*reshape(ur10.pi_reg,[60,1]);
id_lagrreg = full_regressor(q,qd,q2d)*reshape(ur10.pi,[60,1]);

