function generate_rb_dynamics(path_to_urdf)
% ----------------------------------------------------------------------
% Function generates rigid body dynamics equations, namely M, C matrices
% and g vector: M(q) ddq + C(q, dq) dq + g(q) = tau
% ----------------------------------------------------------------------

% Parse urdf to get robot description
ur10 = parse_urdf(path_to_urdf);

% Create symbolic generilized coordiates, their first and second deriatives
q_sym = sym('q%d',[6,1],'real');
qd_sym = sym('qd%d',[6,1],'real');
q2d_sym = sym('q2d%d',[6,1],'real');

% ------------------------------------------------------------------------
% Getting gradient of energy functions, to derive dynamics
% ------------------------------------------------------------------------
T_pk = sym(zeros(4,4,6)); % transformation between links
w_kk(:,1) = sym(zeros(3,1)); % angular velocity k in frame k
v_kk(:,1) = sym(zeros(3,1)); % linear velocity of the origin of frame k in frame k
g_kk(:,1) = sym([0,0,9.81])'; % vector of graviatational accelerations in frame k
p_kk(:,1) = sym(zeros(3,1)); % origin of frame k in frame k

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
        
    beta_K(i,:) = [sym(0.5)*w2wtlda(w_kk(:,i+1)),...
                   v_kk(:,i+1)'*vec2skewSymMat(w_kk(:,i+1)),...
                   sym(0.5)*v_kk(:,i+1)'*v_kk(:,i+1)];
    beta_P(i,:) = [sym(zeros(1,6)), g_kk(:,i+1)',...
                   g_kk(:,i+1)'*p_kk(:,i+1)];
end

beta_Lf = [beta_K(1,:) - beta_P(1,:), beta_K(2,:) - beta_P(2,:),...
         beta_K(3,:) - beta_P(3,:), beta_K(4,:) - beta_P(4,:),...
         beta_K(5,:) - beta_P(5,:), beta_K(6,:) - beta_P(6,:)];

% Lagrangian dynamics
pi_sndrd_sym = sym('pi%d%d', [60,1], 'real'); % standard parameters
Lagr = beta_Lf*pi_sndrd_sym; % Lagrangian of the system
P = [beta_P(1,:), beta_P(2,:), beta_P(3,:), beta_P(4,:),...
     beta_P(5,:), beta_P(6,:)]*pi_sndrd_sym; % Potential energy
 
dLagr_dqd = jacobian(Lagr, qd_sym)';

% Get inertia matrix M and gravity vector G
M_mtrx_sym = jacobian(dLagr_dqd, qd_sym);  
G_vctr_sym = jacobian(P, q_sym)';

% Get velocity matrix C using Christoffel symbols of the first kind
cs1 = sym(zeros(6,6,6)); 
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

% Generate functions
matlabFunction(M_mtrx_sym, 'File','autogen/M_mtrx_fcn',...
               'Vars',{q_sym, pi_sndrd_sym}, 'Optimize', true);               
matlabFunction(C_mtrx_sym, 'File','autogen/C_mtrx_fcn',...
               'Vars',{q_sym, qd_sym, pi_sndrd_sym}, 'Optimize', false);
matlabFunction(G_vctr_sym, 'File','autogen/G_vctr_fcn',...
               'Vars',{q_sym, pi_sndrd_sym}, 'Optimize', true);

