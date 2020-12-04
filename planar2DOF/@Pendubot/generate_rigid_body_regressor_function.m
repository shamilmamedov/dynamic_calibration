function generate_rigid_body_regressor_function(obj)
% -----------------------------------------------------------------------
% The function generates regressor for planar 2 degrees of freedom
% manipulator. It assumes that links of the robot are rigid bodies.
% Thus the algorithm return rigid body dynamics of the robot in 
% regressor form.
% -----------------------------------------------------------------------
plnr = obj.description;

% Symbolic generilized coordiates, their first and second deriatives
q_sym = sym('q%d',[2,1],'real');
qd_sym = sym('qd%d',[2,1],'real');
q2d_sym = sym('q2d%d',[2,1],'real');


% Getting energy functions, to derive dynamics
T_pk = sym(zeros(4,4,2)); % transformation between links
w_kk(:,1) = sym(zeros(3,1)); % angular velocity k in frame k
v_kk(:,1) = sym(zeros(3,1)); % linear velocity of the origin of frame k in frame k
g_kk(:,1) = sym([0,0,9.81])'; % vector of graviatational accelerations in frame k
p_kk(:,1) = sym(zeros(3,1)); % origin of frame k in frame k

for i = 1:2
    jnt_axs_k = str2num(plnr.robot.joint{i}.axis.Attributes.xyz)';
    % Transformation from parent link frame p to current joint frame
    rpy_k = sym(str2num(plnr.robot.joint{i}.origin.Attributes.rpy));
    R_pj = RPY(rpy_k);
    R_pj(abs(R_pj)<1e-5) = sym(0); % to avoid numerical errors
    p_pj = str2num(plnr.robot.joint{i}.origin.Attributes.xyz)';
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
    
    K_reg(i,:) = [sym(0.5)*w2wtlda(w_kk(:,i+1)),...
                    v_kk(:,i+1)'*vec2skewSymMat(w_kk(:,i+1)),...
                    sym(0.5)*v_kk(:,i+1)'*v_kk(:,i+1)];
    P_reg(i,:) = [sym(zeros(1,6)), g_kk(:,i+1)',...
                    g_kk(:,i+1)'*p_kk(:,i+1)];
end

% Compose lagrangian
Lagr = [K_reg(1,:) - P_reg(1,:), K_reg(2,:) - P_reg(2,:)];

% Use Lagrange equations to derive regressor matrix
dLagr_dq = jacobian(Lagr,q_sym)';
dLagr_dqd = jacobian(Lagr,qd_sym)';
t1 = sym(zeros(2,20));
for i = 1:2
   t1 = t1 + diff(dLagr_dqd,q_sym(i))*qd_sym(i)+...
                diff(dLagr_dqd,qd_sym(i))*q2d_sym(i);
end
Y = simplify(t1 - dLagr_dq);

matlabFunction(Y,'File','planar2DOF/@Pendubot/get_standard_rigid_body_regressor',...
                     'Vars',{q_sym, qd_sym, q2d_sym});