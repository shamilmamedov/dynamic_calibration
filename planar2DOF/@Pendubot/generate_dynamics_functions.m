function generate_dynamics_functions(obj)
% The function derives dynamics of the pendubot in symbolic form
% using Largrange-Euler notation
%
% pd - pendubot description that comes from urdf file
pd = obj.description;


% Symbolic generilized coordiates, their first and second deriatives
q_sym = sym('q%d',[2,1],'real');
qd_sym = sym('qd%d',[2,1],'real');
q2d_sym = sym('q2d%d',[2,1],'real');

% Dynamics of the robot in classical form for control
T_0k(:,:,1) = sym(eye(4)); % homogenous transformation matrices
Jv_0k = sym(zeros(3,2,2)); % translational jacobians
Jw_0k = sym(zeros(3,2,2)); % rotational jacobians
K = sym(0); P = sym(0); % kinetic and potential energy
z_0k = sym(zeros(3,2)); % axes of rotation in inertial frame
r_0k = sym(zeros(3,2)); % center of mass in inertial frame
for i = 1:2
        % Transformation from parent link frame p to current joint frame
        R_pj = sym(RPY(str2num(pd.robot.joint{i}.origin.Attributes.rpy)));
        p_pj = sym(str2num(pd.robot.joint{i}.origin.Attributes.xyz))';
        R_pj(abs(R_pj)<1e-5) = sym(0);
        T_pj = [R_pj, p_pj; sym(zeros(1,3)), sym(1)];
        % Tranformation from joint frame of the joint that rotates body k to
        % link frame. The transformation is pure rotation
        R_jk = Rot(q_sym(i), pd.k(:,i));
        p_jk = sym(zeros(3,1));
        T_jk = [R_jk, p_jk; sym(zeros(1,3)),sym(1)];
        % Transformation from parent link frame p to current link frame k
        T_pk(:,:,i) = T_pj*T_jk;
        T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
        z_0k(:,i) = T_0k(1:3,1:3,i+1)*pd.k(:,i);
        
        r_0k(:,i) = sym([eye(3),zeros(3,1)])*...
                        T_0k(:,:,i+1)*[pd.r_com(:,i);sym(1)];
        
        for j = 1:i
           Jv_0k(:,j,i) = cross(z_0k(:,j),r_0k(:,i)-T_0k(1:3,4,j+1));
           Jw_0k(:,j,i) = z_0k(:,j);
        end
        
        K = K + sym(0.5)*qd_sym'*(pd.m(i)*(Jv_0k(:,:,i)'*Jv_0k(:,:,i)) + ...
                Jw_0k(:,:,i)'*T_0k(1:3,1:3,i+1)*pd.I(:,:,i)*...
                T_0k(1:3,1:3,i+1)'*Jw_0k(:,:,i))*qd_sym;
        P = P + pd.m(i)*sym([0;0;9.81])'*r_0k(:,i);
end

Lagr = K - P; % Lagrangian
dLagr_dq = jacobian(Lagr,q_sym)';
dLagr_dqd = jacobian(Lagr,qd_sym)';
t1 = jacobian(dLagr_dqd,[q_sym;qd_sym])*[qd_sym; q2d_sym];
t1 = simplify(t1);
dnmcs = t1 - dLagr_dq;
M_sym = jacobian(dnmcs,q2d_sym);
g_sym = jacobian(P, q_sym)';
n_sym = simplify(dnmcs - M_sym*q2d_sym);

% compute C matrix using Christoffel symbols
cs1 = sym(zeros(2,2,2)); % Christoffel symbols of the first kind
for i = 1:1:2
    for j = 1:1:2
       for k = 1:1:2
          cs1(i,j,k) = 0.5*(diff(M_sym(i,j), q_sym(k)) + ...
                            diff(M_sym(i,k), q_sym(j)) - ...
                            diff(M_sym(j,k), q_sym(i)));
       end
    end
end

C_sym = sym(zeros(2, 2));
for i = 1:1:2
    for j = 1:1:2
        for k = 1:1:2
            C_sym(i,j) = C_sym(i,j)+cs1(i,j,k)*qd_sym(k);
        end
    end
end

matlabFunction(M_sym,'File','planar2DOF/@Pendubot/get_M','Vars',{q_sym});
matlabFunction(n_sym,'File','planar2DOF/@Pendubot/get_n','Vars',{q_sym,qd_sym});
matlabFunction(C_sym,'File','planar2DOF/@Pendubot/get_C','Vars',{q_sym,qd_sym});
matlabFunction(g_sym,'File','planar2DOF/@Pendubot/get_g','Vars',{q_sym});