function [M,C,G] = screw_MCG(q,q_d,ur10)

gamma0  = [0, 0, 9.81, 0, 0, 0]'; %gravity acceleration vector

% homogenous transformation from frame p to k. Both frames are used to
% indicate a frame attached to a rigid body
T_pk  = zeros(4,4,6); 
% Inverse of adjoint transformation matrix that is used for screw
% tranformation between frames
invAd_pk = zeros(6,6,6); %inverse of adjoint matrix from p to k
%inverse of adjoint matrix from 0 to k
invAd_0k = zeros(6,6,7); invAd_0k(:,:,1) = eye(6,6); 

Jk = zeros(6,6,7); %jacobian up to body k

adj_pk = zeros(6,6,6); % the Lie bracket matrix from p to k
adj_0k = zeros(6,6,6); % the Lie bracket matrix form 0 to k
xi_k = zeros(6,6); % relative velocity between bodies p and k
v_k = zeros(6,7); % velocity of a current link
Jk_d = zeros(6,6,7); % derivative of jacobian
Psi_k = zeros(6,6,6);

M = zeros(6,6); % inertia matrix
C = zeros(6,6); % matrix of coriolis and centrifugal forces
G = zeros(6,1); % vector of gravity forces

for i = 1:1:6
    jnt_axs_k = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
%     Transformation from parent link frame p to current joint frame
    R_pj = RPY(str2num(ur10.robot.joint{i}.origin.Attributes.rpy));
    p_pj = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
    T_pj = [R_pj, p_pj; zeros(1,3), 1];
    invAd_pj = inv_Ad_transf(T_pj);
%     Tranformation from joint frame of the joint that rotaties body k to
%     link frame. The transformation is pure rotation
    R_jk = Rot(q(i),jnt_axs_k);
    p_jk = zeros(3,1);
    T_jk = [R_jk, p_jk; zeros(1,3),1];
    invAd_jk = inv_Ad_transf(T_jk);
%     Transformation from parent link frame p to current link frame k
    T_pk(:,:,i) = T_pj*T_jk;
%     Inverse of the adjoint transformation from frame p attached to parent
%     and frame k to the current link
%     invAd_pk(:,:,i) = inv_Ad_transf(T_pk(:,:,i));
    invAd_pk(:,:,i) = invAd_jk*invAd_pj;
%     Inverse of the adjoint tranformation from inertial frame 0 to cureent
%     body frame k
    invAd_0k(:,:,i+1) = invAd_pk(:,:,i)*invAd_0k(:,:,i);
%     Jacobian of body k
    Jk(:,:,i+1) = invAd_pk(:,:,i)*Jk(:,:,i) + ur10.XI(:,:,i);
%     Twsit of body k
    xi_k(:,i) = ur10.XI(:,:,i)*q_d;
%     Velocity of body k in body frame
    v_k(:,i+1) = invAd_pk(:,:,i)*v_k(:,i) + xi_k(:,i);
    adj_pk(:,:,i) = adj_transf(xi_k(:,i));
    adj_0k(:,:,i) = adj_transf(v_k(:,i+1));
    
    Psi_k(:,:,i) = ur10.Lmbd_k(:,:,i)*adj_0k(:,:,i) - ...
                        adj_0k(:,:,i)'*ur10.Lmbd_k(:,:,i);
    
    Jk_d(:,:,i+1) = invAd_pk(:,:,i)*Jk_d(:,:,i) - ...
                        adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk(:,:,i); 
                 
    M = M + Jk(:,:,i+1)'*ur10.Lmbd_k(:,:,i)*Jk(:,:,i+1);
    
    C = C + Jk(:,:,i+1)'*( Psi_k(:,:,i)*Jk(:,:,i+1) + ...
                        ur10.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) );
    
    gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration
    
    G = G + Jk(:,:,i+1)'*ur10.Lmbd_k(:,:,i)*gamma_k;
end

end