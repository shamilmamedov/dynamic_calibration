function pos = ur_fk(q, ur)

T_pk = zeros(4,4,6); % transformation between links
T_0k = zeros(4,4,7); T_0k(:,:,1) = eye(4);
for i = 1:6
    jnt_axs_k = str2num(ur.robot.joint{i}.axis.Attributes.xyz)';
% Transformation from parent link frame p to current joint frame
    rpy_k = str2num(ur.robot.joint{i}.origin.Attributes.rpy);
    R_pj = RPY(rpy_k);
    p_pj = str2num(ur.robot.joint{i}.origin.Attributes.xyz)';
    T_pj = [R_pj, p_pj; zeros(1,3), 1]; % to avoid numerical errors
% Tranformation from joint frame of the joint that rotaties body k to
% link frame. The transformation is pure rotation
    R_jk = Rot(q(i), jnt_axs_k);
    p_jk = zeros(3,1);
    T_jk = [R_jk, p_jk; zeros(1,3),1];
% Transformation from parent link frame p to current link frame k
    T_pk(:,:,i) = T_pj*T_jk;
%  Transformation to base   
    T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
end
pos = T_0k(1:3,4,7);

% (T_0k(1:3,4,7)-T_0k(1:3,4,5))'
