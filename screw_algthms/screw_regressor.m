function Y = screw_regressor(q,q_d,q_2d,ur10)

gamma0  = [0, 0, 9.81, 0, 0, 0]'; %gravity acceleration vector

T_pk  = zeros(4,4,6); %homogenous transformation from p to k
invAd_pk = zeros(6,6,6); %inverse of adjoint matrix from p to k
invAd_0k = zeros(6,6,7); invAd_0k(:,:,1) = eye(6,6);
Jk = zeros(6,6,7); %jacobian up to body k

adj_pk = zeros(6,6,6);
adj_0k = zeros(6,6,6);
xi_k = zeros(6,6);
v_k = zeros(6,7);
Jk_d = zeros(6,6,7);
FI_k = zeros(6,6,6);

R1 = [];
R2 = [];
Lmda = [];

for i = 1:1:6
    aor_k = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
    R_pk = RPY(str2num(ur10.robot.joint{i}.origin.Attributes.rpy))*Rot(q(i),aor_k);
    p_pk = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
    T_pk(:,:,i)  = [R_pk, p_pk; zeros(1,3), 1];

    invAd_pk(:,:,i) = inv_Ad_transf(T_pk(:,:,i));
    invAd_0k(:,:,i+1) = invAd_pk(:,:,i)*invAd_0k(:,:,i);
    Jk(:,:,i+1) = invAd_pk(:,:,i)*Jk(:,:,i) + ur10.XI(:,:,i);
    
    xi_k(:,i) = ur10.XI(:,:,i)*q_d;
    v_k(:,i+1) = invAd_pk(:,:,i)*v_k(:,i) + xi_k(:,i);
    adj_pk(:,:,i) = adj_transf(xi_k(:,i));
    adj_0k(:,:,i) = adj_transf(v_k(:,i+1));
    FI_k(:,:,i) = ur10.Lmbd_k(:,:,i)*adj_0k(:,:,i) - adj_0k(:,:,i)'*ur10.Lmbd_k(:,:,i);
    Jk_d(:,:,i+1) = invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk(:,:,i); 
                 
   
    gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration
%     -------------------------------------------------------------------
%       Estimateing Regressor
%     -------------------------------------------------------------------
    r_k = str2num(ur10.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    alpha_k = Jk(:,:,i+1)*q_2d + adj_0k(:,:,i)*Jk(:,:,i+1)*q_d + Jk_d(:,:,i+1)*q_d + gamma_k;
    
    A1 = [alpha_k(1:3), vec2skewSymMat(alpha_k(4:6)), zeros(3,6);
        zeros(3,1), -vec2skewSymMat(alpha_k(1:3)) + vec2skewSymMat(r_k)*vec2skewSymMat(alpha_k(4:6)),...
        vec2mat_ssmat(alpha_k(4:6))];
    
    t4 = Jk(:,:,i+1)*q_d;
    
    A2 = [t4(1:3), vec2skewSymMat(t4(4:6)), zeros(3,6);
        zeros(3,1), -vec2skewSymMat(t4(1:3)) + vec2skewSymMat(r_k)*vec2skewSymMat(t4(4:6)),...
        vec2mat_ssmat(t4(4:6))];
    
    t4 = A1 - adj_0k(:,:,i)'*A2;  
    
    R1 = vertcat(R1, Jk(:,:,i+1));
    R2 = blkdiag(R2, t4);
%     -------------------------------------------------------------------
end
Y = R1'*R2;