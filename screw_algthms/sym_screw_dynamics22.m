function Y_g = sym_screw_dynamics22(q,q_d,ur10)

gamma0  = [0, 0, 9.81, 0, 0, 0]'; %gravity acceleration vector

T_pk  = sym(zeros(4,4,6)); %homogenous transformation from p to k
invAd_pk = sym(zeros(6,6,6)); %inverse of adjoint matrix from p to k
invAd_0k = sym(zeros(6,6,7)); invAd_0k(:,:,1) = eye(6,6);
Jk = sym(zeros(6,6,7)); %jacobian up to body k

% adj_pk = zeros(6,6,6);
% adj_0k = zeros(6,6,6);
% xi_k = zeros(6,6);
% v_k = zeros(6,7);
% Jk_d = zeros(6,6,7);
% FI_k = zeros(6,6,6);

% M = zeros(6,6);
% C = zeros(6,6);
G = zeros(6,1);
g2 = zeros(6,1);

Y_pi = zeros(6,1);

for i = 1:1:6
    aor_k = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
    R_pk = RPY(str2num(ur10.robot.joint{i}.origin.Attributes.rpy))*Rot(q(1),aor_k);
    p_pk = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
    T_pk(:,:,i)  = [R_pk, p_pk; zeros(1,3), 1];

    invAd_pk(:,:,i) = inv_Ad_transf(T_pk(:,:,i));
    invAd_0k(:,:,i+1) = invAd_pk(:,:,i)*invAd_0k(:,:,i);
    Jk(:,:,i+1) = invAd_pk(:,:,i)*Jk(:,:,i) + ur10.XI(:,:,i);
    
%     xi_k(:,i) = ur10.XI(:,:,i)*q_d;
%     v_k(:,i+1) = invAd_pk(:,:,i)*v_k(:,i) + xi_k(:,i);
%     adj_pk(:,:,i) = adj_transf(xi_k(:,i));
%     adj_0k(:,:,i) = adj_transf(v_k(:,i+1));
%     FI_k(:,:,i) = ur10.Lmbd_k(:,:,i)*adj_0k(:,:,i) - adj_0k(:,:,i)'*ur10.Lmbd_k(:,:,i);
%     Jk_d(:,:,i+1) = invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk(:,:,i); 
%                  
%     M = M + Jk(:,:,i+1)'*ur10.Lmbd_k(:,:,i)*Jk(:,:,i+1);
%     C = C + Jk(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + ur10.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) );
    
    gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration
    G = G + Jk(:,:,i+1)'*ur10.Lmbd_k(:,:,i)*gamma_k;
    
    t1 = gamma_k;
    t2 = str2num(ur10.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    A = [t1(1:3), vec2skewSymMat(t1(4:6)), zeros(3,6);
        zeros(3,1), -vec2skewSymMat(t1(1:3)) + vec2skewSymMat(t2)*vec2skewSymMat(t1(4:6)),...
        vec2mat_ssmat(t1(4:6))];
    
    dpi_k(:,:,i) = Jk(:,:,i+1)'*A;
    g2 = g2 + dpi_k(:,:,i)*ur10.pi_reg(:,i);
 
end

Y_g = reshape(dpi_k,[6,60]);

end