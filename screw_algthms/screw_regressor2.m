function Y = screw_regressor2(q,q_d,q_2d,ur10)

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

Y = [];
for i = 1:1:6
    jnt_axs_k = ur10.rot_axes(:,i);
    T_pj = ur10.T_pj(:,:,i);
    R_jk = Rot(q(i),jnt_axs_k);
    p_jk = zeros(3,1);
    T_jk = [R_jk, p_jk; zeros(1,3),1];
    T_pk(:,:,i) = T_pj*T_jk;
    
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
% ------------------------------------------------------------------------
%       Estimateing Regressor
% ------------------------------------------------------------------------
    r_k = ur10.r_com(:,i);
    alpha_k = Jk(:,:,i+1)*q_2d + adj_0k(:,:,i)*Jk(:,:,i+1)*q_d + Jk_d(:,:,i+1)*q_d + gamma_k;
    
    t1 = vec2skewSymMat(r_k);
    t2 = vec2skewSymMat(alpha_k(4:6));
    A1 = [alpha_k(1:3), t2, zeros(3,6);
          zeros(3,1), -vec2skewSymMat(alpha_k(1:3)) + t1*t2, ...
          vec2mat_ssmat(alpha_k(4:6))];
    
    t4 = Jk(:,:,i+1)*q_d;
    t5 = vec2skewSymMat(t4(4:6));
    
    A2 = [t4(1:3), t5, zeros(3,6);
          zeros(3,1), -vec2skewSymMat(t4(1:3)) + t1*t5,...
          vec2mat_ssmat(t4(4:6))];
    
    t6 = A1 - adj_0k(:,:,i)'*A2;  
    
    Y = [Y, Jk(:,:,i+1)'*t6];
% ------------------------------------------------------------------------
end
