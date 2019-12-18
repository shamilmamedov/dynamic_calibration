function [M,C,G,Y] = screw_dynamics22(q,q_d,q_2d,ur10)

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

M = zeros(6,6);
C = zeros(6,6);
G = zeros(6,1);
g2 = zeros(6,1);
M1 = zeros(6,1);

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
                 
    M = M + Jk(:,:,i+1)'*ur10.Lmbd_k(:,:,i)*Jk(:,:,i+1);
    
    C = C + Jk(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + ur10.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) );
    
    gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration
    G = G + Jk(:,:,i+1)'*ur10.Lmbd_k(:,:,i)*gamma_k;
%     --------------------------------------------------------------------
%     t1 = gamma_k;
%     t2 = str2num(ur10.robot.link{i+1}.inertial.origin.Attributes.xyz)';
%     A = [t1(1:3), vec2skewSymMat(t1(4:6)), zeros(3,6);
%         zeros(3,1), -vec2skewSymMat(t1(1:3)) + vec2skewSymMat(t2)*vec2skewSymMat(t1(4:6)),...
%         vec2mat_ssmat(t1(4:6))];
%     
%     dpi_k(:,:,i) = Jk(:,:,i+1)'*A;
%     g2 = g2 + dpi_k(:,:,i)*ur10.pi_reg(:,i);
%     
%     t3 = Jk(:,1,i+1);
%     A = [t3(1:3), vec2skewSymMat(t3(4:6)), zeros(3,6);
%         zeros(3,1), -vec2skewSymMat(t3(1:3)) + vec2skewSymMat(t2)*vec2skewSymMat(t3(4:6)),...
%         vec2mat_ssmat(t3(4:6))];
%     M1 = M1 + Jk(:,:,i+1)'*A*ur10.pi_reg(:,i);
    

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

% g3 = reshape(dpi_k,[6,60])*reshape(ur10.pi_reg,[60,1]);
return







%PRELOCATING VARIABLES
if ordr >= 1
    Tpk         = zeros(4,4,6); %homogenous transformation from p to k
    invAd_pk    = zeros(6,6,6); %inverse of adjoint matrix from p to k
    invAd_0k    = zeros(6,6,7); invAd_0k(:,:,1) = eye(6,6);
    Jk          = zeros(6,6,7); %jacobian up to body k

    if sum(strcmp(lst,'M'))~=0 || sum(strcmp(lst,'sys_jcbn'))~=0;  scrw.M = zeros(6,6); end
    if sum(strcmp(lst,'g'))~=0 || sum(strcmp(lst,'sys_jcbn'))~=0 || sum(strcmp(lst,'momentum'))~=0; scrw.g = zeros(6,1); end
    
%-------------------------------------------------------------------------------    
    if ordr >= 2
        xi_k        = zeros(6,6); 
        v_k         = zeros(6,7);   %body velocity of k-th body
        Jk_d        = zeros(6,6,7);
        adj_pk      = zeros(6,6,6);
        adj_0k      = zeros(6,6,6);
        FI_k        = zeros(6,6,6);
        
        if sum(strcmp(lst,'C'))~=0 || sum(strcmp(lst,'sys_jcbn'))~=0 || sum(strcmp(lst,'momentum'))~=0; scrw.C = zeros(6,6); end
        if sum(strcmp(lst,'M_d'))~=0 ; scrw.M_d = zeros(6,6); end
        if sum(strcmp(lst,'g_d'))~=0; scrw.g_d = zeros(6,1); end
        if sum(strcmp(lst,'momentum'))~=0; scrw.momentum = zeros(6,1); end
        if sum(strcmp(lst,'sys_jcbn'))~=0
            scrw.sys_jcbn = zeros(6,12);
            
            Jk_q1       = zeros(6,6,7);
            Jk_q2       = zeros(6,6,7);
            Jk_q3       = zeros(6,6,7);
            Jk_q4       = zeros(6,6,7);
            Jk_q5       = zeros(6,6,7);
            Jk_q6       = zeros(6,6,7);

            Jk_d_q1     = zeros(6,6,7);
            Jk_d_q2     = zeros(6,6,7);
            Jk_d_q3     = zeros(6,6,7);
            Jk_d_q4     = zeros(6,6,7);
            Jk_d_q5     = zeros(6,6,7);
            Jk_d_q6     = zeros(6,6,7);
            
            M_q1        = zeros(6,6);
            M_q2        = zeros(6,6);
            M_q3        = zeros(6,6);
            M_q4        = zeros(6,6);
            M_q5        = zeros(6,6);
            M_q6        = zeros(6,6);

            g_q1        = zeros(6,1);
            g_q2        = zeros(6,1);
            g_q3        = zeros(6,1);
            g_q4        = zeros(6,1);
            g_q5        = zeros(6,1);
            g_q6        = zeros(6,1);

            C_q1        = zeros(6,6);
            C_q2        = zeros(6,6);
            C_q3        = zeros(6,6);
            C_q4        = zeros(6,6);
            C_q5        = zeros(6,6);
            C_q6        = zeros(6,6);
            
              
            C_q1_d      = zeros(6,6);
            C_q2_d      = zeros(6,6);
            C_q3_d      = zeros(6,6);
            C_q4_d      = zeros(6,6);
            C_q5_d      = zeros(6,6);
            C_q6_d      = zeros(6,6); 
        end
        
        
%-------------------------------------------------------------------------------        
        if ordr >= 3
           xi_k_d      = zeros(6,6); 
           adj_pk_d    = zeros(6,6,6);
           adj_0k_d    = zeros(6,6,6);
           v_k_d       = zeros(6,7);
           FI_k_d      = zeros(6,6,6);
           Jk_2d       = zeros(6,6,7); %second derivative of a jacobian
            
           if sum(strcmp(lst,'C_d'))~=0 ; scrw.C_d = zeros(6,6); end
           if sum(strcmp(lst,'M_2d'))~=0 ; scrw.M_2d = zeros(6,6); end
           if sum(strcmp(lst,'g_2d'))~=0 ; scrw.g_2d = zeros(6,6); end
           
%--------------------------------------------------------------------------------           
           if ordr >=4
               xi_k_2d     = zeros(6,6); 
               adj_pk_2d   = zeros(6,6,6);
               adj_0k_2d   = zeros(6,6,6);
               v_k_2d      = zeros(6,7);
               Jk_3d       = zeros(6,6,7); %second derivative of a jacobia
               
               if sum(strcmp(lst,'C_2d'))~=0 ; scrw.C_2d = zeros(6,6); end
           end
        end
        
    end    
end

for i = 1:1:6
    if ordr >= 1
        R           = RPY(par.joint_origin(i, 4:6))*Rotz_s(Q(1,i));
        p           = par.joint_origin(i, 1:3)';
        Tpk(:,:,i)  = [R, p; zeros(1,3), 1];

        invAd_pk(:,:,i)     = inv_Ad_transf(Tpk(:,:,i));
        invAd_0k(:,:,i+1)   = invAd_pk(:,:,i)*invAd_0k(:,:,i);

        Jk(:,:,i+1)         = invAd_pk(:,:,i)*Jk(:,:,i) + par.XI(:,:,i);
        
        if sum(strcmp(lst,'M'))~=0 || sum(strcmp(lst,'sys_jcbn'))~=0
            scrw.M = scrw.M + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1); 
        end
        
        if sum(strcmp(lst,'g'))~=0 || sum(strcmp(lst,'sys_jcbn'))~=0 || sum(strcmp(lst,'momentum'))~=0   
            gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration  
            scrw.g = scrw.g + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*gamma_k;
        end
        
%----------------------------------------------------------------------------------        
        if ordr >= 2
            xi_k(:,i)       = par.XI(:,:,i)*Q(2,:)';
            v_k(:,i+1)      = invAd_pk(:,:,i)*v_k(:,i) + xi_k(:,i);
            adj_pk(:,:,i)   = adj_transf(xi_k(:,i));
            adj_0k(:,:,i)   = adj_transf(v_k(:,i+1));
            FI_k(:,:,i)     = par.Lmbd_k(:,:,i)*adj_0k(:,:,i) - adj_0k(:,:,i)'*par.Lmbd_k(:,:,i);
            Jk_d(:,:,i+1)   = invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk(:,:,i); 
            
            if sum(strcmp(lst,'C'))~=0 || sum(strcmp(lst,'sys_jcbn'))~=0 || sum(strcmp(lst,'momentum'))~=0
                scrw.C       = scrw.C + Jk(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) );               
            end
                
            if sum(strcmp(lst,'M_d'))~=0 
                 scrw.M_d     = scrw.M_d + Jk_d(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + ...
                                Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1);
            end
            
            if sum(strcmp(lst,'g_d'))~=0 
                if sum(strcmp(lst,'g')) == 0
                    gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration 
                end
                scrw.g_d     = scrw.g_d + (Jk_d(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_0k(:,:,i))*gamma_k;
            end
            
            if sum(strcmp(lst,'sys_jcbn'))~=0 
                    Jk_q1(:,:,i+1)  = invAd_pk(:,:,i)*Jk_q1(:,:,i) - adj_transf(par.XI(:,1,i))*invAd_pk(:,:,i)*Jk(:,:,i);
                    Jk_q2(:,:,i+1)  = invAd_pk(:,:,i)*Jk_q2(:,:,i) - adj_transf(par.XI(:,2,i))*invAd_pk(:,:,i)*Jk(:,:,i);
                    Jk_q3(:,:,i+1)  = invAd_pk(:,:,i)*Jk_q3(:,:,i) - adj_transf(par.XI(:,3,i))*invAd_pk(:,:,i)*Jk(:,:,i);
                    Jk_q4(:,:,i+1)  = invAd_pk(:,:,i)*Jk_q4(:,:,i) - adj_transf(par.XI(:,4,i))*invAd_pk(:,:,i)*Jk(:,:,i);
                    Jk_q5(:,:,i+1)  = invAd_pk(:,:,i)*Jk_q5(:,:,i) - adj_transf(par.XI(:,5,i))*invAd_pk(:,:,i)*Jk(:,:,i);
                    Jk_q6(:,:,i+1)  = invAd_pk(:,:,i)*Jk_q6(:,:,i) - adj_transf(par.XI(:,6,i))*invAd_pk(:,:,i)*Jk(:,:,i);

                    Jk_d_q1(:,:,i+1) = invAd_pk(:,:,i)*Jk_d_q1(:,:,i) - adj_transf(par.XI(:,1,i))*invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*Jk_q1(:,:,i+1);
                    Jk_d_q2(:,:,i+1) = invAd_pk(:,:,i)*Jk_d_q2(:,:,i) - adj_transf(par.XI(:,2,i))*invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*Jk_q2(:,:,i+1);
                    Jk_d_q3(:,:,i+1) = invAd_pk(:,:,i)*Jk_d_q3(:,:,i) - adj_transf(par.XI(:,3,i))*invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*Jk_q3(:,:,i+1);
                    Jk_d_q4(:,:,i+1) = invAd_pk(:,:,i)*Jk_d_q4(:,:,i) - adj_transf(par.XI(:,4,i))*invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*Jk_q4(:,:,i+1);
                    Jk_d_q5(:,:,i+1) = invAd_pk(:,:,i)*Jk_d_q5(:,:,i) - adj_transf(par.XI(:,5,i))*invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*Jk_q5(:,:,i+1);
                    Jk_d_q6(:,:,i+1) = invAd_pk(:,:,i)*Jk_d_q6(:,:,i) - adj_transf(par.XI(:,6,i))*invAd_pk(:,:,i)*Jk_d(:,:,i) - adj_pk(:,:,i)*Jk_q6(:,:,i+1);
                
                    M_q1    = M_q1 + (Jk_q1(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q1(:,:,i+1));
                    M_q2    = M_q2 + (Jk_q2(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q2(:,:,i+1));
                    M_q3    = M_q3 + (Jk_q3(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q3(:,:,i+1));
                    M_q4    = M_q4 + (Jk_q4(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q4(:,:,i+1));
                    M_q5    = M_q5 + (Jk_q5(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q5(:,:,i+1));
                    M_q6    = M_q6 + (Jk_q6(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q6(:,:,i+1));

                    g_q1    = g_q1 + (Jk_q1(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_transf(Jk(:,1,i+1)))*gamma_k;
                    g_q2    = g_q2 + (Jk_q2(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_transf(Jk(:,2,i+1)))*gamma_k;
                    g_q3    = g_q3 + (Jk_q3(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_transf(Jk(:,3,i+1)))*gamma_k;
                    g_q4    = g_q4 + (Jk_q4(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_transf(Jk(:,4,i+1)))*gamma_k;
                    g_q5    = g_q5 + (Jk_q5(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_transf(Jk(:,5,i+1)))*gamma_k;
                    g_q6    = g_q6 + (Jk_q6(:,:,i+1)'*par.Lmbd_k(:,:,i) - Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_transf(Jk(:,6,i+1)))*gamma_k;

                    C_q1    = C_q1 + (Jk_q1(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_d_q1(:,:,i+1) + FI_k(:,:,i)*Jk_q1(:,:,i+1)) +...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_transf(Jk_q1(:,:,i+1)*Q(2,:)') - ...
                                     adj_transf(Jk_q1(:,:,i+1)*Q(2,:)')'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1));
                    C_q2    = C_q2 + (Jk_q2(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_d_q2(:,:,i+1) + FI_k(:,:,i)*Jk_q2(:,:,i+1)) +...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_transf(Jk_q2(:,:,i+1)*Q(2,:)') - ...
                                     adj_transf(Jk_q2(:,:,i+1)*Q(2,:)')'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1));
                    C_q3    = C_q3 + (Jk_q3(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_d_q3(:,:,i+1) + FI_k(:,:,i)*Jk_q3(:,:,i+1)) +...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_transf(Jk_q3(:,:,i+1)*Q(2,:)') - ...
                                     adj_transf(Jk_q3(:,:,i+1)*Q(2,:)')'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1));
                    C_q4    = C_q4 + (Jk_q4(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_d_q4(:,:,i+1) + FI_k(:,:,i)*Jk_q4(:,:,i+1)) +...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_transf(Jk_q4(:,:,i+1)*Q(2,:)') - ...
                                     adj_transf(Jk_q4(:,:,i+1)*Q(2,:)')'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1));
                    C_q5    = C_q5 + (Jk_q5(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_d_q5(:,:,i+1) + FI_k(:,:,i)*Jk_q5(:,:,i+1)) +...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_transf(Jk_q5(:,:,i+1)*Q(2,:)') - ...
                                     adj_transf(Jk_q5(:,:,i+1)*Q(2,:)')'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1));
                    C_q6    = C_q6 + (Jk_q6(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_d_q6(:,:,i+1) + FI_k(:,:,i)*Jk_q6(:,:,i+1)) +...
                                     Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_transf(Jk_q6(:,:,i+1)*Q(2,:)') - ...
                                     adj_transf(Jk_q6(:,:,i+1)*Q(2,:)')'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1));
                                 
                 
                   C_q1_d  = C_q1_d + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q1(:,:,i+1) + ...
                                      Jk(:,:,i+1)'*( par.Lmbd_k(:,:,i)*adj_transf(Jk(:,1,i+1)) - adj_transf(Jk(:,1,i+1))'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1);
                   C_q2_d  = C_q2_d + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q2(:,:,i+1) + ...
                                      Jk(:,:,i+1)'*( par.Lmbd_k(:,:,i)*adj_transf(Jk(:,2,i+1)) - adj_transf(Jk(:,2,i+1))'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1);
                   C_q3_d  = C_q3_d + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q3(:,:,i+1) + ...
                                      Jk(:,:,i+1)'*( par.Lmbd_k(:,:,i)*adj_transf(Jk(:,3,i+1)) - adj_transf(Jk(:,3,i+1))'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1);
                   C_q4_d  = C_q4_d + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q4(:,:,i+1) + ...
                                      Jk(:,:,i+1)'*( par.Lmbd_k(:,:,i)*adj_transf(Jk(:,4,i+1)) - adj_transf(Jk(:,4,i+1))'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1);
                   C_q5_d  = C_q5_d + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q5(:,:,i+1) + ...
                                      Jk(:,:,i+1)'*( par.Lmbd_k(:,:,i)*adj_transf(Jk(:,5,i+1)) - adj_transf(Jk(:,5,i+1))'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1);
                   C_q6_d  = C_q6_d + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_q6(:,:,i+1) + ...
                                      Jk(:,:,i+1)'*( par.Lmbd_k(:,:,i)*adj_transf(Jk(:,6,i+1)) - adj_transf(Jk(:,6,i+1))'*par.Lmbd_k(:,:,i) )*Jk(:,:,i+1); 
                 
            end
            
%---------------------------------------------------------------------------------            
            if ordr >= 3
                xi_k_d(:,i)     = par.XI(:,:,i)*Q(3,:)';
                adj_pk_d(:,:,i) = adj_transf(xi_k_d(:,i));
                v_k_d(:,i+1)    = invAd_pk(:,:,i)*v_k_d(:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*v_k(:,i) + xi_k_d(:,i);
                adj_0k_d(:,:,i) = adj_transf(v_k_d(:,i+1)); 
                FI_k_d(:,:,i)   = par.Lmbd_k(:,:,i)*adj_0k_d(:,:,i) - adj_0k_d(:,:,i)'*par.Lmbd_k(:,:,i);
                Jk_2d(:,:,i+1)  = invAd_pk(:,:,i)*Jk_2d(:,:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk_d(:,:,i) - ...
                        adj_pk_d(:,:,i)*invAd_pk(:,:,i)*Jk(:,:,i) - adj_pk(:,:,i)*Jk_d(:,:,i+1);
                
                if sum(strcmp(lst,'M_2d'))~=0
                    scrw.M_2d    = scrw.M_2d + Jk_2d(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk(:,:,i+1) + ...
                                    2*Jk_d(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) + ...
                                    Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_2d(:,:,i+1); 
                end
                
                if sum(strcmp(lst,'C_d'))~=0
                    scrw.C_d     = scrw.C_d + Jk_d(:,:,i+1)'*( FI_k(:,:,i)*Jk(:,:,i+1) + par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) ) + ...
                                        Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*Jk_2d(:,:,i+1) + FI_k(:,:,i)*Jk_d(:,:,i+1)) +...
                                        Jk(:,:,i+1)'*FI_k_d(:,:,i)*Jk(:,:,i+1);
                end
                
                if sum(strcmp(lst,'g_2d'))~=0
                    if (sum(strcmp(lst,'g')) == 0) && (sum(strcmp(lst,'g_d')) == 0) 
                        gamma_k = invAd_0k(:,:,i+1)*gamma0; %body gravitational acceleration 
                    end
                    scrw.g_2d    = scrw.g_2d + (Jk_2d(:,:,i+1)'*par.Lmbd_k(:,:,i) - 2*Jk_d(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_0k(:,:,i) - ...
                                        Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_0k_d(:,:,i) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*adj_0k(:,:,i)*adj_0k(:,:,i))*gamma_k;
                end
                
%-----------------------------------------------------------------------------------                
                if ordr >=4
                    xi_k_2d(:,i)    = par.XI(:,:,i)*Q(4,:)';
                    v_k_2d(:,i+1)   = invAd_pk(:,:,i)*v_k_2d(:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*v_k_d(:,i) - ...
                                            adj_pk(:,:,i)*(invAd_pk(:,:,i)*v_k_d(:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*v_k(:,i)) - ...
                                            adj_pk_d(:,:,i)*invAd_pk(:,:,i)*v_k(:,i) + xi_k_2d(:,i);
                    adj_pk_2d(:,:,i) = adj_transf(xi_k_2d(:,i));
                    adj_0k_2d(:,:,i) = adj_transf(v_k_2d(:,i+1));
                    Jk_3d(:,:,i+1)  = invAd_pk(:,:,i)*Jk_3d(:,:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk_2d(:,:,i) - ...
                                        adj_pk_d(:,:,i)*invAd_pk(:,:,i)*Jk_d(:,:,i) - ...
                                        adj_pk(:,:,i)*(invAd_pk(:,:,i)*Jk_2d(:,:,i) - adj_pk(:,:,i)*invAd_pk(:,:,i)*Jk_d(:,:,i)) - ...
                                        adj_pk_2d(:,:,i)*invAd_pk(:,:,i)*Jk(:,:,i) - 2*adj_pk_d(:,:,i)*Jk_d(:,:,i+1) - ...
                                        adj_pk(:,:,i)*Jk_2d(:,:,i+1);                    
                    
                    if sum(strcmp(lst,'C_2d'))~=0
                        scrw.C_2d    = scrw.C_2d + Jk_2d(:,:,i+1)'*FI_k(:,:,i)*Jk(:,:,i+1) + 2*Jk_d(:,:,i+1)'*FI_k(:,:,i)*Jk_d(:,:,i+1) + ...
                                            2*Jk_d(:,:,i+1)'*FI_k_d(:,:,i)*Jk(:,:,i+1) + ...
                                            Jk_2d(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_d(:,:,i+1) + Jk(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_3d(:,:,i+1) + ...
                                            2*Jk_d(:,:,i+1)'*par.Lmbd_k(:,:,i)*Jk_2d(:,:,i+1) + Jk(:,:,i+1)'*FI_k(:,:,i)*Jk_2d(:,:,i+1) + ...
                                            2*Jk(:,:,i+1)'*FI_k_d(:,:,i)*Jk_d(:,:,i+1) + ...
                                            Jk(:,:,i+1)'*(par.Lmbd_k(:,:,i)*adj_0k_2d(:,:,i) - adj_0k_2d(:,:,i)'*par.Lmbd_k(:,:,i))*Jk(:,:,i+1);
                    end
                end
            end
        end        
    end
end

if sum(strcmp(lst,'J'))~=0; scrw.J = Jk(:,:,6+1); end
if sum(strcmp(lst,'J_d'))~=0; scrw.J_d = Jk_d(:,:,6+1); end
if sum(strcmp(lst,'J_2d'))~=0; scrw.J_2d = Jk_2d(:,:,6+1); end

if sum(strcmp(lst,'sys_jcbn'))~=0 
    I6 = eye(6);
    invM = scrw.M\I6;
    temp_tau = scrw.C*Q(2,:)' + scrw.g;
    scrw.sys_jcbn(:,1) = invM*M_q1*invM*temp_tau - invM*(C_q1*Q(2,:)' + g_q1);
    scrw.sys_jcbn(:,2) = invM*M_q2*invM*temp_tau - invM*(C_q2*Q(2,:)' + g_q2);
    scrw.sys_jcbn(:,3) = invM*M_q3*invM*temp_tau - invM*(C_q3*Q(2,:)' + g_q3);
    scrw.sys_jcbn(:,4) = invM*M_q4*invM*temp_tau - invM*(C_q4*Q(2,:)' + g_q4);
    scrw.sys_jcbn(:,5) = invM*M_q5*invM*temp_tau - invM*(C_q5*Q(2,:)' + g_q5);
    scrw.sys_jcbn(:,6) = invM*M_q6*invM*temp_tau - invM*(C_q6*Q(2,:)' + g_q6);    
    scrw.sys_jcbn(:,7) = -invM*(C_q1_d*Q(2,:)' + scrw.C*I6(:,1));
    scrw.sys_jcbn(:,8) = -invM*(C_q2_d*Q(2,:)' + scrw.C*I6(:,2));
    scrw.sys_jcbn(:,9) = -invM*(C_q3_d*Q(2,:)' + scrw.C*I6(:,3));
    scrw.sys_jcbn(:,10) = -invM*(C_q4_d*Q(2,:)' + scrw.C*I6(:,4));
    scrw.sys_jcbn(:,11) = -invM*(C_q5_d*Q(2,:)' + scrw.C*I6(:,5));
    scrw.sys_jcbn(:,12) = -invM*(C_q6_d*Q(2,:)' + scrw.C*I6(:,6));
    
    if sum(strcmp(lst,'M'))==0; scrw = rmfield(scrw,'M'); end
    if sum(strcmp(lst,'C'))==0; scrw = rmfield(scrw,'C'); end
    if sum(strcmp(lst,'g'))==0; scrw = rmfield(scrw,'g'); end
end

if sum(strcmp(lst,'momentum'))~=0
    scrw.momentum = -scrw.g + scrw.C'*Q(2,:)';
    
    if sum(strcmp(lst,'C'))==0; scrw = rmfield(scrw,'C'); end
    if sum(strcmp(lst,'g'))==0; scrw = rmfield(scrw,'g'); end
end

end