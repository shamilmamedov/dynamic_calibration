clc; clear all; close all;

syms l real
syms q2 q1_d q2_d real

p1 = [l;0;0]; % distnace between parallel joints 1 and 2

% transformation from link 1 frame to joint 2 frame
Tj1 = [eye(3), p1; zeros(1,3) 1];
invTj = [Tj1(1:3,1:3)', -Tj1(1:3,1:3)'*Tj1(1:3,4); zeros(1,3),1];
% transformation from joint 2 frame to link 2 frame
Rl1 = [cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1];
Tl1 = [Rl1, zeros(3,1); zeros(1,3),1];
% Transfomation from link 1 frame to link 2 frame
T = Tj1*Tl1;
% Transformation from link 2 frame to link 1 frame
invT = [T(1:3,1:3)', -T(1:3,1:3)'*T(1:3,4); zeros(1,3),1];


% You can proceed with two options in terms of notation. First you can
% assume twist to be [w v], and second you can assue twist to be 
% [v w], and you can slightly different results

% ------------------------------------------------------------------------
% [w v]
% ------------------------------------------------------------------------
Phi1 = [0 0 1 0 0 0]'; % screw axis of joint 1 in frame 1 
Phi2 = [0 0 1 0 0 0]'; % screw axis of joint 2 in frame 2

% adjoint transformation from frame of link 2 to frame of link 1
invAdT = [invT(1:3,1:3), zeros(3); vec2skewSymMat(invT(1:3,4))*...
                                        invT(1:3,1:3), invT(1:3,1:3)];
                                    
invAdTj = [invTj(1:3,1:3), zeros(3); vec2skewSymMat(invTj(1:3,4))*...
                                        invTj(1:3,1:3), invTj(1:3,1:3)];
                                    
v1 = Phi1*q1_d; % velocity of link 1 

v2 = invAdT*v1 + Phi2*q2_d; % velocity of link 2

V2 = [0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0]';

syms Ixx Iyy Izz Ixy Ixz Iyz m hx hy hz  real

I11 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
I12 = vec2skewSymMat([hx,hy,hz]);
I21 = -vec2skewSymMat([hx,hy hz]);
I22 = eye(3)*m;

I = [I11, I12; I21, I22];

momentum_cond = Phi2'*I*V2;

rlxd_inv_cond = simplify(Phi1'*invAdT'*I*invAdT*Phi1);


% ------------------------------------------------------------------------
% [v w]
% -----------------------------------------------------------------------
Phi1 = [0 0 0 0 0 1]'; % screw axis of joint 1 in frame 1 
Phi2 = [0 0 0 0 0 1]'; % screw axis of joint 2 in frame 2

% adjoint transformation from frame of link 2 to frame of link 1
invAdT = inv_Ad_transf(T);

v1 = Phi1*q1_d; % velocity of link 1 

v2 = invAdT*v1 + Phi2*q2_d; % velocity of link 2

V2 = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 0 0 1]';

I11 = m*eye(3);
I12 = -m*vec2skewSymMat([hx,hy,hz]);
I21 = m*vec2skewSymMat([hx,hy,hz]);
I22 = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz] - ...
        m*vec2skewSymMat([hx,hy,hz])*vec2skewSymMat([hx,hy,hz]);

I = [I11, I12; I21, I22];

momentum_cond = Phi2'*I*V2

rlxd_inv_cond = simplify( Phi1'*invAdT'*I*invAdT*Phi1 )
