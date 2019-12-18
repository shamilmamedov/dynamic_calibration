clc; clear all; close all;

q_sym = sym('q',[6 1],'real');
syms 'R3' 'D3' 'D4' 'R4' real
%       al, r,  d,   th
dh = [0,    0,  0,  q_sym(1); 
     -pi/2, 0,  0,  q_sym(2);
      0,    R3, D3, q_sym(3);
     -pi/2, D4, R4, q_sym(4);
      pi/2, 0,  0,  q_sym(5);
     -pi/2, 0,  0,  q_sym(6)];
     

% get transformations
z0 = sym([0;0;1]);
for i = 1:6
    par = dh(i,:);
    H = homog_trans_mdh(par(2),par(1),par(3),par(4));
    R(:,:,i) = H(1:3,1:3);
    P(:,:,i) = H(1:3,4);
    Z(:,:,i) = z0;
end

% find parameters
[base,regr] = khalil_gautier(R,P,Z,q_sym);  
     
     
function out = homog_trans_mdh(a,alpha,d,theta)
    t1 = sin(theta);
    t2 = cos(theta);
    t3 = sin(alpha);
    t4 = cos(alpha);
    
    out = [t2,      -t1,    0,      a;
           t1*t4,   t2*t4,  -t3,    -d*t3;
           t1*t3,   t2*t3,  t4,     d*t4; 
           0,       0,      0,      1];    
end