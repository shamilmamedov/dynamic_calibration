clc; clear all; close all;


% Symbolic treatment
%{
syms xs ys zs real
syms x y z real
syms a b c real
syms m real

v = [x; y; z];
vs = [xs; ys; zs];
abc = [a; b; c];
Q = diag(abc.^2);

t1 = (v-vs)'*inv(Q)*(v-vs)
t2 = m*(v-vs)'*inv(Q)*(v-vs)
t3 = (m*v-m*vs)'*inv(m*Q)*(m*v-m*vs)


%}


xs = [0.125 0 0]';
abc = [0.125 0.015 0.005]';

[x,y,z] = ellipsoid(xs(1), xs(2), xs(3), ...
                    abc(1), abc(2), abc(3), 40);
                
figure
surf(x, y, z, 'FaceAlpha', 1)
axis equal
xlabel('X, m')
ylabel('Y, m')
zlabel('Z, m')