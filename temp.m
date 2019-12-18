clc; clear all;  

% Ki = 100;
% Kb = 100;
% Jm = 2e-3;
% Bm = 0.02;
% L = 1e-3;
% R = 2;
% 
% W = tf([Ki],[L*Jm (Bm*L + R*Jm) (R*Bm + Ki*Kb) 0])
% 
% w = 1e-2;
% s = 1i*w;
% frsp = evalfr(W,s)


syms s w L R Ki Kb Jm Bm real

G = s*( (L*s+R)*(Jm*s+Bm) + Ki*Kb );
G2 = 1i*w*((L*1i*w+R)*(Jm*1i*w+Bm) + Ki*Kb);

G3 = ( -(Bm*L*w^2 + R*Jm*w^2) + 1i*(R*Bm*w + Ki*Kb*w - L*Jm*w^3));

G4 = G2*( -(Bm*L*w^2 + R*Jm*w^2) - 1i*(R*Bm*w + Ki*Kb*w - L*Jm*w^3) )

G5 = w^4*(Bm*L + R*Jm)^2 + w^2*(R*Bm + Ki*Kb - L*Jm*w^2)^2