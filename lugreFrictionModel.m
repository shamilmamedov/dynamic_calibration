clc; clear all; close all;


Fc = 0.6;
Fs = 1;
vs = 0.001;
sigma0 = 1e+3;
sigma1 = sqrt(1e+3);
sigma2 = 1.3;
prms = [sigma0, sigma1, sigma2, Fc, Fs, vs]';

t = 0:1e-4:10;
q_fcn = @(t) sin(10*t);
qd_fcn = @(t) cos(10*t)*10;

x(1) = 0;
xd(1) = 0;
for i = 1:length(t)-1
   xd(i+1) =  lugreFriction(x(i), qd_fcn(t(i)), prms);
   x(i+1) = x(i) +  lugreFriction(x(i), qd_fcn(t(i)), prms)*1e-3;
end
F = sigma0*x + sigma1*xd + sigma2*qd_fcn(t);

figure
plot(qd_fcn(t), x)


function dxdt = lugreFriction(x,qd,prms)
    sigma0 = prms(1);
    sigma1 = prms(2);
    sigma2 = prms(3);
    Fc = prms(4);
    Fs = prms(5);
    vs = prms(6);
    
    r = Fc/sigma0 + (Fs - Fc)/sigma0*exp(-(qd/vs)^2);
    dxdt = qd - abs(qd)/r*x;
end