function [q,qd,q2d] = mixed_traj(t,C,A,B,w,N)
% ---------------------------------------------------------------------
% This function computes "mixed trajectory" meaning the trajectory
% that consistes of finite fourier series and fifth order polynomial
% Inputs:
%   t - time instants at which trajectory should be evaluated
%   C - coefficients of the fifth order polynomail
%   A - coeffincients of the sine in finite fourier series
%   B - coefficinets of the cosine in finitne fourier series
%   w - fundamental frequency
%   N - number of harmonics
% ---------------------------------------------------------------------
% finite fourier series
[qh,qhd,qh2d] = fourier_series_traj(t,zeros(6,1),A,B,w,N);

% fifth order polynomail trajectory
qp = C(:,1) + C(:,2).*t + C(:,3).*t.^2 + C(:,4).*t.^3 + ...
                                    C(:,5).*t.^4 + C(:,6).*t.^5;
qpd = C(:,2) + 2*C(:,3).*t + 3*C(:,4).*t.^2 + ...
                                4*C(:,5).*t.^3 + 5*C(:,6).*t.^4;
qp2d = 2*C(:,3) + 6*C(:,4).*t + 12*C(:,5).*t.^2 + 20*C(:,6).*t.^3;

q = qh + qp;
qd = qhd + qpd;
q2d = qh2d + qp2d;
