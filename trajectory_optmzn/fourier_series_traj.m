function [q,qd,q2d] = fourier_series_traj(t,q0,A,B,w,N)
% -----------------------------------------------------------------------
% The function evaluates trancated Fourier series polynomial
% Inputs:
%   t - time instantes at which the polynomail should be evaluated
%   q0 - initial offset, should be vector (nx1)
%   A - sine coefficients
%   B - cosine coefficients
%   w - fundamental frequency
%   N - number of harmonics
% Outputs:
%   q - positions i.e. value of polynomial
%   qd - velocities i.e. derivative of polynomail
%   q2d - accelerations i.e. second derivative of polynomial
% -----------------------------------------------------------------------
q = q0;
qd = zeros(size(q0));
q2d = zeros(size(q0));
for k = 1:N
   q = q + A(:,k)/(w*k).*sin(w*k*t) - B(:,k)/(w*k).*cos(w*k*t);
   qd = qd + A(:,k).*cos(w*k*t) + B(:,k)*sin(w*k*t);
   q2d = q2d - A(:,k)*w*k.*sin(w*k*t) + B(:,k)*w*k*cos(w*k*t);
end
