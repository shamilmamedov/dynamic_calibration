function tau = nonlinearFrictionModel(a, qd)
% ------------------------------------------------------------------
% The function computes torque due to friction using nonlinear
% friction model
% Inputs:
%   a - vector containing friction paramters, a(1) - viscous friction, 
%       a(2) - offset, a(3:5) - nonlinear term paramteres
%   qd - generilized velocity of the joint(link)
% Outputs:
%   tau - troque due to friction
% -------------------------------------------------------------------
tau = a(1)*qd + a(2) + a(3)./( 1 + exp(-a(4).*(qd + a(5)) ) );