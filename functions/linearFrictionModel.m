function tau = linearFrictionModel(a, qd)
% ------------------------------------------------------------------
% The function computes torque due to friction using linear
% friction model: tau = fv*qd + fc*sign(qd) + fo
% Inputs:
%   a - vector containing friction paramters, a(1) - viscous friction, 
%       a(2) - Coulomb friction, a(3) - offset
%   qd - generilized velocity of the joint(link)
% Outputs:
%   tau - troque due to friction
% -------------------------------------------------------------------
tau = a(1)*qd + a(2)*sign(qd) + a(3);
