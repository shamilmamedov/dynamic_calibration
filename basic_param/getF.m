function f = getF(v,w,z,qdi)
% Find f multiplier
%   v - linear velocity in frame i
%   w - angular velocity in frame i
%   z - axis of rotaion (in frame i)
%   dqi - angular velocity of i-th joint
% Return:
%   vector of coefficients

    f = [w(1)*z(1)-z(1)*qdi/2; w(1)*z(2)+w(2)*z(1); w(1)*z(3)+w(3)*z(1); 
         w(2)*z(2)-z(2)*qdi/2; w(2)*z(3)+w(3)*z(2); w(3)*z(3)-z(3)*qdi/2;
         v(2)*z(3)-v(3)*z(2); v(3)*z(1)-v(1)*z(3); v(1)*z(2)-v(2)*z(1); 0];
end