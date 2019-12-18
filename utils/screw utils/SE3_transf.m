function m = SE3_transf(xi_hat,theta)
%SE3_TRANSF Summary of this function goes here
% w_hat = xi(1:3,1:3)
% v = xi(1:3,1)
% w = xi(4:6,1)
tmp1 = rodrig_formula(xi_hat(1:3,1:3), theta);
xi = tw2tw_cord(xi_hat);
m = [tmp1, (eye(3,3) - tmp1)*cross(xi(4:6,1), xi(1:3,1)) + xi(4:6,1)*xi(4:6,1)'*xi(1:3,1)*theta;
        0, 0, 0, 1];
end

