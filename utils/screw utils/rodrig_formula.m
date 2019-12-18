function y = rodrig_formula(w_hat,theta)
%RODRI computes exo(w_hat*theta)
y = eye(3) + w_hat*sin(theta) + w_hat^2*(1-cos(theta));
end

