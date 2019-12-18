function Vb = velocity_b(xi_hat,theta_d, g0)
xi = tw2tw_cord(xi_hat);
Vb = tw_cord2tw((inv_adj_transf(g0)*xi))*theta_d;
end

