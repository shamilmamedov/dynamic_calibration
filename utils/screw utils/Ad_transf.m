function Ad_g = Ad_transf(g)
%Computes adjoint matrix of the homogenous transformation 
%matrix g = [R p; 0 1]
Ad_g = [g(1:3,1:3), vec2skewSymMat(g(1:3,4))*g(1:3,1:3);
        zeros(3,3), g(1:3,1:3)];

end

