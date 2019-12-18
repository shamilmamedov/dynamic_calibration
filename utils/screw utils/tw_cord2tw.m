function m = tw_cord2tw(xi_hat)
%WEDGE_OPERATOR formes matrix m representing twist from 
%twist coordinates xi = (v ,w)'
m = [vec2skewSymMat(xi_hat(4:6,1)), xi_hat(1:3,1);
       0, 0, 0, 0];
end

