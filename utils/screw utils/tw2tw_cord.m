function xi = tw2tw_cord(m)
%VEE_OPERATOR - extracts twist coordinates from a matrix m representing
%twist
xi          = zeros(6,1);
xi(1:3,1)   = m(1:3,4);
xi(4,1)     = m(3,2);
xi(5,1)     = m(1,3);
xi(6,1)     = m(2,1);
end

