function m = inv_Ad_transf(g)
R = g(1:3,1:3);
p = g(1:3,4);
t1 = vec2skewSymMat(p);

m = [R', -R'*t1;
     zeros(3,3), R'];
end

