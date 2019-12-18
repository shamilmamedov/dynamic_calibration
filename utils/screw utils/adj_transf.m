function m = adj_transf(v)
t1 = vec2skewSymMat(v(1:3,1));
t2 = vec2skewSymMat(v(4:6,1)); 

m = [t2, t1;
     zeros(3,3), t2];
end

