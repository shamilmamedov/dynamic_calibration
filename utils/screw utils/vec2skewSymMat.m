function m = vec2skewSymMat(v)
%Form skew - symmetric matrix from vector to be used in cross product
m = [0, -v(3), v(2);
        v(3), 0, -v(1);
        -v(2), v(1) 0];
end

