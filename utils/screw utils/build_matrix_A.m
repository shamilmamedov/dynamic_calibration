function A = build_matrix_A(v, n_link, par)
A = [v(1:3), vec2skewSymMat(v(4:6)), zeros(3,6);
        zeros(3,1), -vec2skewSymMat(v(1:3)) + vec2skewSymMat(par.mass_origin(n_link,:)')*vec2skewSymMat(v(4:6)), vec2mat_ssmat(v(4:6))];
end

