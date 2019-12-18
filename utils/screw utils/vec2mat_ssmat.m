function m = vec2mat_ssmat(w)

m = [w(1),   0,     0,      w(2),   w(3),   0;
     0,      w(2),  0,      w(1),   0,      w(3);
     0,      0,     w(3),   0 ,     w(1),   w(2)];

end

