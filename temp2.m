q = rand(6,1);
qd = rand(6,1);
q2d = rand(6,1);

Yf = full_regressor(q,qd,q2d);
Yb = base_regressor(q,qd,q2d);

Yb2 = Yf*Pb';

norm(Yb2 - Yb)


