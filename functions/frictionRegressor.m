function frctn = frictionRegressor(qd_fltrd)
% ----------------------------------------------------------------------
% The function computes friction regressor for each joint of the robot.
% Fv*qd + Fc*sign(qd) + F0, and the second one is continous,
% ---------------------------------------------------------------------
frctn = zeros(6,18);
for i = 1:6
    frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
end

