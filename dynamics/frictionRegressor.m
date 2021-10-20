function frctn = frictionRegressor(qd_fltrd)
% ----------------------------------------------------------------------
% The function computes friction regressor for each joint of the robot.
% Fv*qd + Fc*sign(qd) + F0, and the second one is continous,
% ---------------------------------------------------------------------
noJoints = size(qd_fltrd,1);
frctn = zeros(noJoints, noJoints*3);
for i = 1:noJoints
    frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
end

