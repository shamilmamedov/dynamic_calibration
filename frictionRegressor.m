function frctn = frictionRegressor(qd_fltrd)
% ----------------------------------------------------------------------
% The function computes friction regressor for each joint of the robot.
% There are two models, first one is discontinous, 
% Fv*qd + Fc*sign(qd) + F0, and the second one is continous,
% Fv*qd + Fc*tanh(qd/xi) + F0, where xi is small constant
% ---------------------------------------------------------------------
frctn = zeros(6,18);
for i = 1:6
    frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
%     frctn(i,3*i-2:3*i) = [qd_fltrd(i), tanh(qd_fltrd(i)/1e-2), 1]; 
end

%     xi = 1e-4;
%     cont_Clmb_frcn = 0;
%     
%     frctn = zeros(6,3);
%     if cont_Clmb_frcn 
%         for i = 1:6
%             frctn(i,:) = [qd_fltrd(i), tanh(qd_fltrd(i)/xi), 1];
%         end
%     else
%         for i = 1:6
%             frctn(i,:) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
%         end
%     end
    
%     for i = 1:6
%        frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
%     end
%     for i = 1:6
%        frctn(i,3*i-2:3*i) = [qd_fltrd(i), tanh(qd_fltrd(i)/1e-2), 1];
%     end