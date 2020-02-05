function Y = regressorWithMotorDynamicsPndbt(q,qd,q2d)
% ----------------------------------------------------------------------
% This function adds motor dynamics to rigid body regressor.
% It is simplified model of motor dynamics, it adds only reflected
% inertia i.e. I_rflctd = Im*N^2 where N is reduction ratio - I_rflctd*q_2d
% parameter is added to existing vector of each link [pi_i I_rflctd_i]
% so that each link has 11 parameters
% ----------------------------------------------------------------------
if size(q,1)==2 && size(q,2)==1 && size(qd,1)==2 && size(qd,2)==1 ...
        && size(q2d,1)==2 && size(q2d,2)==1
    Y_rgd_bdy = full_regressor_plnr(q,qd,q2d);
    Y = [Y_rgd_bdy(:,1:10), [q2d(1) 0]', Y_rgd_bdy(:,11:20)];
else
    error('Input dimension error!')
end
