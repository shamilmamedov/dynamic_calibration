function Y = regressorWithMotorDynamics(q,qd,q2d)
% ----------------------------------------------------------------------
% This function adds motor dynamics to rigid body regressor.
% It is simplified model of motor dynamics, it adds only reflected
% inertia i.e. I_rflctd = Im*N^2 where N is reduction ratio - I_rflctd*q_2d
% parameter is added to existing vector of each link [pi_i I_rflctd_i]
% so that each link has 11 parameters
% ----------------------------------------------------------------------
if size(q,1)==6 && size(q,2)==1 && size(qd,1)==6 && size(qd,2)==1 ...
        && size(q2d,1)==6 && size(q2d,2)==1
    Y_rgd_bdy = standard_regressor_UR10E(q,qd,q2d);
    Y_mtrs = diag(q2d);
    Y = [Y_rgd_bdy(:,1:10), Y_mtrs(:,1), Y_rgd_bdy(:,11:20), Y_mtrs(:,2),...
         Y_rgd_bdy(:,21:30), Y_mtrs(:,3), Y_rgd_bdy(:,31:40), Y_mtrs(:,4),...
         Y_rgd_bdy(:,41:50), Y_mtrs(:,5), Y_rgd_bdy(:,51:60), Y_mtrs(:,6)];
else
    error('Input dimension error!')
end
