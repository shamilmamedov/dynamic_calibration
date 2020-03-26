clear all; close all; clc;
%-------------------------------------------------------------------------
% Loading file from urdf
% ------------------------------------------------------------------------

ur10 = xml2struct('ur10e.urdf');

% ------------------------------------------------------------------------
% Building screw axes, paramter vector and generilized mass matrix for
% further computations, %     plot(traj_par.t,q)
% i.e. dynamics, regresor ... .
% ------------------------------------------------------------------------
ur10.XI = zeros(6,6,6);
ur10.Lmbd_k = zeros(6,6,6); % constant body inertia matrix
ur10.pi_scrw = zeros(10,6);

for i = 1:6
    % axis of rotation of a joint i in coordinate system of joint i    
    axis_of_rot = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
   
    % screw axis of joint i in coordiante frame of joint i, but written in a
    % matrix form to account for more complex type of joints i.e. spherical
    ur10.XI(:,i,i) =  [0; 0; 0; axis_of_rot];
    
    % mass of link (i+1) because joint i rotates link (i+1) as the numbering of
    % links starts from base link that it not moving
    link_mass = str2double(ur10.robot.link{i+1}.inertial.mass.Attributes.value);
    
    % poistion of the com in frame attached to link
    com_pos = str2num(ur10.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    com_vec2mat = vec2skewSymMat(com_pos);
    
    % inertial parameters of the link expressed in coordinate system attached
    % the center of mass.
    ixx = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.ixx);
    ixy = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.ixy);
    ixz = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.ixz);
    iyy = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.iyy);
    iyz = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.iyz);
    izz = str2double(ur10.robot.link{i+1}.inertial.inertia.Attributes.izz);
    
    % the inertia tensor wrt the frame oriented as the body frame and with the
    % origin in the COM
    link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
    
    % generilized mass matrix expressed in the frame attached to the origin of
    % the link.
    ur10.Lmbd_k(:,:,i) = [link_mass*eye(3,3), -link_mass*com_vec2mat;
                          link_mass*com_vec2mat, link_inertia - ...   
                                link_mass*com_vec2mat*com_vec2mat];
    
    % manipulator regressor. It is true for regressor matrix obtained
    % using screw notation
    ur10.pi_scrw(:,i) = [link_mass;link_mass*com_pos;ixx;iyy;izz;ixy;ixz;iyz];
   
    % mass, inertia, first moment of inertia and inertia vector 
    ur10.m(i) = link_mass;
    ur10.I(:,:,i) = link_inertia;
    ur10.h(:,i) = link_mass*com_pos;
    ur10.I_vec(:,i) = inertiaMatrix2Vector(link_inertia-...
                            link_mass*com_vec2mat*com_vec2mat);
    ur10.pi(:,i) = [ur10.I_vec(:,i);ur10.h(:,i);ur10.m(i)];
end



% ------------------------------------------------------------------------
% Testing Dynamics. We compare inverse kinematics using calssical
% manipulator mode - M*q_2d + C*q_d + G = tau, inverse kinematics using
% regressor - Y*pi = tau, and matlab Robotic Systems Toolbox inverse
% dynamics command.
% ------------------------------------------------------------------------
TEST_DYNAMICS = 0;

if TEST_DYNAMICS
    rbt = importrobot('ur10e.urdf');
    rbt.DataFormat = 'column';
    rbt.Gravity = [0 0 -9.81];
    
    load('baseQR');
    
    noIter = 100;
    err1 = zeros(noIter,1); err2 = zeros(noIter,1); 
    err3 = zeros(noIter,1); err4 = zeros(noIter,1);
    for i = 1:noIter
        q = -2*pi + 4*pi*rand(6,1);
        q_d = zeros(6,1);
        q_2d = zeros(6,1);

        [M,C,G] = screw_MCG(q,q_d,ur10);
        Yscrw = screw_regressor(q,q_d,q_2d,ur10);
        Ylgr = full_regressor_UR10E(q,q_d,q_2d);
        
        tau1 = M*q_2d + C*q_d + G;
        tau2 = Yscrw*reshape(ur10.pi_scrw,[60,1]);
        tau3 = inverseDynamics(rbt,q,q_d,q_2d);
        tau4 = Ylgr*reshape(ur10.pi,[60,1]);
        tau5 = M_mtrx_fcn(q, ur10.pi(:))*q_2d + ...
               C_mtrx_fcn(q, q_d, ur10.pi(:))*q_d + ...
               G_vctr_fcn(q, ur10.pi(:));
                
    %   verifying if regressor is computed correctly  
        err1(i) = norm(tau3 - tau1);
    %   verifying if our inverse dynamics coincides with matlabs
        err2(i) = norm(tau3 - tau2);
        err3(i) = norm(tau3 - tau4);
        err4(i) = norm(tau3 - tau5);
    end

    figure
    subplot(2,1,1)
        plot(err1)
        hold on
        plot(err2)
        legend('||\Delta MCG||', '||\Delta Yscrw||')
        grid on
    subplot(2,1,2)
        plot(err3)
        hold on
        plot(err4)
        legend('||\Delta Ylgr||', '||\Delta MCG||')
        grid on
end














