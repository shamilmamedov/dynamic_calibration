function robot = parse_urdf(file)
% Loading file from urdf
% The function is tailored for UR robots: serial robots with 6 DOF
% Modify it if you have  a robot with different degrees of freedom
robot = xml2struct(file);
no_dof = 6;

% Extracting parameters of the robot
for i = 1:no_dof
    % axis of rotation of a joint i in coordinate system of joint i    
    axis_of_rot = str2num(robot.robot.joint{i}.axis.Attributes.xyz)';
    % mass of link (i+1) because joint i rotates link (i+1) as the numbering of
    % links starts from base link that it not moving
    link_mass = str2double(robot.robot.link{i+1}.inertial.mass.Attributes.value);
    % poistion of the com in frame attached to link
    com_pos = str2num(robot.robot.link{i+1}.inertial.origin.Attributes.xyz)';
    com_vec2mat = vec2skewSymMat(com_pos);
    % inertial parameters of the link expressed in coordinate system attached
    % the center of mass.
    ixx = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixx);
    ixy = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixy);
    ixz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.ixz);
    iyy = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.iyy);
    iyz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.iyz);
    izz = str2double(robot.robot.link{i+1}.inertial.inertia.Attributes.izz);
    % the inertia tensor wrt the frame oriented as the body frame and with the
    % origin in the COM
    link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
    % manipulator regressor                               
    robot.m(i) = link_mass;
    robot.k(:,i) = axis_of_rot;
    robot.r_com(:,i) = com_pos;
    robot.I(:,:,i) = link_inertia;
    robot.h(:,i) = link_mass*com_pos;
    robot.I_vec(:,i) = inertiaMatrix2Vector(link_inertia-...
                            link_mass*com_vec2mat*com_vec2mat);
    robot.pi(:,i) = [robot.I_vec(:,i); robot.h(:,i); robot.m(i)];
end