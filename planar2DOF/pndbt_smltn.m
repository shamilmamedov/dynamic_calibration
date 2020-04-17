clc; clear all; close all;


robot = importrobot('planar_manip.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

%%
% forwardDynamics(robot,configuration,jointVel,jointTorq)
q0 = [-deg2rad(90), 0]';
qd0 = [0, 0]';


t = [0, 5];
x0 = [q0; qd0];
[t, x] = ode45(@(t,x)pendubot_ode(t,x,robot) , t, x0);

%% 
q = x(:,1:2);
qd = x(:,3:4);
plnr = parse_urdf('planar_manip.urdf');
plnr_visualize(q, plnr)


function u = PDcontroller(x)
    Kp = 2;
    Kd = 0.1;
    x_des = [-deg2rad(70); 0];
    u = [Kp Kd]*(x - x_des);
end


function dxdt = pendubot_ode(t, x, robot)
    q = x(1:2);
    qd = x(3:4);
    B = [0; 1];
    M = massMatrix(robot, q);
    Cqd = velocityProduct(robot, q, qd);
    g = gravityTorque(robot, q);
    u = PDcontroller(x([1 3]));
    dxdt = [qd;
            M\(B*u - Cqd - g)];
end
