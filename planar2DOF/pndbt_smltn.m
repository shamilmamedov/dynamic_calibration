clc; clear all; close all;


robot = importrobot('planar_manip.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];


%%
% forwardDynamics(robot,configuration,jointVel,jointTorq)
q0 = [1, 1]';
qd0 = [0, 0]';
x0 = [q0; qd0];

t = [0, 4];
[t, x] = ode45(@(t,x)plnrMnpltr_ode(t,x,robot) , t, x0);


%% visualize motion
q = x(:,1:2);
qd = x(:,3:4);
plnr = parse_urdf('planar_manip.urdf');
plnr_visualize(q, plnr)


%% plot trajectories
figure
subplot(2,1,1)
    plot(t, x(:,1))
    hold on
    plot(t, x(:,2))
    plot(t, cos(t))
    legend('$q_1$', '$q_2$', '$q_{ref}$', 'Interpreter', 'latex')
    xlabel('$t$, sec', 'Interpreter', 'latex') 
    ylabel('$q$, rad', 'Interpreter', 'latex')
    grid minor
subplot(2,1,2)
    plot(t, x(:,3))
    hold on
    plot(t, x(:,4))
    plot(t, -sin(t))
    legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_{ref}$', 'Interpreter', 'latex')
    xlabel('$t$, sec', 'Interpreter', 'latex') 
    ylabel('$\dot{q}$, rad/s', 'Interpreter', 'latex')
    grid minor







function x_des = refrenceTrajectory(t)
    x_des = [cos(t); cos(t); -sin(t) ; -sin(t)];
end

function u = PDcontroller2(x, x_des)
    Kp = diag([20 15]);
    Kd = diag([2 1]);
    u = [Kp Kd]*(x_des - x);
end

function dxdt = plnrMnpltr_ode(t, x, robot)
    q = x(1:2);
    qd = x(3:4);
    M = massMatrix(robot, q);
    Cqd = velocityProduct(robot, q, qd);
    g = gravityTorque(robot, q);
    x_des = refrenceTrajectory(t);
    u = PDcontroller2(x, x_des);
    dxdt = [qd;
            M\(u - Cqd - g)];
end









function u = PDcontroller(x)
    Kp = 10;
    Kd = 1;
    x_des = [-deg2rad(70); 0];
    u = [Kp Kd]*(x_des - x);
end

function dxdt = pendubot_ode(t, x, robot)
    q = x(1:2);
    qd = x(3:4);
    B = [1; 0];
    M = massMatrix(robot, q);
    Cqd = velocityProduct(robot, q, qd);
    g = gravityTorque(robot, q);
    u = PDcontroller(x([1 3]));
    dxdt = [qd;
            M\(B*u - Cqd - g)];
end