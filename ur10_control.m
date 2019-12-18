% Gains are taken from here
% https://github.com/PositronicsLab/reveal_packages/blob/master/
%                               industrial_arm/scenario/arm_controller.h
% // setup gains
% const double SH_KP = 300.0, SH_KV = 120.0;
% const double EL_KP = 60.0, EL_KV = 24.0;
% const double WR_KP = 15.0, WR_KV = 6.0;

SH_KP = 300; SH_KV = 100;
% EL_KP = 60; EL_KV = 24;
% WR_KP = 15; WR_KV = 6;
EL_KP = 180; EL_KV = 35;
WR_KP = 10; WR_KV = 1;


Kp = diag([SH_KP,SH_KP,EL_KP,WR_KP,WR_KP,1]);
Kd = diag([SH_KV,SH_KV,EL_KV,WR_KV,WR_KV,0.05]);

% -----------------------------------------------------------------------
% Trajectory
% -----------------------------------------------------------------------
T = 4;
w = 2*pi/T;

t = 0:0.01:2;
q0 = -0.2; %offset of the position trajectory
a = [0.1;0.1];
b = [0.1;0.1];

[q1_des,q1d_des] = periodic_trajecctory(t,q0,a,b,w,2);
[q2_des,q2d_des] = periodic_trajecctory(t,q0,a,b,w,2);
[q3_des,q3d_des] = periodic_trajecctory(t,q0,a,b,w,2);
[q4_des,q4d_des] = periodic_trajecctory(t,q0,a,b,w,2);
[q5_des,q5d_des] = periodic_trajecctory(t,q0,a,b,w,2);
[q6_des,q6d_des] = periodic_trajecctory(t,q0,a,b,w,2);

q_des = [q1_des;q2_des;q3_des;q4_des;q5_des;q6_des];
qd_des = [q1d_des;q2d_des;q3d_des;q4d_des;q5d_des;q6d_des];

% figure
% plot(t,q1_des)
% hold on
% plot(t,q1d_des,'-.')

x(:,1) = zeros(12,1);
for i = 1:length(t)-1
    tspan = [t(i),t(i+1)];
   err = q_des(:,i) - x(1:6,i);
   err_d = qd_des(:,i) - x(7:12,i);
%    err = zeros(6,1) - x(1:6,i);
%    err_d = zeros(6,1) - x(7:12,i);

   [~,~,G] = screw_MCG(x(1:6,i),zeros(6,1),ur10);
   u = G + Kp*err + Kd*err_d;
   
   [ti,xi] = ode45(@(t,x)ode_ur10(t,x,u,ur10),tspan,x(:,i));
   x(:,i+1) = xi(end,:);
    
end

figure
plot(t,q_des(1,:))
hold on
for i = 1:3
    plot(t,x(i,:))
end
legend('ref','1','2','3')

figure
plot(t,q_des(4,:))
hold on
for i = 4:6
    plot(t,x(i,:))
end
legend('ref','4','5','6')


function [q_des,qd_des] = periodic_trajecctory(t,q0,a,b,w,N)
    q_des = q0 + a'*sin((1:N)'*w.*t) + b'*cos((1:N)'*w.*t);
    qd_des = w*(a'.*(1:N)*cos((1:N)'*w.*t) - b'.*(1:N)*sin((1:N)'*w.*t));
end

function dxdt = ode_ur10(t,x,u,ur10)
    dxdt = zeros(12,1);
    dxdt(1:6) = x(7:12);
    
    [M,C,G] = screw_MCG(x(1:6),x(7:12),ur10);
    
    dxdt(7:12) = M\(u-C*x(7:12)-G);
end