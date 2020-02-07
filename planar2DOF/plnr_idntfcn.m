clc; clear all; close all;

% Loading file from urdf
plnr = xml2struct('planar_manip.urdf');

% Extracting parameters of the robot
for i = 1:2
% axis of rotation of a joint i in coordinate system of joint i    
   axis_of_rot = str2num(plnr.robot.joint{i}.axis.Attributes.xyz)';
% mass of link (i+1) because joint i rotates link (i+1) as the numbering of
% links starts from base link that it not moving
   link_mass = str2double(plnr.robot.link{i+1}.inertial.mass.Attributes.value);
% poistion of the com in frame attached to link
   com_pos = str2num(plnr.robot.link{i+1}.inertial.origin.Attributes.xyz)';
   com_vec2mat = vec2skewSymMat(com_pos);
% inertial parameters of the link expressed in coordinate system attached
% the center of mass.
   ixx = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.ixx);
   ixy = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.ixy);
   ixz = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.ixz);
   iyy = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.iyy);
   iyz = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.iyz);
   izz = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.izz);
% the inertia tensor wrt the frame oriented as the body frame and with the
% origin in the COM
   link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
% manipulator regressor                               
   plnr.m(i) = link_mass;
   plnr.k(:,i) = axis_of_rot;
   plnr.r_com(:,i) = com_pos;
   plnr.I(:,:,i) = link_inertia;
   plnr.h(:,i) = link_mass*com_pos;
   plnr.I_vec(:,i) = inertiaMatrix2Vector(link_inertia-...
                            link_mass*com_vec2mat*com_vec2mat);
   plnr.pi(:,i) = [plnr.I_vec(:,i); plnr.h(:,i); plnr.m(i)];
end

return
% ------------------------------------------------------------------------
% Defining parameters symbolically
% ------------------------------------------------------------------------
m = sym('m%d',[2,1],'real');
hx = sym('h%d_x',[2,1],'real');
hy = sym('h%d_y',[2,1],'real');
hz = sym('h%d_z',[2,1],'real');
ixx = sym('i%d_xx',[2,1],'real');
ixy = sym('i%d_xy',[2,1],'real');
ixz = sym('i%d_xz',[2,1],'real');
iyy = sym('i%d_yy',[2,1],'real');
iyz = sym('i%d_yz',[2,1],'real');
izz = sym('i%d_zz',[2,1],'real');

% Vector of symbolic parameters
for i = 1:2
    pi_plnr_sym(:,i) = [ixx(i),ixy(i),ixz(i),iyy(i),iyz(i),izz(i),...
                        hx(i),hy(i),hz(i),m(i)]';
end

% ------------------------------------------------------------------------
% Symbolic generilized coordiates, their first and second deriatives
% ------------------------------------------------------------------------
q_sym = sym('q%d',[2,1],'real');
qd_sym = sym('qd%d',[2,1],'real');
q2d_sym = sym('q2d%d',[2,1],'real');

% ------------------------------------------------------------------------
% Getting energy functions, to derive dynamics
% ------------------------------------------------------------------------
T_pk = sym(zeros(4,4,2)); % transformation between links
w_kk(:,1) = sym(zeros(3,1)); % angular velocity k in frame k
v_kk(:,1) = sym(zeros(3,1)); % linear velocity of the origin of frame k in frame k
g_kk(:,1) = sym([0,0,9.81])'; % vector of graviatational accelerations in frame k
p_kk(:,1) = sym(zeros(3,1)); % origin of frame k in frame k
DK(:,1) = sym(zeros(10,1)); % gradient of kinetic energy
DP(:,1) = sym([zeros(1,6),[0,0,9.81],0]'); % gradient of gravitational energy

for i = 1:2
    jnt_axs_k = str2num(plnr.robot.joint{i}.axis.Attributes.xyz)';
% Transformation from parent link frame p to current joint frame
    rpy_k = sym(str2num(plnr.robot.joint{i}.origin.Attributes.rpy));
    R_pj = RPY(rpy_k);
%     R_pj(abs(R_pj)<sqrt(eps)) = sym(0); % to avoid numerical errors
    R_pj(abs(R_pj)<1e-5) = sym(0); % to avoid numerical errors
    p_pj = str2num(plnr.robot.joint{i}.origin.Attributes.xyz)';
    T_pj = sym([R_pj, p_pj; zeros(1,3), 1]); % to avoid numerical errors
% Tranformation from joint frame of the joint that rotaties body k to
% link frame. The transformation is pure rotation
    R_jk = Rot(q_sym(i),sym(jnt_axs_k));
    p_jk = sym(zeros(3,1));
    T_jk = [R_jk, p_jk; sym(zeros(1,3)),sym(1)];
% Transformation from parent link frame p to current link frame k
    T_pk(:,:,i) = T_pj*T_jk;
    z_kk(:,i) = sym(jnt_axs_k);
        
    w_kk(:,i+1) = T_pk(1:3,1:3,i)'*w_kk(:,i) + sym(jnt_axs_k)*qd_sym(i);
    v_kk(:,i+1) = T_pk(1:3,1:3,i)'*(v_kk(:,i) + cross(w_kk(:,i),sym(p_pj)));
    g_kk(:,i+1) = T_pk(1:3,1:3,i)'*g_kk(:,i);
    p_kk(:,i+1) = T_pk(1:3,1:3,i)'*(p_kk(:,i) + sym(p_pj));
    
    K_reg(i,:) = [sym(0.5)*w2wtlda(w_kk(:,i+1)),...
                    v_kk(:,i+1)'*vec2skewSymMat(w_kk(:,i+1)),...
                    sym(0.5)*v_kk(:,i+1)'*v_kk(:,i+1)];
    P_reg(i,:) = [sym(zeros(1,6)), g_kk(:,i+1)',...
                    g_kk(:,i+1)'*p_kk(:,i+1)];
    
    lmda(:,:,i) = getLambda(T_pk(1:3,1:3,i),sym(p_pj));
    f(:,i) = getF(v_kk(:,i+1),w_kk(:,i+1),sym(jnt_axs_k),qd_sym(i));
    DK(:,i+1) = lmda(:,:,i)*DK(:,i) + qd_sym(i)*f(:,i);
    DP(:,i+1) = lmda(:,:,i)*DP(:,i);
end

% Lagr = [K_reg(1,:) - P_reg(1,:), K_reg(2,:) - P_reg(2,:)];
% dLagr_dq = jacobian(Lagr,q_sym)';
% dLagr_dqd = jacobian(Lagr,qd_sym)';
% t1 = sym(zeros(2,20));
% for i = 1:2
%    t1 = t1 + diff(dLagr_dqd,q_sym(i))*qd_sym(i)+...
%                 diff(dLagr_dqd,qd_sym(i))*q2d_sym(i);
% end
% Y = t1 - dLagr_dq;
% simplify(Y)

% ------------------------------------------------------------------------
% Grouping paramters that can be grouped
% ------------------------------------------------------------------------
pi_plnr_num_rdcd = zeros(10,2);
pi_plnr_num_rdcd(:,2) = plnr.pi(:,2);

pi_plnr_sym_rdcd = sym(zeros(10,2));
pi_plnr_sym_rdcd(:,2) = pi_plnr_sym(:,2);
for i = 2:-1:2
    pi_prv_num = plnr.pi(:,i-1);
    [pi_plnr_num_rdcd(:,i-1),pi_plnr_num_rdcd(:,i)] = ...
            group(pi_prv_num,pi_plnr_num_rdcd(:,i),lmda(:,:,i),z_kk(:,i));
    
    pi_prv_sym = pi_plnr_sym(:,i-1);
    [pi_plnr_sym_rdcd(:,i-1),pi_plnr_sym_rdcd(:,i)] = ...
            group(pi_prv_sym, pi_plnr_sym_rdcd(:,i),lmda(:,:,i),z_kk(:,i));
end
pi_plnr_num_rdcd = double(pi_plnr_num_rdcd);

% ------------------------------------------------------------------------
% if paramerer doesn't contribute to energy then we remove it
% and compose lagrangian
% ------------------------------------------------------------------------
Lagr = [];
pi_base_vctr = [];
for i = 1:2
    DK_base{i} = sym([]);
    DP_base{i} = sym([]);
    pi_base_sym{i} = sym([]);
    pi_base_num{i} = [];
    for j = 1:10
%       We try to obtain base parameters, parameters tha affect energy
%       of the system, by anlayzing kinetic energy and potenial energy
%       as well as vector of regrouped parameters
% 
%       first condition says that if DK(j,i+1) and DP(j,i+1) are zero
%       or DK(j,i+1) is zero and DP(j,i+1) is constant
%       the second condition says that if we have regrouped some parmeter
%       with parameters of previous link so it is zero now
        if (DK(j,i+1) == 0 && (DP(j,i+1)==0 || isempty(symvar(DP(j,i+1)))))...
                || pi_plnr_sym_rdcd(j,i) == 0
            str = strcat('\nlink\t ',num2str(i), ',\tparameter\t',num2str(j));
            fprintf(str)
        else
            DK_base{i} = vertcat(DK_base{i},DK(j,i+1));
            DP_base{i} = vertcat(DP_base{i},DP(j,i+1));
            
            pi_base_sym{i} = vertcat(pi_base_sym{i},pi_plnr_sym_rdcd(j,i));
            pi_base_num{i} = vertcat(pi_base_num{i},pi_plnr_num_rdcd(j,i));
        end
    end
    fprintf('\n\n')
    Lagr = horzcat(Lagr, (DK_base{i} - DP_base{i})');
    pi_base_vctr = vertcat(pi_base_vctr,pi_base_num{i});
end

return
% ------------------------------------------------------------------------
% Using lagrange equations of the second kind, find dynamics
% ------------------------------------------------------------------------
dLagr_dq = jacobian(Lagr,q_sym)';
dLagr_dqd = jacobian(Lagr,qd_sym)';
t1 = sym(zeros(2,length(pi_base_vctr)));
for i = 1:2
   t1 = t1 + diff(dLagr_dqd,q_sym(i))*qd_sym(i)+...
                diff(dLagr_dqd,qd_sym(i))*q2d_sym(i);
end
Y_hat = t1 - dLagr_dq;
% bidlokod
Y_hat = [Y_hat(:,1:3), [qd_sym(1) sign(qd_sym(1)); 0 0], Y_hat(:,4:end), ...
            [ 0 0; qd_sym(2) sign(qd_sym(2))]];
Y_hat_fcn = matlabFunction(Y_hat,'Vars',{q_sym,qd_sym,q2d_sym});



% ------------------------------------------------------------------------
% Dynamics of the robot in classical form for control
% ------------------------------------------------------------------------
T_0k(:,:,1) = sym(eye(4));
Jv_0k = sym(zeros(3,2,2));
Jw_0k = sym(zeros(3,2,2));
K = sym(0); P = sym(0);
for i = 1:2
%       Transformation from parent link frame p to current joint frame
        R_pj = RPY(str2num(plnr.robot.joint{i}.origin.Attributes.rpy));
        p_pj = str2num(plnr.robot.joint{i}.origin.Attributes.xyz)';
        T_pj = [R_pj, p_pj; zeros(1,3), 1];
%       Tranformation from joint frame of the joint that rotaties body k to
%       link frame. The transformation is pure rotation
        R_jk = Rot(q_sym(i),plnr.k(:,i));
        p_jk = zeros(3,1);
        T_jk = [R_jk, p_jk; zeros(1,3),1];
%       Transformation from parent link frame p to current link frame k
        T_pk(:,:,i) = T_pj*T_jk;
        T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
        z_0k(:,i) = T_0k(1:3,1:3,i+1)*plnr.k(:,i);
        
        r_0k(:,i) = [eye(3),zeros(3,1)]*T_0k(:,:,i+1)*[plnr.r_com(:,i);1];
        
        for j = 1:i
           Jv_0k(:,j,i) = cross(z_0k(:,j),r_0k(:,i)-T_0k(1:3,4,j+1));
           Jw_0k(:,j,i) = z_0k(:,j);
        end
        
        K = K + 0.5*qd_sym'*(plnr.m(i)*(Jv_0k(:,:,i)'*Jv_0k(:,:,i)) + ...
                Jw_0k(:,:,i)'*T_0k(1:3,1:3,i+1)*plnr.I(:,:,i)*...
                T_0k(1:3,1:3,i+1)'*Jw_0k(:,:,i))*qd_sym;
        P = P + plnr.m(i)*[0;0;9.81]'*r_0k(:,i);
end
Lagr = K - P;
dLagr_dq = jacobian(Lagr,q_sym)';
dLagr_dqd = jacobian(Lagr,qd_sym)';
t1 = jacobian(dLagr_dqd,[q_sym;qd_sym])*[qd_sym; q2d_sym];
t1 = simplify(t1);
dnmcs = t1 - dLagr_dq;
M_sym = jacobian(dnmcs,q2d_sym);
n_sym = simplify(dnmcs - M_sym*q2d_sym);

M_mtrx_fcn = matlabFunction(M_sym,'Vars',{q_sym});
n_vctr_fcn = matlabFunction(n_sym,'Vars',{q_sym,qd_sym});

%% ------------------------------------------------------------------------
% Trajectory planning (desired trajectory parameters)
% ------------------------------------------------------------------------
T = 5; %period of oscilllations
w = 2*pi/T;
N = 3; %number of harmonics


q0 = 0;
a = [0.22    0.41    0.13];
b = [1.58    0.49    0.65];

a2 = [1.21    0.71    0.63];
b2 = [0.27    0.18    0.05];

a3 = [-1.3    0.7    0.12];
b3 = [0    0    0];

a4 = rand(1,3);
b4 = rand(1,3);
% ------------------------------------------------------------------------
% Simulating planar manipulator motion
% ------------------------------------------------------------------------
tf = 5; % simulation time
t_smp = 1e-3;
t = 0:t_smp:tf;

[q_des,qd_des,~] = des_traj(t,q0,a,b,w,N);

[q_des2,qd_des2,~] = des_traj(t,q0,a2,b2,w,N);

[q_des3,qd_des3,~] = des_traj(t,q0,a3,b3,w,N);

%{
fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 10 9]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
subplot(2,1,1)
    ax = gca;
    ax.Units='centimeters';
    set(gca,'TickLabelInterpreter','latex')
    hold(ax,'on')
    plot(t,q_des,'k-')
    plot(t,q_des2,'k--')
    plot(t,q_des3,'k-.')
    box(ax,'on');
    grid(ax,'on');
    legend('$q_1$, rad','$q_2$, rad','$q_3$, rad','Interpreter','latex')
subplot(2,1,2)
    ax = gca;
    ax.Units='centimeters';
    set(gca,'TickLabelInterpreter','latex')
    hold(ax,'on')
    plot(t,qd_des,'k-')
    plot(t,qd_des2,'k--')
    plot(t,qd_des3,'k-.')
    xlabel('$t$, sec','interpreter','latex')
    box(ax,'on');
    grid(ax,'on');
    legend('$\dot{q}_1$, rad/s','$\dot{q}_2$, rad/s','$\dot{q}_3$, rad/s','Interpreter','latex')
%}
% hgexport(fig,'traj1');% writes figure fig to the EPS file filename

% Controller paramters
kp = 45;
kd = 10;

x(1,:) = [q_des(1) 0 0 0];
for i = 1:length(t)-1
   qi = x(i,1:2)';
   qdi = x(i,3:4)';
   
   [qdesi,qddesi,~] = des_traj(t(i),q0,a4,b4,w,N);
   ei = qdesi - qi(1);
   edi = qddesi - qdi(1);
   
   ui = [kp*ei + kd*edi, 0]';
   
   [ti,xi] = ode45(@(t,x)ode_plnr(t,x,M_mtrx_fcn,n_vctr_fcn,ui), [t(i) t(i+1)], x(i,:));
   
   u(i,:) = ui';
   x(i+1,:) = xi(end,:);
end

% plnr_visualize(x(:,1:2),plnr)
%%


fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 16 5]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
subplot(1,2,1)
    ax = gca;
    ax.Units='centimeters';
    set(gca,'TickLabelInterpreter','latex')
    hold(ax,'on')
    plot(t,x(:,1),'k')
%     plot(t,q_des,'b--')
    plot(t,x(:,2),'k--')
    xlim([0 5])
    box(ax,'on');
    grid(ax,'on');
    legend('$q_1$, rad','$q_2$, rad','Interpreter','latex')
subplot(1,2,2)
    ax = gca;
    ax.Units='centimeters';
    set(gca,'TickLabelInterpreter','latex')
    hold(ax,'on')
    plot(t,x(:,3),'k')
%     plot(t,qd_des,'b--')
    plot(t,x(:,4),'k--')
    xlim([0 5])
    xlabel('$t$, sec','interpreter','latex')
    box(ax,'on');
    grid(ax,'on');
    legend('$\dot{q}_1$, rad/s','$\dot{q}_2$, rad/s','Interpreter','latex')
    
% ------------------------------------------------------------------------
% Identification
% ------------------------------------------------------------------------

% Numerical differentiation to find q2d
q2d_est(1,:) = [0, 0];
for i = 2:length(t)-1
   q2d_est(i,:) = (x(i+1,3:4) - x(i-1,3:4))/(2*t_smp);
end

tau_bar = [];
Y_bar = [];
for i = 1:length(t)-1
   tau_bar = vertcat(tau_bar,u(i,:)');
      
   qi = x(i,1:2)';
   qdi = x(i,3:4)';
   q2di = q2d_est(i,:)';
   
   Yi = Y_hat_fcn(qi,qdi,q2di);
   Y_bar = vertcat(Y_bar,Yi);
end

pi_hat = (Y_bar'*Y_bar)\(Y_bar'*tau_bar);

% ------------------------------------------------------------------------
%-------------------------------------------------------------------------
pi1 = [1.2038  0.9515  -0.001  2.0585  1.1304 ...
        0.4048  0.2222  -0.0002  0.5053  0.2281]';

pi2 = [1.1932 0.9462 0.0065 2.0037 1.1658 ...
        0.4115 0.2258 0 0.4998 0.2468]';
    
pi3 = [1.1942 0.9480 0.0028 2.0249 1.1478 ...
        0.4099 0.2237 0.0001 0.5173 0.2144]';
    
for i = 1:length(t)-1
     
   qi = x(i,1:2)';
   qdi = x(i,3:4)';
   q2di = q2d_est(i,:)';
   
   Yi = Y_hat_fcn(qi,qdi,q2di);
   u1(i,:) = Yi*pi1;
   u2(i,:) = Yi*pi1;
   u3(i,:) = Yi*pi1;
end

fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 10 5]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
ax = gca;
ax.Units='centimeters';
set(gca,'TickLabelInterpreter','latex')
hold(ax,'on')
plot(t(1:end-1),u(:,1),'k')
plot(t(1:end-1),u1(:,1),'k--')
plot(t(1:end-1),u(:,1)-u1(:,1),'k-.')
xlim([0 5])
box(ax,'on');
grid(ax,'on');
legend('$u$, Nm','$u_1$, Nm','$u - u_1$, Nm','Interpreter','latex')

fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 10 5]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
ax = gca;
ax.Units='centimeters';
set(gca,'TickLabelInterpreter','latex')
hold(ax,'on')
plot(t(1:end-1),u(:,1),'k')
plot(t(1:end-1),u2(:,1),'k--')
plot(t(1:end-1),u(:,1)-u2(:,1),'k-.')
xlim([0 5])
box(ax,'on');
grid(ax,'on');
legend('$u$, Nm','$u_2$, Nm','$u - u_2$, Nm','Interpreter','latex')

fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 10 5]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
ax = gca;
ax.Units='centimeters';
set(gca,'TickLabelInterpreter','latex')
hold(ax,'on')
plot(t(1:end-1),u(:,1),'k')
plot(t(1:end-1),u3(:,1),'k--')
plot(t(1:end-1),u(:,1)-u3(:,1),'k-.')
xlim([0 5])
box(ax,'on');
grid(ax,'on');
legend('$u$, Nm','$u_3$, Nm','$u - u_3$, Nm','Interpreter','latex')

return
% ------------------------------------------------------------------------
% Testing inverse dynamics with base parmaters
% ------------------------------------------------------------------------
q = pi*rand(2,1);
qd = rand(2,1);
q2d = rand(2,1);

rbt = importrobot('planar_manip.urdf');
rbt.DataFormat = 'column';
rbt.Gravity = [0 0 -9.81];

id_matlab = inverseDynamics(rbt,q,qd,q2d)
id_base = Y_hat_fcn(q,qd,q2d)*pi_base_vctr
id_full = M_mtrx_fcn(q)*q2d + n_vctr_fcn(q,qd)


function dxdt = ode_plnr(t,x,M,n,u)
    dxdt = zeros(4,1);
    
    vsc_frcn = [2 0; 0 0.5]*x(3:4);
    clmb_frcn = [1.2 0; 0 0.25]*tanh(x(3:4)./0.05);%sign(x(3:4));
    
    dxdt(1:2) = x(3:4);
    dxdt(3:4) = M(x(1:2))\(u - n(x(1:2),x(3:4)) - vsc_frcn - clmb_frcn);
end

function plnr_visualize(q,plnr)
    figure
    for j = 1:25:length(q)
        clf
        qj = q(j,:);

        T_0k(:,:,1) = eye(4);
        for i = 1:2
            R_pj = RPY(str2num(plnr.robot.joint{i}.origin.Attributes.rpy));
            p_pj = str2num(plnr.robot.joint{i}.origin.Attributes.xyz)';
            T_pj = [R_pj, p_pj; zeros(1,3), 1];

            R_jk = Rot(qj(i),plnr.k(:,i));
            p_jk = zeros(3,1);
            T_jk = [R_jk, p_jk; zeros(1,3),1];
            T_pk(:,:,i) = T_pj*T_jk;

            T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
            z_0k(:,i) = T_0k(1:3,1:3,i+1)*plnr.k(:,i);
            r_0k(:,i) = [eye(3),zeros(3,1)]*T_0k(:,:,i+1)*[plnr.r_com(:,i);1];
        end

        MARKER_SIZE = 100; %marker size of the scatter function
        scatter(T_0k(1,4,2), T_0k(3,4,2),MARKER_SIZE,'k','filled')
        hold on
        scatter(T_0k(1,4,3), T_0k(3,4,3),MARKER_SIZE,'k','filled')
        % scatter(Tee0(1,3), Tee0(2,3))
        line([T_0k(1,4,2) T_0k(1,4,3)], [T_0k(3,4,2), T_0k(3,4,3)],'Color','k','LineWidth',1.5)
        line([T_0k(1,4,3) r_0k(1,2)], [T_0k(3,4,3), r_0k(3,2)],'Color','k','LineWidth',1.5)
        xlim([-2.5 2.5])
        ylim([-2.5 2.5])

        pause(1e-1)
    end
end


function [q,qd,q2d] = des_traj(t,q0,a,b,w,N)
    q = q0;
    qd = 0;
    q2d = 0;
    for k = 1:N
       q = q + a(k)/(w*k).*sin(w*k*t) - b(k)/(w*k).*cos(w*k*t);
       qd = qd + a(k).*cos(w*k*t) + b(k)*sin(w*k*t);
       q2d = q2d - a(k)*w*k.*sin(w*k*t) + b(k)*w*k*cos(w*k*t);
    end
end