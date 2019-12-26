% ---------------------------------------------------------------------
% In this script we plot data processing of real data from UR
% ---------------------------------------------------------------------
trajectory = unloadedTrajectory; % choose trajectory

%% Positions
% Ploting ideal posititions and obtained ones
% %{ 
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(trajectory.t, trajectory.q(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$q_1$','$q_2$','$q_3$','interpreter','latex')
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(trajectory.t, trajectory.q(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$q_4$','$q_5$','$q_6$','interpreter','latex')
    grid on
%}

%% Velocities
% %{
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(trajectory.t, trajectory.qd(:,i))
        plot(trajectory.t, trajectory.qd_fltrd(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$',...
                'interpreter','latex')
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(trajectory.t, trajectory.qd(:,i))
        plot(trajectory.t, trajectory.qd_fltrd(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$',...
                'interpreter','latex')
    grid on
%}

%% Accelerations
% %{
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(trajectory.t, trajectory.q2d_est(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    ylabel('$\ddot{q}$,\ rad','interpreter','latex')
    legend('$\ddot{q}_1$','$\ddot{q}_2$','$\ddot{q}_3$',...
                'interpreter','latex')
    ylim([-2.5, 2.5])
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(trajectory.t, trajectory.q2d_est(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    ylabel('$\ddot{q}$,\ rad','interpreter','latex')
    legend('$\ddot{q}_4$','$\ddot{q}_5$','$\ddot{q}_6$',...
                'interpreter','latex')
    ylim([-2.5, 2.5])
    grid on
%} 

%% measured currents vs desired currents
% %{
for i = 1:6
    figure
    plot(trajectory.t,trajectory.i(:,i))
    hold on
    plot(trajectory.t,trajectory.i_des(:,i))
    grid on
    legend('msrd','dsrd','interpreter','latex')
end
%}

%% currents vs Filteres currents
% %{
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(trajectory.t,trajectory.i(:,i))
        plot(trajectory.t,trajectory.i_fltrd(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$i_1$, A','$i_2$, A','$i_3$, A','interpreter','latex')
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(trajectory.t,trajectory.i(:,i))
        plot(trajectory.t,trajectory.i_fltrd(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$i_4$, A','$i_5$, A','$i_6$, A','interpreter','latex')
    grid on

%}
