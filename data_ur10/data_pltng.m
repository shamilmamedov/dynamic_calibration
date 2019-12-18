% ------------------------------------------------------------------------
% Positions
% ------------------------------------------------------------------------
% %{ 
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(t_msrd, q_msrd(:,i))
        plot(traj_par.t, q(i,:), '--')
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$q_1$, rad','$q_2$, rad','$q_3$, rad','interpreter','latex')
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(t_msrd, qd_msrd(:,i))
        plot(traj_par.t, qd(i,:), '--')
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$q_4$, rad','$q_5$, rad','$q_6$, rad','interpreter','latex')
    grid on
%}
% ------------------------------------------------------------------------
% Velocities
% ------------------------------------------------------------------------
%{
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(t_msrd, qd_msrd(:,i))
        plot(traj_par.t, qd(i,:), '--')
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$\dot{q}_1$, rad/s','$\dot{q}_2$, rad/s','$\dot{q}_3$, rad/s',...
                'interpreter','latex')
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(t_msrd, qd_msrd(:,i))
        plot(traj_par.t, qd(i,:), '--')
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$\dot{q}_4$, rad/s','$\dot{q}_5$, rad/s','$\dot{q}_6$, rad/s',...
                'interpreter','latex')
    grid on
%}

% ------------------------------------------------------------------------
% Accelerations
% ------------------------------------------------------------------------
%{
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(t_msrd, q2d_est(:,i))
        plot(traj_par.t, q2d(i,:), '--')
    end
    xlabel('$t$,\ sec','interpreter','latex')
    ylabel('$\ddot{q}$,\ rad','interpreter','latex')
    ylim([-2.5, 2.5])
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(t_msrd, q2d_est(:,i))
        plot(traj_par.t, q2d(i,:), '--')
    end
    xlabel('$t$,\ sec','interpreter','latex')
    ylabel('$\ddot{q}$,\ rad','interpreter','latex')
    ylim([-2.5, 2.5])
    grid on
%} 

% ------------------------------------------------------------------------
% measured torque vs desired torque
% ------------------------------------------------------------------------
%{
for i = 1:6
    figure
    plot(t_msrd,i_msrd(:,i))
    hold on
    plot(t_msrd,i_des(:,i))
    grid on
    legend('msrd','dsrd','interpreter','latex')
end
%}

% ------------------------------------------------------------------------
% currents vs Filteres currents
% ------------------------------------------------------------------------
%{
figure
subplot(2,1,1)
    hold on
    for i = 1:3
        plot(t_msrd,i_msrd(:,i))
        plot(t_msrd,i_fltrd(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$i_1$, A','$i_2$, A','$i_3$, A','interpreter','latex')
    grid on
subplot(2,1,2)
    hold on
    for i = 4:6
        plot(t_msrd,i_msrd(:,i))
        plot(t_msrd,i_fltrd(:,i))
    end
    xlabel('$t$,\ sec','interpreter','latex')
    legend('$i_4$, A','$i_5$, A','$i_6$, A','interpreter','latex')
    grid on

%}



