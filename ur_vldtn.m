% ------------------------------------------------------------------------
% Load validation trajectory
% ------------------------------------------------------------------------

% vldtnTrjctry = parseURData('ur-19_10_01-14_04_13.csv', 1, 759);
% vldtnTrjctry = parseURData('ur-19_10_01-13_51_41.csv', 1, 660);
% vldtnTrjctry = parseURData('ur-19_09_27-11_28_22.csv', 1, 594);
vldtnTrjctry = parseURData('ur-20_01_17-p2.csv', 1, 250);
% vldtnTrjctry = parseURData('ur-20_01_17-ptp_10_points.csv', 1, 5346);
vldtnTrjctry = filterData(vldtnTrjctry);


% -----------------------------------------------------------------------
% Predicting torques
% -----------------------------------------------------------------------
%Constracting regressor matrix
tau_msrd = []; tau_prdctd1 = []; tau_prdctd2 = [];
i_prdct1 = []; i_prdct2 = [];
for i = 1:length(vldtnTrjctry.t)
    Yi = regressorWithMotorDynamics(vldtnTrjctry.q(i,:)',...
                                    vldtnTrjctry.qd_fltrd(i,:)',...
                                    vldtnTrjctry.q2d_est(i,:)');
    
    tau_withoutFriction = Yi*E1*pi_b;
    
    tau_lnr_frcn = zeros(6,1);
%     tau_nonlnr_frcn = zeros(6,1);
    for j = 1:6
        tau_lnr_frcn(j) = linearFrictionModel(pi_frctn(3*(j-1)+1:3*(j-1)+3),...
                                              vldtnTrjctry.qd_fltrd(i,j)');
%         tau_nonlnr_frcn(j) = nonlinearFrictionModel(pi_nonlnr_frcn(5*(j-1)+1:5*(j-1)+5),...
%                                                     vldtnTrjctry.qd_fltrd(i,j)');
    end
    tau_msrd = horzcat(tau_msrd, diag(drvGains)*vldtnTrjctry.i(i,:)');
    tau_prdctd1 = horzcat(tau_prdctd1, tau_withoutFriction + tau_lnr_frcn);
%     tau_prdctd2 = horzcat(tau_prdctd2, tau_withoutFriction + tau_nonlnr_frcn);
    
    i_prdct1 = horzcat(i_prdct1, diag(drvGains)\(tau_withoutFriction + tau_lnr_frcn));
%     i_prdct2 = horzcat(i_prdct2, diag(drvGains)\(tau_withoutFriction + tau_nonlnr_frcn));
end

%%
plotTorquesWithLinearFriction = 0;
plotTorquesWithNonlinearFriction = 0;

for i = 1:6
    figure
    hold on
    plot(vldtnTrjctry.t, vldtnTrjctry.i(:,i), 'r-')
    plot(vldtnTrjctry.t, i_prdct1(i,:), 'k-')
    legend('measured', 'prdctd1')
    grid on
end



if plotTorquesWithLinearFriction
    for i = 1:6
        figure
        hold on
        plot(vldtnTrjctry.t, tau_msrd(i,:), 'r-')
        plot(vldtnTrjctry.t, tau_prdctd1(i,:), 'k-')
        plot(vldtnTrjctry.t, tau_prdctd1(i,:) - tau_msrd(i,:), '--')
        legend('measured', 'predicted', 'error')
        grid on
    end
end

if plotTorquesWithNonlinearFriction
    for i = 1:6
        figure
        hold on
        plot(vldtnTrjctry.t, tau_msrd(i,:), 'r-')
        plot(vldtnTrjctry.t, tau_prdctd2(i,:), 'k-')
        plot(vldtnTrjctry.t, tau_prdctd2(i,:) - tau_msrd(i,:), '--')
        legend('measured', 'predicted', 'error')
        grid on
    end
end

return
for i = 1:6
    figure
    hold on
    plot(vldtnTrjctry.t, vldtnTrjctry.i(:,i),'r-')
    plot(vldtnTrjctry.t, i_prdct1(i,:),'k-')
    plot(vldtnTrjctry.t, i_prdct2(i,:),'b-')
    legend('real','with linear friction', 'with nonlinear friction')
    grid on
end

return
fig = figure;
subplot(3,2,1)
    plot(t_msrd,tau_msrd(1,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(1,:),'k-')
    ylabel('\tau_1')
    grid on
subplot(3,2,2)
    plot(t_msrd,tau_msrd(2,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(2,:),'k-')
    ylabel('\tau_2')
    grid on
subplot(3,2,3)
    plot(t_msrd,tau_msrd(3,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(3,:),'k-')
    ylabel('\tau_3')
    grid on
subplot(3,2,4)
    plot(t_msrd,tau_msrd(4,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(4,:),'k-')
    ylabel('\tau_4')
    grid on
subplot(3,2,5)
    plot(t_msrd,tau_msrd(5,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(5,:),'k-')
    ylabel('\tau_5')
    xlabel('t')
    grid on
subplot(3,2,6)
    plot(t_msrd,tau_msrd(6,:),'r-')
    hold on
    plot(t_msrd,tau_prdctd(6,:),'k-')
    ylabel('\tau_6')
    xlabel('t')
    grid on



