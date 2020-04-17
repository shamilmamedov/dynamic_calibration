% ------------------------------------------------------------------------
% Load validation trajectory
% ------------------------------------------------------------------------
close all;
staticValidation = 0;

if ~staticValidation
%     vldtnTrjctry = parseURData('ur-20_01_17-ptp_10_points.csv', 1, 5346);
    vldtnTrjctry = parseURData('ur-19_12_23_free.csv', 1, 2005);
    vldtnTrjctry = filterData(vldtnTrjctry);
else
    load('vldtnTrjctrySttcs.mat');
    vldtnTrjctry = vldtnTrjctrySttcs;
end

% -----------------------------------------------------------------------
% Predicting torques
% -----------------------------------------------------------------------
%Constracting regressor matrix
tau_msrd = []; 
i_OLS = {}; i_SDP = {};
tau_OLS = {}; tau_SDP = {};
for j = 1:length(idntfcnTrjctry)
    i_OLS{j} = [];
    i_SDP{j} = [];
    tau_SDP{j} = [];
    tau_OLS{j} = [];
end

t1 = reshape(pi_full, [11,6]);
pi_rgd = reshape(t1(1:10,:), [60,1]);
pi_drvs = t1(11,:)';
Tau1 = []; Tau2 = [];
for i = 1:length(vldtnTrjctry.t)
    qi = vldtnTrjctry.q(i,:)';
    qdi = vldtnTrjctry.qd_fltrd(i,:)';
    q2di = vldtnTrjctry.q2d_est(i,:)';
    Yi = regressorWithMotorDynamics(qi, qdi, q2di);
    
    Ybi = Yi*baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);
    Yfrctni = frictionRegressor(qdi);
    
    Tau1 = [Tau1, (M_mtrx_fcn(qi, pi_rgd) + diag(pi_drvs))*q2di + ...
                  C_mtrx_fcn(qi, qdi, pi_rgd)*qdi + ...
                  G_vctr_fcn(qi, pi_rgd) + F_vctr_fcn(qdi, pifrctn_SDP(:,1))];
    
    tau_msrd = horzcat(tau_msrd, diag(drvGains)*vldtnTrjctry.i(i,:)');
    
    for j = 1:length(idntfcnTrjctry)
        i_OLS{j} = horzcat(i_OLS{j}, diag(drvGains)\([Ybi Yfrctni]*[pib_OLS(:,j); pifrctn_OLS(:,j)]));
        i_SDP{j} = horzcat(i_SDP{j}, diag(drvGains)\([Ybi Yfrctni]*[pib_SDP(:,j); pifrctn_SDP(:,j)]));
        tau_SDP{j} = horzcat(tau_SDP{j}, [Ybi Yfrctni]*[pib_SDP(:,j); pifrctn_SDP(:,j)]);
        tau_OLS{j} = horzcat(tau_OLS{j}, [Ybi Yfrctni]*[pib_OLS(:,j); pifrctn_OLS(:,j)]);
    end
end


%%
% clrs = {'r', 'b', 'm', 'y'};
% 
% for i = 1:6
%     figure
%     hold on
%     plot(vldtnTrjctry.t, vldtnTrjctry.i(:,i), 'k-')
%     for j = 1:length(idntfcnTrjctry)
%         plot(vldtnTrjctry.t, i_SDP{j}(i,:), clrs{j}, 'LineWidth',1.5)
%     end
%     ylabel('\tau, Nm')
%     xlabel('t, sec')
%     grid on
% end

%%
close all


dlta_tau_1 = tau_msrd(1,:) - tau_SDP{1}(1,:);
dlta_tau_2 = tau_msrd(2,:) - tau_SDP{1}(2,:);
dlta_tau_3 = tau_msrd(3,:) - tau_SDP{1}(3,:);
dlta_tau_4 = tau_msrd(4,:) - tau_SDP{1}(4,:);
figure
histogram(dlta_tau_3,'Normalization','pdf')


%% Figure for paper
close all


fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 15, 5]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
subplot(1,2,1)
ax = gca;
ax.TickLabelInterpreter = 'latex';
plot(vldtnTrjctry.t, tau_msrd(2,:), 'k-')
hold on
plot(vldtnTrjctry.t, tau_SDP{1}(2,:), 'r', 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_msrd(3,:), 'b-')
plot(vldtnTrjctry.t, tau_SDP{1}(3,:), 'm-', 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_msrd(1,:))
plot(vldtnTrjctry.t, tau_SDP{1}(1,:), 'LineWidth',1.5)
xlim([0 20])
xlabel('$t$, sec', 'interpreter', 'latex')
ylabel('$\tau_{1-3}$, Nm', 'interpreter', 'latex')
grid minor


% fig = figure;
% fig.Units = 'centimeters';
% fig.InnerPosition = [10, 10, 9, 5]; %[left bottom width height]
% fig.GraphicsSmoothing = 'on';
subplot(1,2,2)
ax = gca;
ax.TickLabelInterpreter = 'latex';
plot(vldtnTrjctry.t, tau_msrd(4,:), 'k-')
hold on
plot(vldtnTrjctry.t, tau_SDP{1}(4,:), 'r', 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_msrd(5,:), 'b-')
plot(vldtnTrjctry.t, tau_SDP{1}(5,:), 'm-', 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_msrd(6,:))
plot(vldtnTrjctry.t, tau_SDP{1}(6,:), 'LineWidth',1.5)
xlim([0 20])
xlabel('$t$, sec', 'interpreter', 'latex')
grid minor

hgexport(fig,'HRI_paper/vldtn')

return
figure
plot(vldtnTrjctry.t, tau_msrd(2,:), 'k-')
hold on
plot(vldtnTrjctry.t, tau_msrd(3,:), 'k-')
plot(vldtnTrjctry.t, tau_SDP{1}(2,:), clrs{1}, 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_SDP{1}(3,:), clrs{1}, 'LineWidth',1.5)

figure
plot(vldtnTrjctry.t, tau_msrd(1,:), 'k-')
hold on
plot(vldtnTrjctry.t, tau_msrd(4,:), 'k-')
plot(vldtnTrjctry.t, tau_SDP{1}(1,:), clrs{1}, 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_SDP{1}(4,:), clrs{1}, 'LineWidth',1.5)

figure
plot(vldtnTrjctry.t, tau_msrd(5,:), 'k-')
hold on
plot(vldtnTrjctry.t, tau_msrd(6,:), 'k-')
plot(vldtnTrjctry.t, tau_SDP{1}(5,:), clrs{1}, 'LineWidth',1.5)
plot(vldtnTrjctry.t, tau_SDP{1}(6,:), clrs{1}, 'LineWidth',1.5)

return
for i = 1:3
    figure
    plot(vldtnTrjctry.t, tau_msrd(i,:), 'k-')
    hold on
    for j = 1:length(idntfcnTrjctry)
        plot(vldtnTrjctry.t, tau_SDP{j}(i,:), clrs{j}, 'LineWidth',1.5)
    end
    ylabel('\tau, Nm')
    xlabel('t, sec')
    grid on
end





