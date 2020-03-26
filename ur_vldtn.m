% ------------------------------------------------------------------------
% Load validation trajectory
% ------------------------------------------------------------------------
close all;
staticValidation = 0;

if ~staticValidation
    vldtnTrjctry = parseURData('ur-20_01_17-ptp_10_points.csv', 1, 5346);
%     vldtnTrjctry = parseURData('ur-19_12_23_free.csv', 1, 2005);
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
clrs = {'r', 'b', 'm', 'y'};

for i = 1:6
    figure
    hold on
    plot(vldtnTrjctry.t, vldtnTrjctry.i(:,i), 'k-')
    for j = 1:length(idntfcnTrjctry)
        plot(vldtnTrjctry.t, i_SDP{j}(i,:), clrs{j}, 'LineWidth',1.5)
    end
    ylabel('\tau, Nm')
    xlabel('t, sec')
    grid on
end

%% 
% no_joint = 4;
% figure
% plot(Tau1(no_joint,:), 'r')
% hold on
% plot(tau_SDP{1}(no_joint,:), 'b')
% grid on





