% ------------------------------------------------------------------------
% Load validation trajectory
% ------------------------------------------------------------------------

vldtnTrjctry = parseURData('ur-19_10_01-14_04_13.csv', 1, 759);
% vldtnTrjctry = parseURData('ur-19_10_01-13_51_41.csv', 1, 660);
% vldtnTrjctry = parseURData('ur-19_09_27-11_28_22.csv', 1, 594);
% vldtnTrjctry = parseURData('ur-20_01_17-p8.csv', 1, 250);
% vldtnTrjctry = parseURData('ur-20_01_17-ptp_10_points.csv', 1, 5346);
% vldtnTrjctry = parseURData('ur-19_12_23_free.csv', 1, 2005);
vldtnTrjctry = filterData(vldtnTrjctry);


% -----------------------------------------------------------------------
% Predicting torques
% -----------------------------------------------------------------------
%Constracting regressor matrix
tau_msrd = []; tau_OLS = []; tau_SDP = [];
i_OLS1 = []; i_OLS2 = []; i_OLS3 = []; 
i_SPD1 = []; i_SPD2 = []; i_SPD3 = [];
for i = 1:length(vldtnTrjctry.t)
    Yi = regressorWithMotorDynamics(vldtnTrjctry.q(i,:)',...
                                    vldtnTrjctry.qd_fltrd(i,:)',...
                                    vldtnTrjctry.q2d_est(i,:)');
    
    Ybi = Yi*baseQR.permutationMatrix(:,1:baseQR.numberOfBaseParameters);
    Yfrctni = frictionRegressor(vldtnTrjctry.qd_fltrd(i,:)');
    
%     tau_msrd = horzcat(tau_msrd, diag(drvGains)*vldtnTrjctry.i(i,:)');
%     tau_OLS = horzcat(tau_OLS, [Ybi Yfrctni]*[pib_OLS; pifrctn_OLS]);
%     tau_SDP = horzcat(tau_SDP, [Ybi Yfrctni]*[pib_SDP; pifrctn_SDP]);
    
    i_OLS1 = horzcat(i_OLS1, diag(drvGains)\([Ybi Yfrctni]*[pib_OLS(:,1); pifrctn_OLS(:,1)]));
    i_OLS2 = horzcat(i_OLS2, diag(drvGains2)\([Ybi Yfrctni]*[pib_OLS(:,2); pifrctn_OLS(:,2)]));
    i_OLS3 = horzcat(i_OLS3, diag(drvGains3)\([Ybi Yfrctni]*[pib_OLS(:,3); pifrctn_OLS(:,3)]));
    
    i_SPD1 = horzcat(i_SPD1, diag(drvGains)\([Ybi Yfrctni]*[pib_SDP(:,1); pifrctn_SDP(:,1)]));
    i_SPD2 = horzcat(i_SPD2, diag(drvGains2)\([Ybi Yfrctni]*[pib_SDP(:,2); pifrctn_SDP(:,2)]));
    i_SPD3 = horzcat(i_SPD3, diag(drvGains3)\([Ybi Yfrctni]*[pib_SDP(:,3); pifrctn_SDP(:,3)]));
end

%%

for i = 1:6
    figure
    hold on
    plot(vldtnTrjctry.t, vldtnTrjctry.i(:,i), 'r-')
    plot(vldtnTrjctry.t, i_OLS1(i,:), 'k-', 'LineWidth',1.5)
    plot(vldtnTrjctry.t, i_OLS2(i,:), 'k--', 'LineWidth',1.5)
    plot(vldtnTrjctry.t, i_OLS3(i,:), 'k-.', 'LineWidth',1.5)
    plot(vldtnTrjctry.t, i_SPD1(i,:), 'b-', 'LineWidth',1.5)
    plot(vldtnTrjctry.t, i_SPD2(i,:), 'b--', 'LineWidth',1.5)
    plot(vldtnTrjctry.t, i_SPD3(i,:), 'b-.', 'LineWidth',1.5)
    legend('measured', 'OLS1', 'OLS2', 'OLS3', 'SDP1', 'SDP2', 'SDP3')
    grid on
end







