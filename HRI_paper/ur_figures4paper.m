% The script creates figures for the paper "Comparison of obaser??? "
clc; clear all; close all;

idntfcnTrjctry = parseURData('ur-20_02_10-30sec_12harm.csv', 635, 3510);

%%
close all


fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 15, 6]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
ax = gca;
ax.TickLabelInterpreter = 'latex';

plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,1), 'k-', 'LineWidth', 1.25)
hold on
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,2), 'r-', 'LineWidth', 1.25)
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,3), 'b-', 'LineWidth', 1.25)
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,4), 'k-.', 'LineWidth', 1.25)
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,5), 'r-.', 'LineWidth', 1.25)
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,6), 'b-.', 'LineWidth', 1.25)
ylabel('$q$, rad', 'interpreter', 'latex', 'Fontname', 'Times', 'FontSize', 9)
xlabel('$t$, sec', 'interpreter', 'latex', 'Fontname', 'Times', 'FontSize', 9)
legend('$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$',...
        'location','southeast' ,'NumColumns', 2, 'interpreter', 'latex',...
        'Fontname', 'Times', 'FontSize', 9)
xlim([0 32])
    
ax = gca;
ax.FontUnits = 'points';
ax.FontWeight = 'normal';
ax.FontSize = 9;
ax.FontName = 'Times';
ax.TickLabelInterpreter = 'latex';
box off
    
grid on
grid minor

print -djpeg HRI_paper/idnt_trj.jpeg

% hgexport(fig,'HRI_paper/idnt_trj')