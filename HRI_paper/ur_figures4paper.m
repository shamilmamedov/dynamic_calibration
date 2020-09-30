% The script creates figures for the paper "Comparison of obaser??? "
clc; clear all; close all;

idntfcnTrjctry = parseURData('ur-20_02_10-30sec_12harm.csv', 635, 3510);

%%
close all


fig = figure;
fig.Units = 'centimeters';
fig.InnerPosition = [10, 10, 9, 6]; %[left bottom width height]
fig.GraphicsSmoothing = 'on';
ax = gca;
ax.TickLabelInterpreter = 'latex';

plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,1), 'k-')
hold on
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,2), 'r-')
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,3), 'b-')
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,4), 'k--')
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,5), 'r--')
plot(idntfcnTrjctry.t, idntfcnTrjctry.q(:,6), 'b--')
ylabel('$q$, rad', 'interpreter', 'latex')
xlabel('$t$, sec', 'interpreter', 'latex')
legend('$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$',...
        'location','northoutside','NumColumns', 3, 'interpreter', 'latex')
grid on
grid minor

hgexport(fig,'HRI_paper/idnt_trj')