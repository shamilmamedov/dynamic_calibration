close all

marker_area = 75;

R01 = RPY(str2num(ur10.robot.joint{1}.origin.Attributes.rpy));
p01 = str2num(ur10.robot.joint{1}.origin.Attributes.xyz)';
T01 = [R01, p01; zeros(1,3),1];

R12 = RPY(str2num(ur10.robot.joint{2}.origin.Attributes.rpy));
p12 = str2num(ur10.robot.joint{2}.origin.Attributes.xyz)';
T12 = [R12, p12; zeros(1,3),1];
T02 = T01*T12;

R23 = RPY(str2num(ur10.robot.joint{3}.origin.Attributes.rpy));
p23 = str2num(ur10.robot.joint{3}.origin.Attributes.xyz)';
T23 = [R23, p23; zeros(1,3),1];
T03 = T02*T23;

R34 = RPY(str2num(ur10.robot.joint{4}.origin.Attributes.rpy));
p34 = str2num(ur10.robot.joint{4}.origin.Attributes.xyz)';
T34 = [R34, p34; zeros(1,3),1];
T04 = T03*T34;

R45 = RPY(str2num(ur10.robot.joint{5}.origin.Attributes.rpy));
p45 = str2num(ur10.robot.joint{5}.origin.Attributes.xyz)';
T45 = [R45, p45; zeros(1,3),1];
T05 = T04*T45;

R56 = RPY(str2num(ur10.robot.joint{6}.origin.Attributes.rpy));
p56 = str2num(ur10.robot.joint{6}.origin.Attributes.xyz)';
T56 = [R56, p56; zeros(1,3),1];
T06 = T05*T56;

figure
hold on
scatter3(T01(1,4),T01(2,4),T01(3,4),marker_area,'filled')
scatter3(T02(1,4),T02(2,4),T02(3,4),marker_area,'filled')
scatter3(T03(1,4),T03(2,4),T03(3,4),marker_area,'filled')
scatter3(T04(1,4),T04(2,4),T04(3,4),marker_area,'filled')
scatter3(T05(1,4),T05(2,4),T05(3,4),marker_area,'filled')
scatter3(T06(1,4),T06(2,4),T06(3,4),marker_area,'filled')

line([0,T01(1,4)],[0,T01(2,4)],[0,T01(3,4)],'LineWidth',2)
line([T01(1,4),T02(1,4)],[T01(2,4),T02(2,4)],[T01(3,4),T02(3,4)],'LineWidth',2)
line([T02(1,4),T03(1,4)],[T02(2,4),T03(2,4)],[T02(3,4),T03(3,4)],'LineWidth',2)
line([T03(1,4),T04(1,4)],[T03(2,4),T04(2,4)],[T03(3,4),T04(3,4)],'LineWidth',2)
line([T04(1,4),T05(1,4)],[T04(2,4),T05(2,4)],[T04(3,4),T05(3,4)],'LineWidth',2)
line([T05(1,4),T06(1,4)],[T05(2,4),T06(2,4)],[T05(3,4),T06(3,4)],'LineWidth',2)

xlabel('X, m','interpreter','latex')
ylabel('Y, m','interpreter','latex')
zlabel('Z, m','interpreter','latex')

axis equal