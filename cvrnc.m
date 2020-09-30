% In this script we try to find covariance matrices for tuning kalman
% filter in order to estimate disturbance (ideally external) torques
clc; clear all; close all;

% load drive gains
load('driveGains.mat');

% load dynamic parameter
prmtrs = load('/home/shamil/Desktop/work/projects/observers/ur10e_dynamic_parameters.mat');

jointTrajectories{1} = parseURData('ur-20_01_22-joint_1.csv', 300, 5650);
jointTrajectories{2} = parseURData('ur-20_01_22-joint_2.csv', 350, 5770);
jointTrajectories{3} = parseURData('ur-20_01_22-joint_3.csv', 370, 5830);
jointTrajectories{4} = parseURData('ur-20_01_22-joint_4.csv', 540, 5980);
jointTrajectories{5} = parseURData('ur-20_01_22-joint_5.csv', 450, 5890);
jointTrajectories{6} = parseURData('ur-20_01_22-joint_6.csv', 650, 6090);

for i = 1:6
    jointTrajectories{i} = filterData(jointTrajectories{i});
end

%% Covariance matrix of the torque measurement
for i = 1:6
    pd_tau{i} = fitdist(jointTrajectories{i}.i(:,i)*drvGains(i) - ...
                       jointTrajectories{i}.i_fltrd(:,i)*drvGains(i), 'Normal');
end


% create data based on probability distrubtion found from data
for i = 1:6
   x{i} = linspace(-3*pd_tau{i}.sigma, 3*pd_tau{i}.sigma, 100);
   y{i} = normpdf(x{i}, pd_tau{i}.mu, pd_tau{i}.sigma);
end

% for no_joint = 1:6
%    figure 
%    plot(jointTrajectories{no_joint}.t, jointTrajectories{no_joint}.i(:,no_joint)*drvGains(no_joint))
%    hold on
%    plot(jointTrajectories{no_joint}.t, jointTrajectories{no_joint}.i_fltrd(:,no_joint)*drvGains(no_joint))
%    grid on 
% end

figure
for i = 1:6
    subplot(3,2,i)
    histogram(jointTrajectories{i}.i(:,i)*drvGains(i) - ...
              jointTrajectories{i}.i_fltrd(:,i)*drvGains(i),...
              'Normalization', 'pdf')
    hold on
    plot(x{i}, y{i})
    title(num2str(i))
end

tau_std = zeros(6,1);
for i = 1:6
    tau_std(i) = pd_tau{i}.sigma;
end
return
%% Covariance matrix of the joint velocity
% fit probability distribution to data; to the difference between the real
% joint velocity measurements and the filtered one
% pd_qd1 = fitdist(jointTrajectories{1}.qd(:,1) - jointTrajectories{1}.qd_fltrd(:,1), 'Normal');
for i = 1:6
    pd_qd{i} = fitdist(jointTrajectories{i}.qd(:,i) - ...
                       jointTrajectories{i}.qd_fltrd(:,i), 'Normal');
end

% create data based on probability distrubtion found from data
for i = 1:6
   x{i} = linspace(-3*pd_qd{i}.sigma, 3*pd_qd{i}.sigma, 100);
   y{i} = normpdf(x{i}, pd_qd{i}.mu, pd_qd{i}.sigma);
end

% for i = 1:6
% figure
% plot(jointTrajectories{i}.t, jointTrajectories{i}.qd(:,i) - ...
%                              jointTrajectories{i}.qd_fltrd(:,i))
% end


figure
for i = 1:6
    subplot(3,2,i)
    histogram(jointTrajectories{i}.qd(:,i) - jointTrajectories{i}.qd_fltrd(:,i),...
              'Normalization', 'pdf')
    hold on
    plot(x{i}, y{i})
    title(num2str(i))
end


%% Covariance matrix of the torque prediction error
for j = 1:6
    tau_hat{j} = zeros(size(jointTrajectories{j}.t));
    for i = 1:length(jointTrajectories{j}.t)
       qi = jointTrajectories{j}.q(i,:)';
       qdi = jointTrajectories{j}.qd(i,:)';
       q2di = jointTrajectories{j}.q2d_est(i,:)';
       Mi = M_mtrx_fcn_mex(qi, prmtrs.pi_rgd) + diag(prmtrs.pi_drvs);
       Ci = C_mtrx_fcn_mex(qi, qdi, prmtrs.pi_rgd);
       Gi = G_vctr_fcn_mex(qi, prmtrs.pi_rgd);
       Fi = F_vctr_fcn_mex(qdi,prmtrs.pi_frcn);
       taui = Mi*q2di + Ci*qdi + Gi + Fi;
       tau_hat{j}(i) = taui(j);
    end
end

%%
for i = 2:2
figure
plot(jointTrajectories{i}.t, jointTrajectories{i}.i(:,i)*drvGains(i))
hold on
plot(jointTrajectories{i}.t, tau_hat{i})
% plot(jointTrajectories{i}.t, jointTrajectories{i}.i(:,1)*drvGains(i) - tau_hat{i})
end
return

for i = 1:6
    pd_tau{i} = fitdist(jointTrajectories{i}.i(:,i)*drvGains(i) - tau_hat{i}, 'Normal');
end

% create data based on probability distrubtion found from data
for i = 1:6
    x{i} = linspace(-3*pd_tau{i}.sigma, 3*pd_tau{i}.sigma, 100);
    y{i} = normpdf(x{i}, pd_tau{i}.mu, pd_tau{i}.sigma);
end


figure
for i = 1:6
    subplot(3,2,i)
    histogram(jointTrajectories{i}.i(:,i)*drvGains(i) - tau_hat{i},...
              'Normalization', 'pdf')
    hold on
    plot(x{i}, y{i})
    title(num2str(i))
end


