function data = filterData(data)
% ---------------------------------------------------------------------
% The function filters UR10E data for identification purposes.
% It filters generilized velocities and current, as well as estimates
% acceleration based on velocities by means of numerical differentiation
% with central difference scheme. The function also plots filtered data
% against unfileterd for visual inspection of the quality of filtering
% Input:
%   data - structure which has data.t, data.q, data.qd, data.i
% Ouput:
%   data - the same structure but with more elements, that are
%          data.qd_fltrd - filtered velocities, data.i_fltrd - 
%          filtered currents, data.q2d_est - estimated accelerations
% ---------------------------------------------------------------------

% data = parseURData('ur-20_02_05-20sec_8harm.csv', 320, 2310);

% ---------------------------------------------------------------------
% Filtering Velocities
% ---------------------------------------------------------------------
% Design filter
vel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.15,'DesignMethod','butter');

data.qd_fltrd = zeros(size(data.qd));
for i = 1:6
    data.qd_fltrd(:,i) = filtfilt(vel_filt,data.qd(:,i));
end

% ------------------------------------------------------------------------
% Estimating accelerations
% ------------------------------------------------------------------------
% Three point central difference
data.q2d_est = zeros(size(data.qd_fltrd));
for i = 2:length(data.qd_fltrd)-1
   dlta_qd_fltrd =  data.qd_fltrd(i+1,:) -  ...
                        data.qd_fltrd(i-1,:);
   dlta_t_msrd = data.t(i+1) - data.t(i-1);
   data.q2d_est(i,:) = dlta_qd_fltrd/dlta_t_msrd;
end

% Zeros phase filtering acceleration obtained by finite difference
accel_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.15,'DesignMethod','butter');
for i = 1:6
    data.q2d_est(:,i) = filtfilt(accel_filt,data.q2d_est(:,i));
end

% ------------------------------------------------------------------------
% Filtering current
% ------------------------------------------------------------------------
% Zeros phase filtering acceleration obtained by finite difference
curr_filt = designfilt('lowpassiir','FilterOrder',5, ...
        'HalfPowerFrequency',0.20,'DesignMethod','butter');

for i = 1:6
    data.i_fltrd(:,i) = filtfilt(curr_filt,data.i(:,i));
end

% -----------------------------------------------------------------------
% Filtering desired current and desired torque
% -----------------------------------------------------------------------
for i = 1:6
    data.i_des_fltrd(:,i) = filtfilt(curr_filt,data.i_des(:,i));
end

for i = 1:6
    data.tau_des_fltrd(:,i) = filtfilt(curr_filt,data.tau_des(:,i));
end



% Functions for plotting
function plotVelocity(trj)
    figure
    plot(trj.t, trj.qd)
    hold on
    plot(trj.t, trj.qd_fltrd)
end

function plotCurrent(trj)
    figure
    plot(trj.t, trj.i)
    hold on
    plot(trj.t, trj.i_fltrd)
end

function plotAcceleration(trj)
    figure
    plot(trj.t, trj.q2d_est)
end

end