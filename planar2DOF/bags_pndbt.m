clc; clear all;

% name = 'position_A_0.7_v_0.5.bag';
% name = 'position_A_1.2_v_0.05.bag';
% name = 'position_A_1.1_v_0.01.bag';
% name = 'position_A_0.5_v_1.bag';
% name = 'ramp_A_1.2_v_1.bag';

bag = rosbag(name);
end_time = 20;

% extract the desired topics
start = bag.StartTime;
bagselect_current = select(bag, 'Time', [start start + end_time], 'Topic', '/current_synch' );
bagselect_torque = select(bag, 'Time', [start start + end_time], 'Topic','/netft_data_synch' );
bagselect_command = select(bag, 'Time', [start start + end_time], 'Topic', '/command_synch' );
bagselect_Jstates = select(bag, 'Time', [start start + end_time], 'Topic','/joint_states_synch' );
bagselect_position = select(bag, 'Time', [start start + end_time], 'Topic', '/position_synch' );
bagselect_velocity = select(bag, 'Time', [start start + end_time], 'Topic','/velocity_synch' );

%
% msgs = readMessages(bagselect, [1 2]);
% extract message data as time series
ts_cur = timeseries(bagselect_current, 'Data');
ts_trq = timeseries(bagselect_torque,  'Wrench.Torque.Z');
ts_cmd = timeseries(bagselect_command, 'Data');
ts_pos = timeseries(bagselect_position, 'Data');
ts_vel = timeseries(bagselect_velocity,  'Data');

% extract the joint states as time series
msgs = readMessages(bagselect_Jstates);
js_pos = [];
for i=  1:numel(msgs)
    js_pos = [js_pos msgs{i}.Position];
end

js_vel = [];
for i=  1:numel(msgs)
    js_vel = [js_vel msgs{i}.Velocity];
end

ts_Js_time = timeseries(bagselect_Jstates,  'Header.Stamp.Sec');
time = ts_Js_time.Time;
%
ts_Js_pos_shldr = timeseries(js_pos(1,:)',time);
ts_Js_pos_elbw = timeseries(js_pos(2,:)',time);

ts_Js_vel_shldr = timeseries(js_vel(1,:)',time);
ts_Js_vel_elbw = timeseries(js_vel(2,:)',time);

len = min([ts_Js_vel_elbw.Length, ts_Js_vel_shldr.Length, ts_Js_pos_elbw.Length, ...
            ts_Js_pos_shldr.Length,  ts_cur.Length, ts_trq.Length, ts_cmd.Length, ts_pos.Length, ts_vel.Length]);

% building the table of all the topics
time = ts_cur.Time(1:len);
current = ts_cur.Data(1:len);
torque = ts_trq.Data(1:len);
command = ts_cmd.Data(1:len);    
position = ts_pos.Data(1:len);
velocity = ts_vel.Data(1:len);
shldr_position = ts_Js_pos_shldr.Data(1:len);
elbw_position = ts_Js_pos_elbw.Data(1:len);
shldr_velocity = ts_Js_vel_shldr.Data(1:len);
elbw_velocity = ts_Js_vel_elbw.Data(1:len);
        
T = table(time, current, torque, command, position, velocity, shldr_position, elbw_position, shldr_velocity, elbw_velocity);      
data = T.Variables;


save(strcat('planar2DOF/data_pndbt/mat_files/',name(1:end-4),'.mat'),'data');


