clc; clear all;

name = 'harmonic_A_0.7854_v_0.5.bag';

bag = rosbag(name);

% extract the desired topics
start_time = bag.StartTime;
end_time = bag.EndTime;

bagselect_current = select(bag, 'Time', [start_time start_time + end_time], 'Topic', '/current_synch' );
bagselect_torque = select(bag, 'Time', [start_time start_time + end_time], 'Topic','/netft_data_synch' );
bagselect_Jstates = select(bag, 'Time', [start_time start_time + end_time], 'Topic','/joint_states_synch' );


% extract message data as time series
ts_cur = timeseries(bagselect_current, 'Data');
ts_trq = timeseries(bagselect_torque,  'Wrench.Torque.Z');


% extract the joint states as time series
msgs = readMessages(bagselect_Jstates);
js_pos = []; 
js_vel = [];
for i=  1:numel(msgs)
    js_pos = [js_pos msgs{i}.Position];
    js_vel = [js_vel msgs{i}.Velocity];
end

ts_js_time = timeseries(bagselect_Jstates,  'Header.Stamp.Sec');
time = ts_js_time.Time;


ts_js_pos_shldr = timeseries(js_pos(1,:)',time);
ts_js_pos_elbw = timeseries(js_pos(2,:)',time);

ts_js_vel_shldr = timeseries(js_vel(1,:)',time);
ts_js_vel_elbw = timeseries(js_vel(2,:)',time);

len = min([ts_js_vel_elbw.Length, ts_js_vel_shldr.Length, ts_js_pos_elbw.Length, ...
            ts_js_pos_shldr.Length,  ts_cur.Length, ts_trq.Length]);

        
% building the table of all the topics
time = ts_cur.Time(1:len);
current = ts_cur.Data(1:len);
torque = ts_trq.Data(1:len);
shldr_position = ts_js_pos_shldr.Data(1:len);
elbw_position = ts_js_pos_elbw.Data(1:len);
shldr_velocity = ts_js_vel_shldr.Data(1:len);
elbw_velocity = ts_js_vel_elbw.Data(1:len);
        
T = table(time, current, torque, shldr_position, elbw_position, shldr_velocity, elbw_velocity);      
data = T.Variables;

save(strcat('planar2DOF/data_pndbt/mat_files/',name(1:end-4),'.mat'),'data');


