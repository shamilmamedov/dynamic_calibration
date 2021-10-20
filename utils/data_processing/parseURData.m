function out = parseURData(file, int_idx, fnl_idx)
% ----------------------------------------------------------------------
% The function loads csv file "file" obtained from UR10E and parses it
% Inputs:
%   file - name of the file
%   int_idx - index which is assumed to be the first one, used to 
%             delete garbage or unneseccary data
%   fnl_idx - index which is assumed to be the final, severs the same
%             purpose as int_idx
% Ouputs:
%   out.t - time
%   out.q - generilized positions of the joints
%   out.qd - generilized velocities of the joints
%   out.i - motor current of each joint motor
%   out.i_des - desired current of each joint motor
%   out.tau_des - desired torque of each joint motor
% ----------------------------------------------------------------------
traj = load(file);

out = struct;
out.t = traj(int_idx:fnl_idx,1) - traj(int_idx,1);
out.q = traj(int_idx:fnl_idx,2:7);
out.qd = traj(int_idx:fnl_idx,8:13);
out.i = traj(int_idx:fnl_idx,14:19);
out.i_des = traj(int_idx:fnl_idx,20:25);
out.tau_des = traj(int_idx:fnl_idx,26:31);
