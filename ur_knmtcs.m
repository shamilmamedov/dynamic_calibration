% Get robot description
run('main_ur.m')

%{
q_sym = sym('q%d',[6,1], 'real');

T_pk = sym(zeros(4,4,6)); % transformation between links
T_0k = sym(zeros(4,4,7)); T_0k(:,:,1) = sym(eye(4));

for i = 1:6
    jnt_axs_k = str2num(ur10.robot.joint{i}.axis.Attributes.xyz)';
% Transformation from parent link frame p to current joint frame
    rpy_k = sym(str2num(ur10.robot.joint{i}.origin.Attributes.rpy));
    R_pj = RPY(rpy_k);
    R_pj(abs(R_pj)<sqrt(eps)) = sym(0); % to avoid numerical errors
    p_pj = str2num(ur10.robot.joint{i}.origin.Attributes.xyz)';
    T_pj = sym([R_pj, p_pj; zeros(1,3), 1]); % to avoid numerical errors
% Tranformation from joint frame of the joint that rotaties body k to
% link frame. The transformation is pure rotation
    R_jk = Rot(q_sym(i),sym(jnt_axs_k));
    p_jk = sym(zeros(3,1));
    T_jk = [R_jk, p_jk; sym(zeros(1,3)),sym(1)];
% Transformation from parent link frame p to current link frame k
    T_pk(:,:,i) = T_pj*T_jk;
    T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
end
pos = T_0k(1:3,4,7);

matlabFunction(pos,'File','autogen/position_fk_UR10E','Vars',{q_sym})
%}

% load('ptrnSrch_N6T20QR.mat');
% load('ptrnSrch_N8T20QR.mat');
% load('ptrnSrch_N10T20QR.mat');
% load('ptrnSrch_N12T20QR.mat');
load('ga_N5T20.mat');
traj_par.t_smp = 5e-2;
traj_par.t = 0:traj_par.t_smp:traj_par.T;
[q,qd,q2d] = mixed_traj(traj_par.t, c_pol, a, b, traj_par.wf, traj_par.N);


%%
for i = 1:size(q,2)
%     pos(:,i) = ur_fk(q(:,i), ur10);
    pos(:,i) = position_fk_UR10E(q(:,i));
end

% Needed for surface
[x, y] = meshgrid(-0.4:0.05:0.4); % Generate x and y data
z = zeros(size(x, 1)); % Generate z data

figure
hold on
surf(x, y, z) % Plot the surface
for i = 1:size(q,2)
   scatter3(pos(1,i), pos(2,i), pos(3,i)) 
end
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal
view(3)

%% write into file
writeTrajectoryCoefficientsIntoFile(a, b, c_pol)

%%
% figure1 = figure;
% axes1 = axes('Parent',figure1);
% hold(axes1,'on');
% for i = 1:6
%     scatter3(T_0k(1,4,i), T_0k(2,4,i), T_0k(3,4,i))
%     line([T_0k(1,4,i) T_0k(1,4,i+1)], [T_0k(2,4,i) T_0k(2,4,i+1)], ...
%          [T_0k(3,4,i) T_0k(3,4,i+1)])
% end
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis equal



