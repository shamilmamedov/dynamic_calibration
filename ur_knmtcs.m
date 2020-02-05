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
load('ptrnSrch_N12T20QR.mat');
traj_par.t_smp = 5e-2;
traj_par.t = 0:traj_par.t_smp:traj_par.T;
[q,qd,q2d] = mixed_traj(traj_par.t, c_pol, a, b, traj_par.wf, traj_par.N);


%%
for i = 1:size(q,2)
%     pos(:,i) = ur_fk(q(:,i), ur10);
    pos(:,i) = position_fk_UR10E(q(:,i));
end

% Needed for surface
[x, y] = meshgrid(-0.7:0.05:0.7); % Generate x and y data
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


function writeTrajectoryCoefficientsIntoFile(a, b, c_pol)
prcsn = 10;
a_coefs = {};
b_coefs = {};
a_coefs{1} = 'a1 = ['; a_coefs{2} = 'a2 = ['; a_coefs{3} = 'a3 = ['; 
a_coefs{4} = 'a4 = ['; a_coefs{5} = 'a5 = ['; a_coefs{6} = 'a6 = [';
b_coefs{1} = 'b1 = ['; b_coefs{2} = 'b2 = ['; b_coefs{3} = 'b3 = ['; 
b_coefs{4} = 'b4 = ['; b_coefs{5} = 'b5 = ['; b_coefs{6} = 'b6 = [';
for i = 1:size(a,2)
    if i < size(a,2)
        for j = 1:6
            a_coefs{j} = strcat(a_coefs{j}, num2str(a(j,i), prcsn), ',');
            b_coefs{j} = strcat(b_coefs{j}, num2str(b(j,i), prcsn), ',');
        end 
    elseif i == size(a,2)
        for j = 1:6
            a_coefs{j} = strcat(a_coefs{j}, num2str(a(j,i), prcsn), ']\n');
            b_coefs{j} = strcat(b_coefs{j}, num2str(b(j,i), prcsn), ']\n');
        end
    end
end

c_coefs = {};
c_coefs{1} = 'c1 = ['; c_coefs{2} = 'c2 = ['; c_coefs{3} = 'c3 = ['; 
c_coefs{4} = 'c4 = ['; c_coefs{5} = 'c5 = ['; c_coefs{6} = 'c6 = [';
for i = 1:size(c_pol,2)
    if i < size(c_pol,2)
        for j = 1:6
            c_coefs{j} = strcat(c_coefs{j}, num2str(c_pol(j,i), prcsn+2), ',');
        end
    elseif i == size(c_pol,2)
        for j = 1:6
            c_coefs{j} = strcat(c_coefs{j}, num2str(c_pol(j,i), prcsn+2), ']\n');
        end
    end
end

fileID_a = fopen('trajectory_optmzn/coeffs4_UR/a_coeffs.script','w');
fileID_b = fopen('trajectory_optmzn/coeffs4_UR/b_coeffs.script','w');
fileID_c = fopen('trajectory_optmzn/coeffs4_UR/c_coeffs.script','w');
for i = 1:6
    fprintf(fileID_a, a_coefs{i});
    fprintf(fileID_b, b_coefs{i});
    fprintf(fileID_c, c_coefs{i});
end
fclose(fileID_a);
fclose(fileID_b);
fclose(fileID_c);

end
