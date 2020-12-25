function visualize_configurations(obj, q)
    % Visualize the set of configurations of the robot
    pd = obj.description; % pendubot description
    
    
    MARKER_SIZE = 50; %marker size of the scatter function
        
    
    fig = figure;
    fig.Units = 'centimeters';
%     fig.InnerPosition = [10, 10, 10, 8]; %[left bottom width height]
%     fig.InnerPosition = [10, 10, 6, 4]; %[left bottom width height]
    fig.GraphicsSmoothing = 'on';
    hold on
    for j = 1:size(q,2)
        qj = q(:,j);
        T_0k(:,:,1) = eye(4);
        for i = 1:2
            R_pj = RPY(str2num(pd.robot.joint{i}.origin.Attributes.rpy));
            p_pj = str2num(pd.robot.joint{i}.origin.Attributes.xyz)';
            T_pj = [R_pj, p_pj; zeros(1,3), 1];

            R_jk = Rot(qj(i),pd.k(:,i));
            p_jk = zeros(3,1);
            T_jk = [R_jk, p_jk; zeros(1,3),1];
            T_pk(:,:,i) = T_pj*T_jk;

            T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
            z_0k(:,i) = T_0k(1:3,1:3,i+1)*pd.k(:,i);
        end
        %  get position of the end-effector
        p_pj = str2num(pd.robot.joint{i+1}.origin.Attributes.xyz)';
        ee_pstn = T_0k(:,:,i+1)*[p_pj;1];
        ee_pstn = ee_pstn(1:3);

        scatter(T_0k(1,4,2), T_0k(3,4,2),MARKER_SIZE,'k','filled')
        hold on
        scatter(T_0k(1,4,3), T_0k(3,4,3),MARKER_SIZE,'k','filled')
        line([T_0k(1,4,2) T_0k(1,4,3)], [T_0k(3,4,2), T_0k(3,4,3)],'Color','k','LineWidth',2)
        line([T_0k(1,4,3) ee_pstn(1)], [T_0k(3,4,3), ee_pstn(3)],'Color','k','LineWidth',2)

        l = 1.1 * (str2num(pd.robot.link{1,2}.visual.geometry.cylinder.Attributes.length) + ...
                str2num(pd.robot.link{1,3}.visual.geometry.cylinder.Attributes.length));
%         xlim([-l  l])
%         ylim([-l  l])
        axis equal
        grid on
    end
end