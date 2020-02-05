function plnr_visualize(q, plnr)

figure
for j = 1:10:length(q)
    clf
    qj = q(j,:);

    T_0k(:,:,1) = eye(4);
    for i = 1:2
        R_pj = RPY(str2num(plnr.robot.joint{i}.origin.Attributes.rpy));
        p_pj = str2num(plnr.robot.joint{i}.origin.Attributes.xyz)';
        T_pj = [R_pj, p_pj; zeros(1,3), 1];

        R_jk = Rot(qj(i),plnr.k(:,i));
        p_jk = zeros(3,1);
        T_jk = [R_jk, p_jk; zeros(1,3),1];
        T_pk(:,:,i) = T_pj*T_jk;

        T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
        z_0k(:,i) = T_0k(1:3,1:3,i+1)*plnr.k(:,i);
        r_0k(:,i) = [eye(3),zeros(3,1)]*T_0k(:,:,i+1)*[plnr.r_com(:,i);1];
    end
    R_pee = RPY(str2num(plnr.robot.joint{3}.origin.Attributes.rpy));
    p_pee = str2num(plnr.robot.joint{3}.origin.Attributes.xyz)';
    T_pee = [R_pee, p_pee; zeros(1,3), 1];
    T_0ee = T_0k(:,:,3)*T_pee;
    
    MARKER_SIZE = 100; %marker size of the scatter function
    scatter(T_0k(1,4,2), T_0k(3,4,2),MARKER_SIZE,'k','filled')
    hold on
    scatter(T_0k(1,4,3), T_0k(3,4,3),MARKER_SIZE,'k','filled')
    % scatter(Tee0(1,3), Tee0(2,3))
    line([T_0k(1,4,2) T_0k(1,4,3)], [T_0k(3,4,2), T_0k(3,4,3)],'Color','k','LineWidth',1.5)
    line([T_0k(1,4,3) T_0ee(1,4)], [T_0k(3,4,3), T_0ee(3,4)],'Color','k','LineWidth',1.5)
    xlim([-0.5 0.5])
    ylim([-0.6 0.2])

    pause(1e-1)
end
