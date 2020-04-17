function [Tau, Wb, W] = computeObservationMatricesForPendubot(pendubot, fullRegressor2BaseRegressor)
    % compose observation matrix and torque vector
    noObservations = length(pendubot.time);
    W = []; Wb = []; Tau = [];
    for i = 1:1:noObservations
        % generalized positions velocities and accelerations
        qi = [pendubot.shldr_position_filtered(i), pendubot.elbw_position_filtered(i)]';
        qdi = [pendubot.shldr_velocity_filtered(i), pendubot.elbw_velocity_filtered(i)]';
        q2di = [pendubot.shldr_acceleration_filtered(i), pendubot.elbow_acceleration_filtered(i)]';

        % regressor
        Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);

        % get base regressor from full regressor
        Ybi = Yi*fullRegressor2BaseRegressor;

        % friction regressor
        Yfrctni = frictionRegressor(qdi);

        % compose base observation matrix
        Wb = vertcat(Wb, [Ybi, Yfrctni]);

        % compose full observation matrix
        W = vertcat(W, [Yi, Yfrctni]);

        % compose torque vector
        taui = [pendubot.torque_filtered(i), 0]';
    %     taui = [pendubot.current(i)*0.123, 0]';
        Tau = vertcat(Tau, taui);
    end
end