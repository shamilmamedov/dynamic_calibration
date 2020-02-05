% run data processing script
run('pndbt_data_processing.m')

% load mapping from standard parameters to base parameters
load('plnrBaseQR.mat')

% compose observation matrix and torque vector
noObservations = length(pendubot.time);
W = []; Tau = [];
for i = 1:noObservations
    qi = [pendubot.shldr_position(i), pendubot.elbw_position(i)]';
    qdi = [pendubot.shldr_velocity_filtered(i), pendubot.elbw_velocity_filtered(i)]';
    q2di = [pendubot.shldr_acceleration_filtered(i), pendubot.elbow_acceleration_filtered(i)]';
    
    if plnrBaseQR.motorDynamicsIncluded
        Yi = regressorWithMotorDynamicsPndbt(qi, qdi, q2di);
    else
        Yi = full_regressor_plnr(qi, qdi, q2di);
    end
    Ybi = Yi*plnrBaseQR.permutationMatrix(:,1:plnrBaseQR.numberOfBaseParameters);
    Yfrctni = frictionRegressor(qdi);
    W = vertcat(W, [Ybi, Yfrctni]);
    
    taui = [pendubot.torque(i), 0]';
    Tau = vertcat(Tau, taui);
end


%% Usual Least Squares Approach
pi_hat = (W'*W)\(W'*Tau)


%% Set-up SDP optimization procedure
physicalConsistency = 1;

pi_frctn = sdpvar(6,1);
pi_b = sdpvar(plnrBaseQR.numberOfBaseParameters,1); % variables for base paramters
pi_d = sdpvar(15,1); % variables for dependent paramters

% Bijective mapping from [pi_b; pi_d] to standard parameters pi
pii = plnrBaseQR.permutationMatrix*[eye(plnrBaseQR.numberOfBaseParameters), ...
                                    -plnrBaseQR.beta; ...
                                    zeros(15,plnrBaseQR.numberOfBaseParameters), ... 
                                    eye(15)]*[pi_b; pi_d];

cnstr = [];
if physicalConsistency
    for i = 1:11:21
        link_inertia_i = [pii(i),   pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];          
        frst_mmnt_i = pii(i+6:i+8);

        Di = [0.5*trace(link_inertia_i)*eye(3) - link_inertia_i, ...
                frst_mmnt_i; frst_mmnt_i', pii(i+9)];

        cnstr = [cnstr, Di>0];
    end
else
    for i = 1:11:21
        link_inertia_i = [pii(i), pii(i+1), pii(i+2); ...
                          pii(i+1), pii(i+3), pii(i+4); ...
                          pii(i+2), pii(i+4), pii(i+5)];

        frst_mmnt_i = vec2skewSymMat(pii(i+6:i+8));

        Di = [link_inertia_i, frst_mmnt_i'; frst_mmnt_i, pii(i+9)*eye(3)];
        cnstr = [cnstr, Di>0];
    end
end
cnstr = [cnstr, pii(11) > 0]; % first motor inertia constraint

% Feasibility constraints on the friction prameters 
for i = 1:2
   cnstr = [cnstr, pi_frctn(3*i-2)>0, pi_frctn(3*i-1)>0];  
end

% Defining pbjective function
obj = norm(Tau - W*[pi_b; pi_frctn], 2);

% Solving sdp problem
sol2 = optimize(cnstr, obj, sdpsettings('solver','sdpt3'));

pi_b = value(pi_b) % variables for base paramters
pi_frctn = value(pi_frctn)



