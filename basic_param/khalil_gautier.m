function [base,regressor] = khalil_gautier(lRot,lPos,lZ,q_sym)
% Find base parameters and regressor of a serial manipulator
%   lRot - array of rotations from i-1 to i (3x3xN)
%   lPos - array of translation from i-1 to i (3x1xN)
%   lZ - array of axis orientaions (3x1xN) in form [0;0;1]
%   q_sym - vector of symbolic joints (Nx1)
% Return:
%   base - vector of base parameters
%   regressor - regressor matrix

    % number of joints
    n = size(lRot,3);     
    
    % get parameters
    [PI,qd_sym,qdd_sym] = params(n);
    
    % uncomment to apply parallel axis theorem
    %PI = steiner(PI);
    
    % find energetical parameters    
    f_sym = sym(zeros(10,n));      % f term
    lam_sym = sym(zeros(10,10,n)); % lambdas
    mu_sym = sym(zeros(10,10,n));  % mus
    PI_reg = sym(zeros(10,n));     % regrouped parameters
    
    vtmp = sym(zeros(3,1));        % linear velocity
    wtmp = sym(zeros(3,1));        % rotational velocity
    detmp = sym(zeros(10,1));      % current DE value
    dutmp = sym([0;0;0;0;0;0;0;0;9.81;0]); % current DU value
    for i = 1:n
        % current transformation
        R = sym(lRot(:,:,i));  % from i-1 to i, transpose
        p = sym(lPos(:,:,i));  % from i-1 to i
        z = sym(lZ(:,:,i));    % around i axis
        % velocities
        vtmp = R'*(vtmp + cross(wtmp,p));        
        wtmp = R'*wtmp + z*qd_sym(i);
        % f 
        f_sym(:,i) = getF(vtmp,wtmp,z,qd_sym(i));
        % DE
        lambda = getLambda(R,p);
        detmp = lambda*detmp + qd_sym(i)*f_sym(:,i);
        lam_sym(:,:,i) = lambda;
        % DU
        mu = getMu(R,p);
        dutmp = mu * dutmp;
        mu_sym(:,:,i) = mu;
        
        % remove those who no effect
        ind = no_effect(detmp,dutmp,[q_sym;qd_sym]);
        PI_reg(:,i) = PI(:,i);
        PI_reg(ind,i) = 0;
    end    
    
    % regroup 
    for i = n:-1:2
        [pprev,pcur] = group(PI_reg(:,i-1),PI_reg(:,i),lam_sym(:,:,i),lZ(:,:,i));
        PI_reg(:,i-1) = pprev;
        PI_reg(:,i) = pcur;
    end
    
    % find base parameters and regressor
    base = sym([]);    
    Lagr = sym([]);
    detmp = sym(zeros(10,1));      % current DE value
    dutmp = sym([0;0;0;0;0;0;0;0;9.81;0]); % current DU value
    for i = 1:n
        detmp = lam_sym(:,:,i)*detmp + qd_sym(i)*f_sym(:,i);
        dutmp = mu_sym(:,:,i)*dutmp;
        ind = find(PI_reg(:,i) ~= 0);
        base = [base;PI_reg(ind,i)];        
        Lagr = [Lagr;(detmp(ind)-dutmp(ind))];
    end
    
    % find regressor    
    regressor = sym(0);
    dL_dqd = jacobian(Lagr,qd_sym)';
    dL_qd  = jacobian(Lagr,q_sym)';
    
    for i = 1:n
        regressor = regressor + diff(dL_dqd,qd_sym(i))*qdd_sym(i) + ...
                                diff(dL_dqd,q_sym(i))*qd_sym(i);
    end
    regressor = regressor - dL_qd;
end

function [par,qd,qdd] = params(n)
% Generate symbolic parameters with size n
    % joints    
    qd  = sym('qd', [n 1],'real');
    qdd = sym('qdd',[n 1],'real');
    % inertial    
    Ixx = sym('Ixx',[1 n],'real');
    Ixy = sym('Ixy',[1 n],'real');
    Ixz = sym('Ixz',[1 n],'real');
    Iyy = sym('Iyy',[1 n],'real');
    Iyz = sym('Iyz',[1 n],'real');
    Izz = sym('Izz',[1 n],'real');
    rX  = sym('r_x', [1 n],'real');
    rY  = sym('r_y', [1 n],'real');
    rZ  = sym('r_z', [1 n],'real');
    m   = sym('m',  [1 n],'real');
    par = [Ixx; Ixy; Ixz; Iyy; Iyz; Izz; m .* rX; m .* rY; m .* rZ; m];
end

function v = no_effect(de,du,vars)
    v = [];
    for i = 1:10
        if de(i) == 0 && (du(i) == 0 || ~has(du(i),vars))
            v = [v;i];
        end
    end
end

function par = steiner(pi)
% Apply parallel axis theorem
   par = pi;
   for i = 1:size(pi,2)
       col = pi(:,i);
       prod = vec2skewSymMat(col(7:9))'*vec2skewSymMat(col(7:9)) / col(10);
       par(1,i) = pi(1,i) + prod(1,1);
       par(2,i) = pi(2,i) + prod(1,2);
       par(3,i) = pi(3,i) + prod(1,2);
       par(4,i) = pi(4,1) + prod(2,2);
       par(5,i) = pi(5,i) + prod(2,3);
       par(6,i) = pi(6,i) + prod(3,3);
   end
end