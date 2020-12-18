classdef Pendubot
   properties
       description
       qr_decomposition
   end
   
   methods
       function obj = Pendubot(path_to_urdf)
           obj.description = parse_urdf(path_to_urdf);
           obj.qr_decomposition = load('qr_decomposition.mat');
       end
       
       generate_dynamics_functions(obj);
       generate_rigid_body_regressor_function(obj);
       get_base_parameters(obj);
       pi_b = get_symbolic_base_parameters(obj);
       animate_motion(obj, q);
       
       function pi = get_dynamic_parameters_from_urdf(obj, type)
           if strcmp(type, 'standard')
               pi = obj.description.pi(:);
           elseif strcmp(type, 'base')
               t1 = [eye(obj.qr_decomposition.no_base_parameters) ...
                        obj.qr_decomposition.beta];
               pi = t1*obj.qr_decomposition.permutation_matrix'*obj.description.pi(:);
           else 
               error('The parameters can be either standard or base');
           end
       end
       
       function Y = get_rigid_body_regressor(obj, q, q_dot, q_2dot, type)
           if strcmp(type, 'standard')
               Y = obj.get_standard_rigid_body_regressor(q, q_dot, q_2dot);
           elseif strcmp(type, 'base')
               Ystnd = obj.get_standard_rigid_body_regressor(q, q_dot, q_2dot);
               Y = Ystnd*obj.qr_decomposition.permutation_matrix(:,...
                                1:obj.qr_decomposition.no_base_parameters);
           else
               error('The parameters can be either standard or base');
           end
       end
       
       function W = get_rigid_body_observation_matrix(obj, q, q_dot, q_2dot, type)
           no_observations = numel(q)/2;
           W = [];
           for k = 1:no_observations
               W = vertcat(W, obj.get_rigid_body_regressor(q(:,k), q_dot(:,k), q_2dot(:,k), type));
           end
       end
       
       function Wf = get_friction_observation_matrix(obj, q_dot, type)
           Wf = [];
           for k = 1:size(q_dot,2)
              Wf = vertcat(Wf, obj.get_friction_regressor(q_dot(:,k), type));
           end
       end
       
       function [pi_rgb_hat, pi_frcn_hat] = identify_parameters(obj, tau, q, q_dot, q_2dot)
           T = obj.get_aggregated_torque_vector(tau);
           Wb = obj.get_rigid_body_observation_matrix(q, q_dot, q_2dot, 'base');
           Wf = obj.get_friction_observation_matrix(q_dot, 'continuous');
           W = [Wb Wf];
           pi_hat = (W'*W)\(W'*T);
           pi_rgb_hat = pi_hat(1:obj.qr_decomposition.no_base_parameters);
           pi_frcn_hat = pi_hat(obj.qr_decomposition.no_base_parameters+1:end);
       end
       
       function q_2dot = get_forward_dynamics(obj, q, q_dot, tau)
           q_2dot = obj.get_M(q)\(obj.get_B()*tau - obj.get_n(q, q_dot)); 
       end
       
       function x_dot = ode(obj, x, u)
           q = x(1:2);
           q_dot = x(3:4);
           q_2dot = obj.get_forward_dynamics(q, q_dot, u);
           x_dot = [q_dot; q_2dot];
       end
       
   end
   
   
   methods (Static)
       Y = get_standard_rigid_body_regressor(q, q_dot, q_2dot);
       
       M = get_M(q);
       n = get_n(q, q_dot);
       C = get_C(q, q_dot);
       g = get_g(q);
       
       function B = get_B()
           B = [1; 0];
       end
       
       function Yf = get_friction_regressor(q_dot, type)
           if strcmp(type, 'continuous')
               beta = 100;
               Yf = [q_dot(1), tanh(beta*q_dot(1)), 0, 0;
                        0, 0, q_dot(2), tanh(beta*q_dot(2))];
           elseif strcmp(type, 'discontinuous')
                Yf = [q_dot(1), sign(q_dot(1)), 0, 0;
                        0, 0, q_dot(2), sign(q_dot(2))];
           else 
               error('Coulomb friction can be either continuous or discontinous!');
           end
       end
       
       function T = get_aggregated_torque_vector(tau)
           no_observations = length(tau);
           T = zeros(2*no_observations,1);
           for k = 1:no_observations
               T(2*k-1) = tau(k);
           end
       end
   end
    
end