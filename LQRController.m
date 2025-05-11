classdef LQRController < handle
    % LQRController: Implementation of the Linear Quadratic Regulator controller
    
    properties
        K   % Feedback gain matrix
    end
    
    methods
        function obj = LQRController(K)
            % Constructor
            obj.K = K;
        end
        
        function [u_seq, cost_seq, x_traj] = solve(obj, x0)
            % Apply LQR control law
            % For compatibility with the MPC interface
            
            % Just return the current control input for the current state
            u = -obj.K * x0;
            
            % Return single step of control input, cost, and state trajectory
            u_seq = u;
            cost_seq = x0' * eye(length(x0)) * x0 + u' * 0.5 * u;  % Approximate cost
            x_traj = [x0, x0];  % Placeholder for state trajectory
        end
    end
end 