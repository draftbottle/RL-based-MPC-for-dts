function [x_history, u_history, cost_history] = simulate_system(system, controller, x0, sim_steps)
% SIMULATE_SYSTEM Simulates a dynamical system with a given controller
%
%   Inputs:
%       system - System object with dynamics and cost methods
%       controller - Controller object with solve method
%       x0 - Initial state
%       sim_steps - Number of simulation steps
%
%   Outputs:
%       x_history - State trajectory history (n x sim_steps+1)
%       u_history - Control input history (m x sim_steps)
%       cost_history - Cost history (1 x sim_steps)

% Declare persistent variables at the beginning of the function
persistent consecutive_failures;

% Get system dimensions
n = size(x0, 1);
m = system.m;

% Initialize history arrays
x_history = zeros(n, sim_steps+1);
u_history = zeros(m, sim_steps);
cost_history = zeros(1, sim_steps);

% Set initial state
x_history(:, 1) = x0;
system.set_state(x0);

% Keep track of the last valid control input for fallback
last_valid_u = zeros(m, 1);

% Initialize consecutive failures counter if not already initialized
if isempty(consecutive_failures)
    consecutive_failures = 0;
end

% Simulation loop
for k = 1:sim_steps
    % Get current state
    x_k = system.get_state();
    
    % Try to compute control input using controller
    try
        [u_seq, ~, ~] = controller.solve(x_k);
        
        % Apply first control input
        if size(u_seq, 2) >= 1
            u_k = u_seq(:, 1);
            last_valid_u = u_k; % Update last valid control
            consecutive_failures = 0; % Reset failures counter
        else
            % If no control sequence returned, use last valid control
            u_k = last_valid_u;
            consecutive_failures = consecutive_failures + 1;
            warning('Empty control sequence at step %d, using last valid control', k);
        end
    catch e
        % If controller fails, use last valid control as fallback
        u_k = last_valid_u;
        consecutive_failures = consecutive_failures + 1;
        
        % More descriptive error message based on failure type
        if contains(e.message, 'converge')
            warning('Controller optimization failed to converge at step %d. Using last valid control.', k);
        elseif contains(e.message, 'infeasible')
            warning('Controller problem is infeasible at step %d. Using last valid control.', k);
        else
            warning('Controller failed at step %d: %s\nUsing last valid control.', k, e.message);
        end
        
        % If we have too many consecutive failures, create a safer fallback
        if consecutive_failures > 3
            % Create a safer fallback that gradually reduces control input
            decay_factor = 0.8;
            u_k = u_k * decay_factor;
            warning('Multiple consecutive failures detected. Reducing control input magnitude for safety.');
            
            % If using nonlinear vehicle, try to create a stabilizing input
            if isa(system, 'NonlinearVehicle') && length(u_k) > 1
                % If velocity is non-zero, reduce it further
                if abs(u_k(1)) > 0.1
                    u_k(1) = u_k(1) * 0.7; % Reduce velocity more aggressively
                end
                
                % Reduce steering angle to stabilize
                if length(u_k) > 1
                    u_k(2) = u_k(2) * 0.5; % Reduce steering angle
                end
            end
        end
    end
    
    % Apply input limits if this is a nonlinear vehicle (safety check)
    if isa(system, 'NonlinearVehicle')
        u_k(1) = max(min(u_k(1), system.v_bounds(2)), system.v_bounds(1));
        if length(u_k) > 1
            u_k(2) = max(min(u_k(2), system.omega_bounds(2)), system.omega_bounds(1));
        end
    end
    
    % Compute cost
    cost_history(k) = system.stage_cost(x_k, u_k);
    
    % Store control input
    u_history(:, k) = u_k;
    
    % Simulate system for one step
    x_next = system.step(u_k);
    
    % Store next state
    x_history(:, k+1) = x_next;
    
    % Provide progress update for long simulations
    if mod(k, 10) == 0
        fprintf('Simulation: %d/%d steps completed\n', k, sim_steps);
    end
end

end 