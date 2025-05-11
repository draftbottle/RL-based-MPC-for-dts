classdef NonlinearVehicle < handle
    % NonlinearVehicle: Class implementing a non-holonomic vehicle system
    % States: [x, y, θ]' where (x,y) is position and θ is orientation
    % Inputs: [v, ω]' where v is linear velocity and ω is angular velocity
    
    properties
        x_bounds        % State bounds [min, max]
        v_bounds        % Linear velocity bounds [min, max]
        omega_bounds    % Angular velocity bounds [min, max]
        dt              % Time step
        Q               % State cost matrix
        R               % Input cost matrix
        x               % Current state [x, y, θ]'
        n               % State dimension
        m               % Input dimension
        y_bounds        % Y position bounds [min, max]
        theta_bounds    % Orientation bounds [min, max]
    end
    
    methods
        function obj = NonlinearVehicle(x_bounds, v_bounds, omega_bounds)
            % Constructor
            obj.x_bounds = x_bounds;
            obj.v_bounds = v_bounds;
            obj.omega_bounds = omega_bounds;
            obj.dt = 0.1;  % 100ms time step
            
            % State and input dimensions
            obj.n = 3;  % [x, y, θ]
            obj.m = 2;  % [v, ω]
            
            % Default bounds for y and theta
            obj.y_bounds = x_bounds;  % Same bounds as x by default
            obj.theta_bounds = [-pi, pi]; % Full orientation range
            
            % Cost matrices - adjusted to better balance state vs input costs
            obj.Q = diag([1, 1, 0.2]);  % State cost
            obj.R = diag([0.05, 0.01]);  % Input cost
            
            % Initialize state
            obj.x = zeros(obj.n, 1);
        end
        
        function x_next = dynamics(obj, x, u)
            % Non-holonomic vehicle dynamics
            % x_{k+1} = x_k + g(x_k)u_k
            
            % Extract states and inputs
            theta = x(3);
            v = u(1);    % Linear velocity
            omega = u(2); % Angular velocity
            
            % Apply input constraints with smooth saturation
            v = obj.smooth_saturation(v, obj.v_bounds(1), obj.v_bounds(2));
            omega = obj.smooth_saturation(omega, obj.omega_bounds(1), obj.omega_bounds(2));
            
            % Improve numerical integration for better accuracy
            % Use Runge-Kutta method (RK4) for more accurate dynamics
            k1 = obj.vehicle_ode(x, [v; omega]);
            k2 = obj.vehicle_ode(x + obj.dt/2 * k1, [v; omega]);
            k3 = obj.vehicle_ode(x + obj.dt/2 * k2, [v; omega]);
            k4 = obj.vehicle_ode(x + obj.dt * k3, [v; omega]);
            
            % Update state using RK4 integration
            x_next = x + obj.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % Apply state constraints with smooth saturation
            x_next(1) = obj.smooth_saturation(x_next(1), obj.x_bounds(1), obj.x_bounds(2));
            x_next(2) = obj.smooth_saturation(x_next(2), obj.y_bounds(1), obj.y_bounds(2));
            
            % Normalize angle to [-π, π]
            x_next(3) = mod(x_next(3) + pi, 2*pi) - pi;
        end
        
        function dxdt = vehicle_ode(obj, x, u)
            % ODE function for vehicle dynamics
            theta = x(3);
            v = u(1);
            omega = u(2);
            
            dxdt = [v * cos(theta); 
                    v * sin(theta);
                    omega];
        end
        
        function val = smooth_saturation(obj, val, min_val, max_val)
            % Smooth saturation function to avoid discontinuities in gradient
            % Uses tanh-based soft saturation
            buffer = 0.01 * (max_val - min_val);
            if val > max_val - buffer
                val = max_val - buffer * tanh((max_val - val) / buffer);
            elseif val < min_val + buffer
                val = min_val + buffer * tanh((val - min_val) / buffer);
            end
        end
        
        function cost = stage_cost(obj, x, u)
            % Calculate stage cost
            % Penalize deviation from target (origin) and control effort
            
            % Get target state (origin)
            x_target = [0; 0; 0];
            
            % Calculate error
            error = x - x_target;
            
            % Normalize angle error to [-π, π]
            error(3) = mod(error(3) + pi, 2*pi) - pi;
            
            % Stage cost with additional smoothness term for control inputs
            if length(u) > 1
                u_prev = u(:, 1);
                u_curr = u(:, end);
                smoothness_cost = 0.01 * norm(u_curr - u_prev)^2;
            else
                smoothness_cost = 0;
            end
            
            cost = error' * obj.Q * error + u' * obj.R * u + smoothness_cost;
        end
        
        function x_next = step(obj, u)
            % Take a step in the system with input u
            obj.x = obj.dynamics(obj.x, u);
            x_next = obj.x;
        end
        
        function x = get_state(obj)
            % Get current state
            x = obj.x;
        end
        
        function set_state(obj, x0)
            % Set state
            obj.x = x0;
        end
        
        function x0 = get_initial_state(obj)
            % Get initial state for simulations
            x0 = [1.5; 1.5; pi/4]; % Example initial state
        end
        
        function samples = generate_samples(obj, num_samples)
            % Generate random state samples for policy evaluation
            
            % Position samples within bounds
            x_samples = obj.x_bounds(1) + (obj.x_bounds(2) - obj.x_bounds(1)) * rand(1, num_samples);
            y_samples = obj.y_bounds(1) + (obj.y_bounds(2) - obj.y_bounds(1)) * rand(1, num_samples);
            
            % Orientation samples between -pi and pi
            theta_samples = 2 * pi * rand(1, num_samples) - pi;
            
            % Combine into state samples
            samples = [x_samples; y_samples; theta_samples];
            
            % Add some samples near the target for better convergence
            num_near_target = min(50, floor(num_samples/4));
            if num_near_target > 0
                target_vicinity = 0.5; % Range around target
                samples(:, 1:num_near_target) = [
                    target_vicinity * (2*rand(1, num_near_target) - 1);
                    target_vicinity * (2*rand(1, num_near_target) - 1);
                    pi * (2*rand(1, num_near_target) - 1)
                ];
            end
        end
        
        function [A, B] = linearize(obj, x, u)
            % Linearize the system at a given point (x, u)
            % Returns Jacobians A = df/dx, B = df/du
            
            % Extract states and inputs
            theta = x(3);
            v = u(1);
            
            % Jacobian with respect to state (discretized)
            A = eye(3) + obj.dt * [0, 0, -v*sin(theta);
                                  0, 0, v*cos(theta);
                                  0, 0, 0];
            
            % Jacobian with respect to input (discretized)
            B = obj.dt * [cos(theta), 0;
                         sin(theta), 0;
                         0, 1];
        end
    end
end 