classdef LinearSystem < handle
    % LinearSystem: Class implementing a discrete-time linear system
    
    properties
        A       % System matrix
        B       % Input matrix
        Q       % State cost matrix
        R       % Input cost matrix
        n       % State dimension
        m       % Input dimension
        x       % Current state
    end
    
    methods
        function obj = LinearSystem(A, B, Q, R)
            % Constructor
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            
            % Determine dimensions
            obj.n = size(A, 1);
            obj.m = size(B, 2);
            
            % Initialize state
            obj.x = zeros(obj.n, 1);
        end
        
        function x_next = dynamics(obj, x, u)
            % System dynamics: x_{k+1} = Ax_k + Bu_k
            x_next = obj.A * x + obj.B * u;
        end
        
        function cost = stage_cost(obj, x, u)
            % Stage cost: l(x, u) = x'Qx + u'Ru
            cost = x' * obj.Q * x + u' * obj.R * u;
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
            x0 = [1; 1]; % Example initial state
        end
        
        function samples = generate_samples(obj, num_samples)
            % Generate random state samples for policy evaluation
            % For the linear system, sample within a reasonable range
            samples = 2 * rand(obj.n, num_samples) - 1; % Range [-1, 1]
        end
        
        function is_stable = is_stable(obj)
            % Check if the system is stable
            eigenvalues = eig(obj.A);
            is_stable = all(abs(eigenvalues) < 1);
        end
    end
end 