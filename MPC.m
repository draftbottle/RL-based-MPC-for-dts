classdef MPC < handle
    % MPC: Model Predictive Control implementation
    % This class implements MPC with the terminal cost derived from RLMPC
    
    properties
        system          % Dynamic system model
        horizon         % Prediction horizon (N)
        vfa             % Value function approximator
        W               % Value function weights
        has_terminal_cost % Flag to indicate if terminal cost is used
        last_solution   % Store the last solution for warm start
    end
    
    methods
        function obj = MPC(system, horizon, vfa, W)
            % Constructor
            obj.system = system;
            obj.horizon = horizon;
            obj.vfa = vfa;
            obj.W = W;
            obj.last_solution = [];
            
            % Check if terminal cost is used
            if isempty(vfa) || isempty(W)
                obj.has_terminal_cost = false;
            else
                obj.has_terminal_cost = true;
            end
        end
        
        function [u_seq, cost_seq, x_traj] = solve(obj, x0)
            % Solve the MPC optimization problem
            % Returns the optimal control sequence, cost sequence, and state trajectory
            
            % Get system dimensions
            n = obj.system.n;  % State dimension
            m = obj.system.m;  % Input dimension
            N = obj.horizon;   % Prediction horizon
            
            % Initialize optimization variables
            num_vars = m * N;  % Number of control inputs over horizon
            
            % Initial state
            x_init = x0;
            
            % Define the objective function
            function [J, grad] = objective_function(U)
                % Reshape U vector to control sequence
                U_seq = reshape(U, [m, N]);
                
                % Initialize cost and state trajectory
                J = 0;
                x = x_init;
                x_traj_local = zeros(n, N+1);
                x_traj_local(:, 1) = x;
                
                % Initialize gradient if needed
                if nargout > 1
                    grad = zeros(size(U));
                end
                
                % Simulate the system over the horizon and compute cost
                cost_seq_local = zeros(1, N);
                for i = 1:N
                    u_i = U_seq(:, i);
                    
                    % Stage cost
                    stage_cost = obj.system.stage_cost(x, u_i);
                    cost_seq_local(i) = stage_cost;
                    J = J + stage_cost;
                    
                    % System dynamics
                    x = obj.system.dynamics(x, u_i);
                    x_traj_local(:, i+1) = x;
                end
                
                % Add terminal cost if available
                if obj.has_terminal_cost
                    terminal_cost = obj.vfa.evaluate(x, obj.W);
                    J = J + terminal_cost;
                end
                
                % We rely on numeric approximation for the gradient
            end
            
            % Define the constraints function for nonlinear constraints
            function [c, ceq, gradc, gradceq] = constraint_function(U)
                % No inequality constraints for the basic implementation
                c = [];
                gradc = [];
                
                % No equality constraints
                ceq = [];
                gradceq = [];
            end
            
            % Set optimization options (improved for convergence)
            options = optimoptions('fmincon', ...
                'Display', 'off', ...
                'Algorithm', 'sqp', ...
                'MaxIterations', 500, ...                      % Increased from 200
                'MaxFunctionEvaluations', 10000, ...           % Increased from 5000
                'OptimalityTolerance', 1e-3, ...               % Relaxed from 1e-4
                'ConstraintTolerance', 1e-3, ...               % Relaxed from 1e-4
                'StepTolerance', 1e-5, ...                     % Relaxed from 1e-6
                'SpecifyObjectiveGradient', false, ...
                'CheckGradients', false, ...
                'ScaleProblem', 'obj-and-constr', ...          % Added scaling
                'HessianApproximation', 'bfgs');               % Added hessian approximation
            
            % Initial guess for control sequence
            if isempty(obj.last_solution) || length(obj.last_solution) ~= num_vars
                % If no previous solution or horizon changed, use zeros
                U0 = zeros(num_vars, 1);
                
                % For nonlinear vehicle, use a better initial guess
                if isa(obj.system, 'NonlinearVehicle')
                    % Simple position-based heuristic for vehicle
                    target = [0; 0; 0];
                    curr_pos = x_init(1:2);
                    curr_theta = x_init(3);
                    
                    % Vector to target
                    vec_to_target = target(1:2) - curr_pos;
                    dist_to_target = norm(vec_to_target);
                    
                    if dist_to_target > 0.1
                        % Desired heading angle
                        desired_theta = atan2(vec_to_target(2), vec_to_target(1));
                        
                        % Angular difference (shortest path)
                        ang_diff = mod(desired_theta - curr_theta + pi, 2*pi) - pi;
                        
                        % Initial control: move toward target with appropriate turning
                        v_init = min(0.5, dist_to_target);
                        omega_init = 0.5 * ang_diff;
                        
                        % Clip to bounds
                        v_init = max(min(v_init, obj.system.v_bounds(2)), obj.system.v_bounds(1));
                        omega_init = max(min(omega_init, obj.system.omega_bounds(2)), obj.system.omega_bounds(1));
                        
                        % Set initial guess
                        for i = 1:N
                            U0((i-1)*m+1:(i-1)*m+m) = [v_init; omega_init];
                        end
                    end
                end
            else
                % Warm start from last solution (shifted)
                U0 = [obj.last_solution(m+1:end); zeros(m, 1)];
            end
            
            % Set bounds for control inputs (if applicable)
            lb = [];
            ub = [];
            
            % For nonlinear vehicle with control constraints
            if isa(obj.system, 'NonlinearVehicle')
                lb = repmat([obj.system.v_bounds(1); obj.system.omega_bounds(1)], [N, 1]);
                ub = repmat([obj.system.v_bounds(2); obj.system.omega_bounds(2)], [N, 1]);
            end
            
            % Try multiple optimization attempts with different options if needed
            exitflag = -99;
            U_opt = U0;
            
            % First attempt: SQP algorithm
            try
                [U_opt, ~, exitflag] = fmincon(@objective_function, U0, [], [], [], [], ...
                                              lb, ub, @constraint_function, options);
            catch
                warning('First optimization attempt failed. Trying fallback method.');
            end
            
            % If first attempt failed, try with interior-point algorithm
            if exitflag <= 0
                options.Algorithm = 'interior-point';
                options.HessianApproximation = 'bfgs';
                options.InitBarrierParam = 0.1;                % Added for interior-point
                options.InitTrustRegionRadius = 1;             % Added for better convergence
                try
                    [U_opt, ~, exitflag] = fmincon(@objective_function, U0, [], [], [], [], ...
                                                  lb, ub, @constraint_function, options);
                catch
                    warning('Second optimization attempt failed. Using best available solution.');
                end
                
                % If that still fails, try active-set as last resort with relaxed tolerances
                if exitflag <= 0
                    options.Algorithm = 'active-set';
                    options.OptimalityTolerance = 1e-2;        % Further relaxed
                    options.ConstraintTolerance = 1e-2;        % Further relaxed
                    try
                        [U_opt, ~, exitflag] = fmincon(@objective_function, U0, [], [], [], [], ...
                                                      lb, ub, @constraint_function, options);
                    catch
                        warning('Third optimization attempt failed. Using best available solution.');
                    end
                end
            end
            
            % Check if optimization was successful
            if exitflag <= 0
                warning('MPC optimization did not converge. Exitflag: %d', exitflag);
                % We'll still use the best solution found so far
                
                % For nonlinear vehicle, use a simple fallback strategy if optimization completely fails
                if isa(obj.system, 'NonlinearVehicle') && norm(U_opt - U0) < 1e-6
                    % If solution is essentially unchanged from initial guess
                    % Generate a simple control sequence to gradually slow down and stabilize
                    for i = 1:N
                        % Start with current guess and gradually reduce inputs
                        decay = max(0, 1 - i/(N/2));  % Decay factor
                        if i <= 2
                            % Keep first couple inputs
                            continue;
                        elseif m == 2 % Assuming this is velocity and angular velocity
                            % Gradually reduce velocity and steering
                            U_opt((i-1)*m+1) = U_opt((i-1)*m+1) * decay;     % Reduce velocity
                            U_opt((i-1)*m+2) = U_opt((i-1)*m+2) * decay^2;   % Reduce steering faster
                        else
                            % Generic reduction for other systems
                            U_opt((i-1)*m+1:i*m) = U_opt((i-1)*m+1:i*m) * decay;
                        end
                    end
                    warning('Using fallback control strategy due to optimization failure.');
                end
            end
            
            % Store solution for warm start next time
            obj.last_solution = U_opt;
            
            % Extract optimal control sequence
            U_opt_seq = reshape(U_opt, [m, N]);
            u_seq = U_opt_seq;
            
            % Compute trajectory and cost sequence with optimal inputs
            x_traj = zeros(n, N+1);
            x_traj(:, 1) = x0;
            cost_seq = zeros(1, N);
            
            for i = 1:N
                % Stage cost
                cost_seq(i) = obj.system.stage_cost(x_traj(:, i), u_seq(:, i));
                
                % System dynamics
                x_traj(:, i+1) = obj.system.dynamics(x_traj(:, i), u_seq(:, i));
            end
        end
    end
end 