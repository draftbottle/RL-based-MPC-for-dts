%% RLMPC - Reinforcement Learning Model Predictive Control
% This script implements the RLMPC framework by combining RL and MPC


clear; clc; close all;

%% Select system type
system_type = 'nonlinear'; % Options: 'linear', 'nonlinear'

%% Parameters
% Common parameters
num_samples = 500;     % Number of samples for policy evaluation

%% System setup based on type
if strcmp(system_type, 'linear')
    % Linear system parameters
    A = [1, 0.5; 0.1, 0.5];
    B = [1; 0];
    Q = eye(2);
    R = 0.5;
    N = 3;  % Prediction horizon
    
    % Define polynomial basis functions 
    basis_order = 2;
    alpha = 1e-2; % 
    lambda_reg = 0.01; % Added for consistency
    
   
    sys = LinearSystem(A, B, Q, R);
    
    [K_lqr, P_lqr] = dlqr(A, B, Q, R);
    
    
    P_terminal = 5 * eye(2); 
    
elseif strcmp(system_type, 'nonlinear')
    % Nonlinear system parameters (non-holonomic vehicle)
    N = 5;  % Prediction horizon for RLMPC
    N_mpc = 20; 
    
    % State and input constraints
    x_bounds = [0, 2];
    y_bounds = x_bounds; % Assuming symmetric bounds for y
    v_bounds = [-1, 1];
    omega_bounds = [-4, 4];
    
    
    basis_order = 4; 
    % Original: 4 (35 features), Paper's hint: 6 (84 features)
    
    
    alpha = 2e-3; % More reasonable alpha since we'll average the gradients
    
    
    lambda_reg = 0.01;  
    
   
    sys = NonlinearVehicle(x_bounds, v_bounds, omega_bounds);
    sys.y_bounds = y_bounds; 
    
 
    P_terminal = diag([5, 5, 1]); 
else
    error('Invalid system type. Choose "linear" or "nonlinear".');
end

%% Initialize value function approximation
vfa = ValueFunctionApproximator(basis_order, size(sys.get_state(), 1));

%% RLMPC Algorithm - Online Learning (Closer to Algorithm 1)
disp('Starting RLMPC online learning and simulation...');

% Simulation parameters
sim_steps_total = 200; % Total steps for the system to run (learning + execution)
learning_phase_steps = 100; % Number of steps during which learning (W update) happens
                           
                            

% Initialize system
sys.set_state(sys.get_initial_state()); % Set initial state of the system
x_k = sys.get_state();

% History for plotting actual system trajectory during learning/execution
x_history_actual = zeros(size(sys.get_state(), 1), sim_steps_total + 1);
if strcmp(system_type, 'linear')
    input_dim = size(B, 2); 
else
    input_dim = sys.m; 
end
u_history_actual = zeros(input_dim, sim_steps_total);
cost_history_actual = zeros(1, sim_steps_total);
x_history_actual(:, 1) = x_k;

W = zeros(vfa.get_num_features(), 1); 
W_history_online = zeros(vfa.get_num_features(), sim_steps_total); 

% MPC controller instance (reused)
mpc_learner = MPC(sys, N, vfa, W); % N is the MPC horizon for RLMPC

% Learning control parameters
alpha_sgd = 1e-5; % SGD learning rate, liner=1e-3,nonliner=1e-5
lambda_reg_sgd = 0.001; 
epsilon_sgd_W_change = 1e-6; % Convergence threshold for W change if we want early stop
max_sgd_updates_per_k = num_samples; 

learning_flag = true; 
consecutive_no_W_change_count = 0;
max_consecutive_no_W_change = 5; 

for k = 1:sim_steps_total
    fprintf('System Step k = %d/%d\n', k, sim_steps_total);
    W_history_online(:, k) = W; 

    % --- Policy Generation (MPC solve) ---
    mpc_learner.W = W; 
    try
        [u_sequence_k, cost_sequence_k, x_predicted_traj_k] = mpc_learner.solve(x_k);
        u_0_k = u_sequence_k(:, 1);
    catch e
        warning('MPC solve failed at system step k=%d: %s. Using zero input.', k, e.message);
        u_0_k = zeros(sys.m, 1); 
    end


    actual_cost_k = sys.stage_cost(x_k, u_0_k);
    x_k_plus_1 = sys.step(u_0_k); 


    u_history_actual(:, k) = u_0_k;
    cost_history_actual(k) = actual_cost_k;
    x_history_actual(:, k+1) = x_k_plus_1;

    % --- Policy Evaluation and W Update (if in learning phase) ---
    if learning_flag && k <= learning_phase_steps
        fprintf('  Learning phase: Updating W...\n');
        W_before_update_this_k = W;

        % 1. Sample Sk (as in your original code)
        %    It's better if Sk is sampled fresh or partially updated each k
        %    For simplicity here, let's resample it.
        samples_Sk = sys.generate_samples(num_samples); 

        for s_idx = 1:num_samples 
            x_j_sample = samples_Sk(:, s_idx);

            % 2. For each x_j in Sk, generate training data J'(x_j, u_j*)
            %    This J' is calculated using the SAME W that was used for u_0_k (i.e., W before this inner loop's updates)
            %    Or, more aligned with PI, J' should be for policy pi_k (based on W at start of step k)
            %    So, mpc_learner.W should be W_before_update_this_k
            mpc_temp_eval = MPC(sys, N, vfa, W_before_update_this_k);
            try
                [u_seq_sample, cost_seq_sample, x_traj_sample] = mpc_temp_eval.solve(x_j_sample);
                
            
                terminal_value_approx_sample = W_before_update_this_k' * vfa.get_features(x_traj_sample(:, end));
                J_target_for_sample_j = sum(cost_seq_sample) + terminal_value_approx_sample;
                
                % 3. SGD Update for W for this sample x_j_sample
                phi_j = vfa.get_features(x_j_sample); 
               
                td_error = J_target_for_sample_j - (W' * phi_j); 
                
                
                delta_W_sgd = alpha_sgd * td_error * phi_j;
                
        
                regularization_term_sgd = alpha_sgd * lambda_reg_sgd * W;
                delta_W_sgd = delta_W_sgd - regularization_term_sgd;
                
                W = W + delta_W_sgd; % Update W immediately

      
                max_grad_norm = 1.0; 
                current_grad_norm = norm(delta_W_sgd);
                if current_grad_norm > max_grad_norm
                    delta_W_sgd = delta_W_sgd * (max_grad_norm / current_grad_norm);
                    fprintf('  Gradient norm clipped from %e to %e\n', current_grad_norm, max_grad_norm);
                end
                
                
                W = min(max(W, -100.0), 100.0); 
                
            catch e_sgd
             
            end
        end 


        w_change_norm_this_k = norm(W - W_before_update_this_k);
        fprintf('  Norm of W update (delta_W_total_for_k): %e, Norm of W: %e\n', ...
            w_change_norm_this_k, norm(W));
        if w_change_norm_this_k < epsilon_sgd_W_change
            consecutive_no_W_change_count = consecutive_no_W_change_count + 1;
            if consecutive_no_W_change_count >= max_consecutive_no_W_change
                fprintf('  W has not changed significantly for %d steps. Stopping learning.\n', max_consecutive_no_W_change);
                learning_flag = false; % Corresponds to Flag = 1
            end
        else
            consecutive_no_W_change_count = 0; % Reset counter
        end
         % Check for NaN/Inf in W
        if any(isnan(W)) || any(isinf(W))
            error('Error: W contains NaN or Inf values. System step k=%d. Stopping.', k);
        end
    end % End of learning_flag check

    % Update current state for next system step
    x_k = x_k_plus_1;

    % If target reached or system becomes unstable, you might want to break
    if norm(x_k(1:2)) < 0.01 && strcmp(system_type, 'nonlinear') % Example for vehicle
        fprintf('Target reached at step k=%d.\n', k);
        break;
    end
    if norm(x_k) > 100 % Generic instability check
        fprintf('System seems unstable at step k=%d. State norm: %f\n', k, norm(x_k));
        break;
    end

end % End of system simulation loop (k)

actual_sim_steps = k;
x_history_actual = x_history_actual(:, 1:actual_sim_steps+1);
u_history_actual = u_history_actual(:, 1:actual_sim_steps);
cost_history_actual = cost_history_actual(1:actual_sim_steps);
W_history_online = W_history_online(:, 1:actual_sim_steps);

%% Evaluation of the FINAL learned policy (RLMPC Epi. 2 from paper)
disp('Evaluating final policy (RLMPC Epi. 2)...');
mpc_final_eval = MPC(sys, N, vfa, W); % MPC with the FINAL learned W
[x_rlmpc_epi2, u_rlmpc_epi2, cost_rlmpc_epi2_traj] = simulate_system(sys, mpc_final_eval, sys.get_initial_state(), 50);
total_cost_rlmpc_epi2 = sum(cost_rlmpc_epi2_traj);
fprintf('Total cost RLMPC (Epi. 2 with final W): %f\n', total_cost_rlmpc_epi2);

%% Evaluate final policy and compare with baselines
disp('Comparing with baseline controllers...');

if strcmp(system_type, 'linear')
    lqr_controller = LQRController(K_lqr);
    [x_lqr, u_lqr, cost_lqr_traj] = simulate_system(sys, lqr_controller, sys.get_initial_state(), 50);
    total_cost_lqr = sum(cost_lqr_traj);
    fprintf('Total cost LQR: %f\n', total_cost_lqr);
    
    mpc_wtc = MPC(sys, N, [], []); 
    [x_mpc_wtc, u_mpc_wtc, cost_mpc_wtc_traj] = simulate_system(sys, mpc_wtc, sys.get_initial_state(), 50);
    total_cost_mpc_wtc = sum(cost_mpc_wtc_traj);
    fprintf('Total cost MPC_WTC: %f\n', total_cost_mpc_wtc);
    

    terminal_cost_func = @(x) x' * P_terminal * x;
    mpc_tc = CustomTerminalCostMPC(sys, N, terminal_cost_func);
    [x_mpc_tc, u_mpc_tc, cost_mpc_tc_traj] = simulate_system(sys, mpc_tc, sys.get_initial_state(), 50);
    total_cost_mpc_tc = sum(cost_mpc_tc_traj);
    fprintf('Total cost MPC with terminal cost: %f\n', total_cost_mpc_tc);
    

    iter_history_dummy = 1:actual_sim_steps;
    
    figure;
    subplot(2,1,1);
    plot(W_history_online'); title('W weights evolution during online learning (k)'); xlabel('System Step k'); ylabel('Weight Value');
    legend_entries = arrayfun(@(i) sprintf('W%d', i), 1:size(W_history_online,1), 'UniformOutput', false);
    legend(legend_entries);
    subplot(2,1,2);

    W_norm_change = vecnorm(diff(W_history_online,1,2));
    plot(W_norm_change); title('Norm of W change per step k'); xlabel('System Step k'); ylabel('||W_{k+1} - W_k||');
    

    plot_comparison_linear(x_rlmpc_epi2, x_lqr, x_mpc_wtc, u_rlmpc_epi2, u_lqr, u_mpc_wtc, ...
                          total_cost_rlmpc_epi2, total_cost_lqr, total_cost_mpc_wtc, ...
                          W_history_online, cost_history_actual, iter_history_dummy);
    
elseif strcmp(system_type, 'nonlinear')

    try
        fprintf('Running traditional MPC with horizon %d...\n', N_mpc);
        mpc_long = MPC(sys, N_mpc, [], []); % Traditional MPC (long horizon, no VFA)
        [x_mpc_long, u_mpc_long, cost_mpc_long_traj] = simulate_system(sys, mpc_long, sys.get_initial_state(), 50);
        total_cost_mpc_long = sum(cost_mpc_long_traj);
        fprintf('Total cost MPC_long: %f\n', total_cost_mpc_long);
    catch

        try
            N_mpc_fallback = 15;
            fprintf('Retrying with reduced horizon %d...\n', N_mpc_fallback);
            mpc_long = MPC(sys, N_mpc_fallback, [], []);
            [x_mpc_long, u_mpc_long, cost_mpc_long_traj] = simulate_system(sys, mpc_long, sys.get_initial_state(), 50);
            total_cost_mpc_long = sum(cost_mpc_long_traj);
            fprintf('Total cost MPC_long (reduced horizon): %f\n', total_cost_mpc_long);
        catch

            x_mpc_long = x_rlmpc_epi2;
            u_mpc_long = u_rlmpc_epi2;
            cost_mpc_long_traj = cost_rlmpc_epi2_traj;
            total_cost_mpc_long = total_cost_rlmpc_epi2;
            fprintf('Using RLMPC results as fallback for comparison.\n');
        end
    end

    try
        mpc_wtc = MPC(sys, N, [], []);
        [x_mpc_wtc, u_mpc_wtc, cost_mpc_wtc_traj] = simulate_system(sys, mpc_wtc, sys.get_initial_state(), 50);
        total_cost_mpc_wtc = sum(cost_mpc_wtc_traj);
        fprintf('Total cost MPC_WTC (N=%d): %f\n', N, total_cost_mpc_wtc);
    catch

        x_mpc_wtc = x_rlmpc_epi2;
        u_mpc_wtc = u_rlmpc_epi2;
        cost_mpc_wtc_traj = cost_rlmpc_epi2_traj;
        total_cost_mpc_wtc = total_cost_rlmpc_epi2;
        fprintf('Using RLMPC results as fallback for comparison.\n');
    end
    

    try

        terminal_cost_func = @(x) x' * P_terminal * x;

        mpc_tc = CustomTerminalCostMPC(sys, N, terminal_cost_func);
        [x_mpc_tc, u_mpc_tc, cost_mpc_tc_traj] = simulate_system(sys, mpc_tc, sys.get_initial_state(), 50);
        total_cost_mpc_tc = sum(cost_mpc_tc_traj);
        fprintf('Total cost MPC with terminal cost: %f\n', total_cost_mpc_tc);

        x_mpc_tc = x_rlmpc_epi2;
        u_mpc_tc = u_rlmpc_epi2;
        cost_mpc_tc_traj = cost_rlmpc_epi2_traj;
        total_cost_mpc_tc = total_cost_rlmpc_epi2;
    end
    
    iter_history_dummy = 1:actual_sim_steps;

    figure;
    subplot(2,1,1);
    plot(W_history_online'); title('W weights evolution during online learning (k)'); xlabel('System Step k'); ylabel('Weight Value');
    legend_entries = arrayfun(@(i) sprintf('W%d', i), 1:size(W_history_online,1), 'UniformOutput', false);
    legend(legend_entries);
    subplot(2,1,2);

    W_norm_change = vecnorm(diff(W_history_online,1,2));
    plot(W_norm_change); title('Norm of W change per step k'); xlabel('System Step k'); ylabel('||W_{k+1} - W_k||');

    plot_comparison_nonlinear_extended(x_rlmpc_epi2, x_mpc_long, x_mpc_wtc, x_mpc_tc, ...
                                     u_rlmpc_epi2, u_mpc_long, u_mpc_wtc, u_mpc_tc, ...
                                     total_cost_rlmpc_epi2, total_cost_mpc_long, total_cost_mpc_wtc, total_cost_mpc_tc, ...
                                     W_history_online, cost_history_actual, iter_history_dummy);
    

    figure;
    subplot(2,1,1);
    plot(x_history_actual'); title('RLMPC Episode 1 - Learning Phase Trajectory'); xlabel('System Step k'); ylabel('State Value');
    state_labels = arrayfun(@(i) sprintf('State%d', i), 1:size(x_history_actual,1), 'UniformOutput', false);
    legend(state_labels);
    subplot(2,1,2);
    plot(u_history_actual'); title('RLMPC Episode 1 - Learning Phase Control'); xlabel('System Step k'); ylabel('Control Value');
    control_labels = arrayfun(@(i) sprintf('Control%d', i), 1:size(u_history_actual,1), 'UniformOutput', false);
    legend(control_labels);
end

disp('RLMPC simulation completed.'); 