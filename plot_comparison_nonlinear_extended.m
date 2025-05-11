function plot_comparison_nonlinear_extended(x_rlmpc, x_mpc_long, x_mpc_wtc, x_mpc_tc, ...
                                     u_rlmpc, u_mpc_long, u_mpc_wtc, u_mpc_tc, ...
                                     cost_rlmpc, cost_mpc_long, cost_mpc_wtc, cost_mpc_tc, ...
                                     W_history, cost_history, iter_history)
% PLOT_COMPARISON_NONLINEAR_EXTENDED Plot comparison results for the nonlinear vehicle system
% including the additional MPC with terminal cost controller
%
%   This function is an extended version of plot_comparison_nonlinear that 
%   includes a fourth controller (MPC with terminal cost) in the comparison

% Calculate time vector
sim_steps = size(x_rlmpc, 2) - 1;
time = 0:sim_steps;

% Calculate accumulated costs
acc_cost_rlmpc = cumsum(cost_rlmpc);
acc_cost_mpc_long = cumsum(cost_mpc_long);
acc_cost_mpc_wtc = cumsum(cost_mpc_wtc);
acc_cost_mpc_tc = cumsum(cost_mpc_tc);

% Create figure for state trajectories
figure('Name', 'Nonlinear Vehicle: State Trajectories', 'Position', [100, 100, 1000, 600]);

% Plot position trajectory in xy-plane
subplot(2, 2, 1);
hold on;
plot(x_rlmpc(1, :), x_rlmpc(2, :), 'b-', 'LineWidth', 2);
plot(x_mpc_long(1, :), x_mpc_long(2, :), 'r--', 'LineWidth', 1.5);
plot(x_mpc_wtc(1, :), x_mpc_wtc(2, :), 'g-.', 'LineWidth', 1.5);
plot(x_mpc_tc(1, :), x_mpc_tc(2, :), 'm:', 'LineWidth', 2); % Add MPC-TC
plot(x_rlmpc(1, 1), x_rlmpc(2, 1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot(0, 0, 'kx', 'MarkerSize', 8, 'LineWidth', 2);
% Plot arrows to show orientation at intervals
interval = max(1, floor(sim_steps/10));
for i = 1:interval:sim_steps
    % RLMPC
    arrow_length = 0.1;
    theta = x_rlmpc(3, i);
    quiver(x_rlmpc(1, i), x_rlmpc(2, i), arrow_length*cos(theta), arrow_length*sin(theta), 0, 'b', 'LineWidth', 1.5);
end
hold off;
grid on;
xlabel('x position');
ylabel('y position');
title('Position Trajectory (x-y plane)');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC', 'Initial Position', 'Target Position');

% Plot x position over time
subplot(2, 2, 2);
hold on;
plot(time, x_rlmpc(1, :), 'b-', 'LineWidth', 2);
plot(time, x_mpc_long(1, :), 'r--', 'LineWidth', 1.5);
plot(time, x_mpc_wtc(1, :), 'g-.', 'LineWidth', 1.5);
plot(time, x_mpc_tc(1, :), 'm:', 'LineWidth', 2); % Add MPC-TC
hold off;
grid on;
xlabel('Time step');
ylabel('x position');
title('x Position over Time');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC');

% Plot y position over time
subplot(2, 2, 3);
hold on;
plot(time, x_rlmpc(2, :), 'b-', 'LineWidth', 2);
plot(time, x_mpc_long(2, :), 'r--', 'LineWidth', 1.5);
plot(time, x_mpc_wtc(2, :), 'g-.', 'LineWidth', 1.5);
plot(time, x_mpc_tc(2, :), 'm:', 'LineWidth', 2); % Add MPC-TC
hold off;
grid on;
xlabel('Time step');
ylabel('y position');
title('y Position over Time');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC');

% Plot orientation over time
subplot(2, 2, 4);
hold on;
plot(time, x_rlmpc(3, :), 'b-', 'LineWidth', 2);
plot(time, x_mpc_long(3, :), 'r--', 'LineWidth', 1.5);
plot(time, x_mpc_wtc(3, :), 'g-.', 'LineWidth', 1.5);
plot(time, x_mpc_tc(3, :), 'm:', 'LineWidth', 2); % Add MPC-TC
hold off;
grid on;
xlabel('Time step');
ylabel('Orientation \theta (rad)');
title('Orientation over Time');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC');

% Create figure for control inputs
figure('Name', 'Nonlinear Vehicle: Control Inputs', 'Position', [100, 700, 1000, 400]);

% Plot linear velocity input
subplot(1, 2, 1);
hold on;
stairs(0:sim_steps-1, u_rlmpc(1, :), 'b-', 'LineWidth', 2);
stairs(0:sim_steps-1, u_mpc_long(1, :), 'r--', 'LineWidth', 1.5);
stairs(0:sim_steps-1, u_mpc_wtc(1, :), 'g-.', 'LineWidth', 1.5);
stairs(0:sim_steps-1, u_mpc_tc(1, :), 'm:', 'LineWidth', 2); % Add MPC-TC
yline(1, 'k--');  % Upper bound
yline(-1, 'k--'); % Lower bound
hold off;
grid on;
xlabel('Time step');
ylabel('Linear velocity v');
title('Linear Velocity Input');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC', 'Bounds');

% Plot angular velocity input
subplot(1, 2, 2);
hold on;
stairs(0:sim_steps-1, u_rlmpc(2, :), 'b-', 'LineWidth', 2);
stairs(0:sim_steps-1, u_mpc_long(2, :), 'r--', 'LineWidth', 1.5);
stairs(0:sim_steps-1, u_mpc_wtc(2, :), 'g-.', 'LineWidth', 1.5);
stairs(0:sim_steps-1, u_mpc_tc(2, :), 'm:', 'LineWidth', 2); % Add MPC-TC
yline(4, 'k--');  % Upper bound
yline(-4, 'k--'); % Lower bound
hold off;
grid on;
xlabel('Time step');
ylabel('Angular velocity \omega');
title('Angular Velocity Input');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC', 'Bounds');

% Create figure for accumulated costs and training progress
figure('Name', 'Nonlinear Vehicle: Performance Comparison', 'Position', [100, 1100, 1000, 600]);

% Plot accumulated costs
subplot(2, 2, 1);
hold on;
% Use time range that matches the actual trajectory length
plot_time = 0:length(acc_cost_rlmpc)-1;
plot(plot_time, acc_cost_rlmpc, 'b-', 'LineWidth', 2);
plot(plot_time, acc_cost_mpc_long, 'r--', 'LineWidth', 1.5);
plot(plot_time, acc_cost_mpc_wtc, 'g-.', 'LineWidth', 1.5);
plot(plot_time, acc_cost_mpc_tc, 'm:', 'LineWidth', 2); % Add MPC-TC
hold off;
grid on;
xlabel('Time step');
ylabel('Accumulated cost');
title('Accumulated Costs (ACC)');
legend('RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC');
% Set y-axis to start from 0 for better comparison
y_lim = get(gca, 'YLim');
set(gca, 'YLim', [0, y_lim(2)]);

% Plot final accumulated costs as bar chart
subplot(2, 2, 2);
final_costs = [acc_cost_rlmpc(end), acc_cost_mpc_long(end), acc_cost_mpc_wtc(end), acc_cost_mpc_tc(end)];
bar(final_costs);
set(gca, 'XTickLabel', {'RLMPC', 'MPC-Long', 'MPC w/o TC', 'MPC-TC'});
grid on;
ylabel('Total cost');
title('Total Accumulated Cost');

% Plot average cost per iteration during training
if ~isempty(cost_history)
    subplot(2, 2, 3);
    % Filter out any NaN values that might be present in cost_history
    valid_indices = ~isnan(cost_history);
    valid_costs = cost_history(valid_indices);
    
    % Prepare iterations data
    if exist('iter_history', 'var') && ~isempty(iter_history)
        iterations = iter_history(valid_indices);
    else
        iterations = find(valid_indices);
    end
    
    % If there's at least one valid data point, plot it
    if ~isempty(valid_costs)
        plot(iterations, valid_costs, 'b-o', 'LineWidth', 2);
        grid on;
        xlabel('Policy iteration');
        ylabel('Average cost');
        title('Average Cost per Policy Iteration');
    else
        text(0.5, 0.5, 'No valid cost data available', 'HorizontalAlignment', 'center');
        axis off;
    end
end

% Plot value function weight convergence history
subplot(2, 2, 4);
if isempty(W_history)
    text(0.5, 0.5, 'No weight history available', 'HorizontalAlignment', 'center');
    axis off;
else
    % Use iter_history if provided, otherwise use 1:size(W_history, 2)
    if exist('iter_history', 'var') && ~isempty(iter_history)
        iterations = iter_history(1:size(W_history, 2));
    else
        iterations = 1:size(W_history, 2);
    end
    
    % Only plot if there are at least 2 iterations of data
    if length(iterations) > 1
        plot(iterations, W_history', 'LineWidth', 1.5);
        grid on;
        xlabel('Policy iteration');
        ylabel('Weight value');
        title('Value Function Weight Convergence');
        
        legend_str = cell(1, size(W_history, 1));
        for i = 1:size(W_history, 1)
            legend_str{i} = ['W_' num2str(i)];
        end
        legend(legend_str, 'Location', 'eastoutside');
    else
        text(0.5, 0.5, 'Insufficient weight history data', 'HorizontalAlignment', 'center');
        axis off;
    end
end

% Display performance metrics
disp('Performance Metrics (Nonlinear Vehicle with MPC-TC):');
disp(['RLMPC Total Cost: ', num2str(acc_cost_rlmpc(end))]);
disp(['MPC-Long Total Cost: ', num2str(acc_cost_mpc_long(end))]);
disp(['MPC w/o TC Total Cost: ', num2str(acc_cost_mpc_wtc(end))]);
disp(['MPC-TC Total Cost: ', num2str(acc_cost_mpc_tc(end))]);
disp(['RLMPC vs MPC-Long improvement: ', num2str((acc_cost_mpc_long(end) - acc_cost_rlmpc(end))/acc_cost_mpc_long(end)*100), '%']);
disp(['RLMPC vs MPC w/o TC improvement: ', num2str((acc_cost_mpc_wtc(end) - acc_cost_rlmpc(end))/acc_cost_mpc_wtc(end)*100), '%']);
disp(['RLMPC vs MPC-TC improvement: ', num2str((acc_cost_mpc_tc(end) - acc_cost_rlmpc(end))/acc_cost_mpc_tc(end)*100), '%']);
disp(['MPC-TC vs MPC w/o TC improvement: ', num2str((acc_cost_mpc_wtc(end) - acc_cost_mpc_tc(end))/acc_cost_mpc_wtc(end)*100), '%']);

end 