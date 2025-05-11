function plot_comparison_linear(x_rlmpc, x_lqr, x_mpc_wtc, u_rlmpc, u_lqr, u_mpc_wtc, ...
                             cost_rlmpc, cost_lqr, cost_mpc_wtc, W_history, cost_history, iter_history)
% PLOT_COMPARISON_LINEAR Plot comparison results for the linear system
%
%   This function plots state trajectories, control inputs, accumulated costs, 
%   and value function weight convergence for the linear system case

% Calculate time vector
sim_steps = size(x_rlmpc, 2) - 1;
time = 0:sim_steps;

% Calculate accumulated costs
acc_cost_rlmpc = cumsum(cost_rlmpc);
acc_cost_lqr = cumsum(cost_lqr);
acc_cost_mpc_wtc = cumsum(cost_mpc_wtc);

% Create figure for state trajectories
figure('Name', 'Linear System: State Trajectories', 'Position', [100, 100, 1000, 600]);

% Plot state trajectories
subplot(2, 2, 1);
hold on;
plot(time, x_rlmpc(1, :), 'b-', 'LineWidth', 2);
plot(time, x_lqr(1, :), 'r--', 'LineWidth', 1.5);
plot(time, x_mpc_wtc(1, :), 'g-.', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time step');
ylabel('State x_1');
title('State x_1 Trajectory');
legend('RLMPC', 'LQR', 'MPC w/o TC');

subplot(2, 2, 2);
hold on;
plot(time, x_rlmpc(2, :), 'b-', 'LineWidth', 2);
plot(time, x_lqr(2, :), 'r--', 'LineWidth', 1.5);
plot(time, x_mpc_wtc(2, :), 'g-.', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time step');
ylabel('State x_2');
title('State x_2 Trajectory');
legend('RLMPC', 'LQR', 'MPC w/o TC');

% Plot state phase portrait
subplot(2, 2, 3);
hold on;
plot(x_rlmpc(1, :), x_rlmpc(2, :), 'b-', 'LineWidth', 2);
plot(x_lqr(1, :), x_lqr(2, :), 'r--', 'LineWidth', 1.5);
plot(x_mpc_wtc(1, :), x_mpc_wtc(2, :), 'g-.', 'LineWidth', 1.5);
plot(x_rlmpc(1, 1), x_rlmpc(2, 1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot(0, 0, 'kx', 'MarkerSize', 8, 'LineWidth', 2);
hold off;
grid on;
xlabel('State x_1');
ylabel('State x_2');
title('State Phase Portrait');
legend('RLMPC', 'LQR', 'MPC w/o TC', 'Initial State', 'Target State');

% Plot control inputs
subplot(2, 2, 4);
hold on;
stairs(0:sim_steps-1, u_rlmpc, 'b-', 'LineWidth', 2);
stairs(0:sim_steps-1, u_lqr, 'r--', 'LineWidth', 1.5);
stairs(0:sim_steps-1, u_mpc_wtc, 'g-.', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time step');
ylabel('Control input u');
title('Control Inputs');
legend('RLMPC', 'LQR', 'MPC w/o TC');

% Create figure for accumulated costs
figure('Name', 'Linear System: Performance Comparison', 'Position', [100, 700, 1000, 600]);

% Plot accumulated costs
subplot(2, 2, 1);
hold on;
% Use time range that matches the actual trajectory length
plot_time = 0:length(acc_cost_rlmpc)-1;
plot(plot_time, acc_cost_rlmpc, 'b-', 'LineWidth', 2);
plot(plot_time, acc_cost_lqr, 'r--', 'LineWidth', 1.5);
plot(plot_time, acc_cost_mpc_wtc, 'g-.', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time step');
ylabel('Accumulated cost');
title('Accumulated Costs (ACC)');
legend('RLMPC', 'LQR', 'MPC w/o TC');
% Set y-axis to start from 0 for better comparison
y_lim = get(gca, 'YLim');
set(gca, 'YLim', [0, y_lim(2)]);

% Plot final accumulated costs as bar chart
subplot(2, 2, 2);
final_costs = [acc_cost_rlmpc(end), acc_cost_lqr(end), acc_cost_mpc_wtc(end)];
bar(final_costs);
set(gca, 'XTickLabel', {'RLMPC', 'LQR', 'MPC w/o TC'});
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

% Plot weight convergence during training
if ~isempty(W_history)
    subplot(2, 2, 4);
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
disp('Performance Metrics:');
disp(['RLMPC Total Cost: ', num2str(acc_cost_rlmpc(end))]);
disp(['LQR Total Cost: ', num2str(acc_cost_lqr(end))]);
disp(['MPC w/o TC Total Cost: ', num2str(acc_cost_mpc_wtc(end))]);
disp(['RLMPC vs LQR improvement: ', num2str((acc_cost_lqr(end) - acc_cost_rlmpc(end))/acc_cost_lqr(end)*100), '%']);
disp(['RLMPC vs MPC w/o TC improvement: ', num2str((acc_cost_mpc_wtc(end) - acc_cost_rlmpc(end))/acc_cost_mpc_wtc(end)*100), '%']);

end 