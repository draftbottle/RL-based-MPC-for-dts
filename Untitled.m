plot_comparison_nonlinear(x_rlmpc, x_mpc_long, x_mpc_wtc, u_rlmpc, u_mpc_long, u_mpc_wtc, ...
                             total_cost_rlmpc, total_cost_mpc_long, total_cost_mpc_wtc, ...
                             W_history, cost_history, iter_history);
% 假设 w 是一个 35x200 的 double 数组
% w = rand(35, 200); % 示例数据，实际使用时请替换为你的数据

% 创建时间轴
time = 0:size(W_history, 2) - 1;

% 绘制每个变量的历史变动
figure;
hold on; % 保持图像，以便绘制多个曲线
for i = 1:size(W_history, 1)
    plot(time, W_history(i, :), 'DisplayName', ['Variable ', num2str(i)]);
end
hold off;

% 添加图例和标签
legend('show');
xlabel('Time');
ylabel('Value');
title('History of 35 Variables');
grid on; % 添加网格