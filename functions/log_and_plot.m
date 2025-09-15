if controller_type == 0
    controller_name = 'PID';
    save('logs/log_PID.mat', 'log');
elseif controller_type == 1
    controller_name = 'SMC';
    save('logs/log_SMC.mat', 'log');
elseif controller_type == 2
    controller_name = 'INDI';
    save('logs/log_INDI.mat', 'log');
end

figure('Position', [100, 100, 500, 500], 'Color', 'w');

t = tiledlayout(2,1);
nexttile;
plot(log.time, rad2deg(log.phi_log), 'r', log.time, rad2deg(log.theta_log), 'g', log.time, rad2deg(log.psi_log), 'b', 'LineWidth', 1.5);
hold on;
plot(log.time, rad2deg(log.phi_des_log), 'r--', 'LineWidth', 1.0);
plot(log.time, rad2deg(log.theta_des_log), 'g--', 'LineWidth', 1.0);
plot(log.time, rad2deg(log.psi_des_log), 'b--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Angle (°)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'northeast');
grid minor
nexttile;
plot(log.time, log.T1_log, 'r', log.time, log.T2_log, 'g', log.time, log.T3_log, 'b', log.time, log.T4_log, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Location', 'northeast');
grid minor
set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
t.TileSpacing = 'compact';
t.Padding = 'compact';
exportgraphics(gcf, fullfile('figures',strcat([controller_name, '_ang'], '.png')), 'Resolution', 300)

figure('Position', [600, 100, 500, 500], 'Color', 'w')
t = tiledlayout(3,1);
nexttile;
plot(log.time, log.x_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.x_des_log, 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('X (m)');
legend('Actual', 'Desired', 'Location', 'best');
nexttile;
plot(log.time, log.y_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.y_des_log, 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Y (m)');
legend('Actual', 'Desired', 'Location', 'best');
nexttile;
plot(log.time, log.z_log, 'b', 'LineWidth', 1.5); hold on;
plot(log.time, log.z_des_log, 'r--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Z (m)');
legend('Actual', 'Desired', 'Location', 'best');
set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
t.TileSpacing = 'compact';
t.Padding = 'compact';
exportgraphics(gcf, fullfile('figures',strcat([controller_name, '_pos'], '.png')), 'Resolution', 300)

if flag.slung_active == 1
    figure('Position', [1100, 100, 500, 400], 'Color', 'w');
    plot(log.time, log.alpha_log, 'r', 'LineWidth', 1.5); hold on;
    plot(log.time, log.beta_log, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Slung Load Angle (deg)');
    legend('\alpha', '\beta', 'Location', 'best');
    grid on;
    set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
    exportgraphics(gcf, fullfile('figures',strcat([controller_name, '_slung'], '.png')), 'Resolution', 300)
end
