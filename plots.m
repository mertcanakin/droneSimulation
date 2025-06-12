pid = load('log_PID.mat');
smc = load('log_SMC.mat');
indi = load('log_INDI.mat');
time = pid.log.time;
lw = 1.5;

figure('Name', 'Results', 'Units', 'centimeters', 'Position', [10 2 20 20])

t = tiledlayout(3,1);
nexttile;
plot(time, rad2deg(pid.log.phi_log), 'r', 'LineWidth', lw);
hold on;
plot(time, rad2deg(smc.log.phi_log), 'g', 'LineWidth', lw);
hold on;
plot(time, rad2deg(indi.log.phi_log), 'b', 'LineWidth', lw);
hold on;
plot(time, rad2deg(pid.log.phi_des_log), 'k--', 'LineWidth', 0.5);
xlabel('Time (s)'); ylabel('Roll (°)');
legend('PID','SMC','INDI');

nexttile;
plot(time, rad2deg(pid.log.theta_log), 'r', 'LineWidth', lw);
hold on;
plot(time, rad2deg(smc.log.theta_log), 'g', 'LineWidth', lw);
hold on;
plot(time, rad2deg(indi.log.theta_log), 'b', 'LineWidth', lw);
hold on;
plot(time, rad2deg(pid.log.theta_des_log), 'k--', 'LineWidth', 0.5);
xlabel('Time (s)'); ylabel('Pitch (°)');
legend('PID','SMC','INDI');

nexttile;
plot(time, rad2deg(pid.log.psi_log), 'r', 'LineWidth', lw);
hold on;
plot(time, rad2deg(smc.log.psi_log), 'g', 'LineWidth', lw);
hold on;
plot(time, rad2deg(indi.log.psi_log), 'b', 'LineWidth', lw);
hold on;
plot(time, rad2deg(pid.log.psi_des_log), 'k--', 'LineWidth', 0.5);
xlabel('Time (s)'); ylabel('Yaw (°)');
legend('PID','SMC','INDI');

set(findobj(gcf,'type','axes'),'FontName', 'Arial', 'FontSize', 12);
arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
exportgraphics(gcf, 'results.png', 'Resolution', 300)
