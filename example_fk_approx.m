clear; clc; close all;

robot = get_robot();
disp("get_forces_sum(r_init, robot): ")   
get_forces_sum(robot.r, robot)

disp('***********************')
disp('***********************')
disp('***********************')
disp('******* ROUND 1 *******')
r = fk_approx(robot.r);

for i = 2:50
    disp('***********************')
    disp('***********************')
    disp('***********************')
    disp(['******* ROUND ', num2str(i), ' *******'])
    r = fk_approx(r);
    disp("get_forces_sum(r, robot):")
    err(i) = norm(get_forces_sum(r, robot));
end

fig = figure('Color', 'w');
fig.Name = "l";
 hold on;

plot( err(2:end), 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k');

grid on; grid minor;

ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
% ax.FontName = 'Tibetan Machine Uni';
ax.FontName = 'Times New Roman';
ax.FontSize = 16;

xlabel_handle = xlabel('$$n$$');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$||e_f||$$');
ylabel_handle.Interpreter = 'latex';
% legend_handle = legend('$$force\ error$$');
% legend_handle.Interpreter = 'latex';
