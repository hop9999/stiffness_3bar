clear; clc; close all;

load track.mat

fig = figure('Color', 'w');
fig.Name = "l";
 hold on;

plot( s_real_ar(1,:), 'LineWidth', 2, 'LineStyle', '-', 'Color', [255. 0 0]/255.);
plot( s_real_ar(2,:), 'LineWidth', 2, 'LineStyle', '-', 'Color', [255. 128. 0]/255.);
plot( s_real_ar(3,:), 'LineWidth', 2, 'LineStyle', '-', 'Color', [0. 0. 128]/255.);

plot( s_des_ar(1,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', [128. 128. 0]/255.);
plot( s_des_ar(2,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', [0 255. 128]/255.);
plot( s_des_ar(3,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', [0 255. 0.]/255.);
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
ylabel_handle = ylabel('$$K$$, (N/m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$K(1,1)$$','$$K(2,2)$$','$$K(3,3)$$','$$K_{des}(1,1)$$','$$K_{des}(2,2)$$','$$K_{des}(3,3)$$');
legend_handle.Interpreter = 'latex';


fig = figure('Color', 'w');
fig.Name = "err";
 hold on;
grid on; grid minor;
plot( f_err_arr, 'LineWidth', 2, 'LineStyle', '-', 'Color', [0 0.5 0]);
plot( f_err_arr, 'o', 'MarkerEdgeColor', [0 0.5 0]);

ax = gca;
% ax.FontName = 'Tibetan Machine Uni';
ax.FontName = 'Times New Roman';
ax.FontSize = 16;

xlabel_handle = xlabel('$$n$$');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\varepsilon$$, N');
ylabel_handle.Interpreter = 'latex';
% legend_handle = legend('$$force\ error$$');
% legend_handle.Interpreter = 'latex';
