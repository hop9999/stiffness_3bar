load results_stiffness_design.mat

fig = figure('Color', 'w');
fig.Name = "l";
 hold on;

plot(res.delta_S, res.S_real(1,:), 'LineWidth', 1, 'LineStyle', '-', 'Color', [255. 0 0]/255.);
plot(res.delta_S, res.S_real(2,:), 'LineWidth', 1, 'LineStyle', '-', 'Color', [20. 128. 0]/255.);
plot(res.delta_S, res.S_real(3,:), 'LineWidth', 1, 'LineStyle', '-', 'Color', [0. 0. 128]/255.);

plot(res.delta_S, res.S_desired(1,:), 'LineWidth', 3, 'LineStyle', ':', 'Color', [128. 128. 0]/255.);
plot(res.delta_S, res.S_desired(2,:), 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'y');
plot(res.delta_S, res.S_desired(3,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', [0 255. 0.]/255.);
grid on; grid minor;

ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
% ax.FontName = 'Tibetan Machine Uni';
ax.FontName = 'Times New Roman';
ax.FontSize = 16;

xlabel_handle = xlabel('$$\delta_k$$, (N/m)');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$K$$, (N/m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend({'$$K_{1,1}$$','$$K_{2,2}$$','$$K_{3,3}$$', '$$K_{1,1}^{*}$$','$$K_{2,2}^{*}$$','$$K_{3,3}^{*}$$'});
legend_handle.Interpreter = 'latex';













fig = figure('Color', 'w');
fig.Name = "l";
 hold on;

plot( res.delta_S, abs(res.S_desired(1,:) - res.S_real(1,:)), 'LineWidth', 1, 'LineStyle', '-', 'Color', [255. 0 0]/255.);
plot( res.delta_S, abs(res.S_desired(2,:) - res.S_real(2,:)), 'LineWidth', 2, 'LineStyle', '--', 'Color', [20. 128. 0]/255.);
plot( res.delta_S, abs(res.S_desired(3,:) - res.S_real(3,:)), 'LineWidth', 3, 'LineStyle', ':', 'Color', [0. 0. 128]/255.);

% plot( s_des_ar(1,:), 'LineWidth', 3, 'LineStyle', ':', 'Color', [128. 128. 0]/255.);
% plot( s_des_ar(2,:), 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'y');
% plot( s_des_ar(3,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', [0 255. 0.]/255.);
grid on; grid minor;

ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
% ax.FontName = 'Tibetan Machine Uni';
ax.FontName = 'Times New Roman';
ax.FontSize = 16;
ax.YScale = 'log';

xlabel_handle = xlabel('$$\delta_k$$, (N/m)');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$K$$, (N/m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend({'$$|K_{1,1}^{*} - K_{1,1}|$$', ...
                        '$$|K_{2,2}^{*} - K_{2,2}|$$', ...
                        '$$|K_{3,3}^{*} - K_{3,3}|$$'});
legend_handle.Interpreter = 'latex';
