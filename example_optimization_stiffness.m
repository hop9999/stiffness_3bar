clear; clc; close all;

%get constant robot parameters
robot = get_robot();
r_init = get_node(robot,6);%robot.r(:);
ro_init = [robot.ro(5),robot.ro(6),robot.ro(3),robot.ro(11)]';
S0 = g_S(r_init, ro_init)

s0_ar = [1,20];
real_s_ar = [1,20];
optim_s_ar = [1,20];

for i = 1:1
    dS = [1 0 0
      0 1 0 
      0 0 1]*5;
    [real_S, S] = stiffness_design(dS, robot)
    s0_ar(i) = S0(1,1) + dS(1,1);
    real_s_ar(i) = real_S(1,1);
    optim_s_ar(i) = S(1,1);
end

fig = figure('Color', 'w');
fig.Name = "l";
plot( s0_ar, 'LineWidth', 1, 'LineStyle', '-', 'Color', 'r'); hold on;
plot( real_s_ar, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'b');
plot( optim_s_ar, 'LineWidth', 3, 'LineStyle', ':', 'Color', 'k');
plot( s0_ar-real_s_ar, 'LineWidth', 2, 'LineStyle', '-.', 'Color', [0.8 0.2 0.2]); 
grid on; grid minor;

ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Tibetan Machine Uni';
ax.FontSize = 16;


xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\bar q_i$$ (m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$\bar q_1$$ (m)', '$$\bar q_2$$ (m)', '$$\bar q_3$$ (m)', '$$\bar q_4$$ (m)');
legend_handle.Interpreter = 'latex';
