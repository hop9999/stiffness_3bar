clear; clc; close all;

%get constant robot parameters
robot = get_robot();
r_init = robot.r;%get_node(robot,6);%robot.r(:);
ro_init =  robot.ro;%[robot.ro(5),robot.ro(6),robot.ro(3),robot.ro(11)]';
S0 = g_S(r_init, ro_init)

s0_ar = zeros(3,20);
real_s_ar = zeros(3,20);
optim_s_ar = zeros(3,20);

for i = 1:20
    dS = [1 0 0
          0 1 0 
          0 0 1]*(i-10);
    [real_S, optim_S] = stiffness_design(dS, robot);
    diag(S0) + diag(dS)
    s0_ar(:,i) = diag(S0) + diag(dS);
    real_s_ar(:,i) = diag(real_S);
    optim_s_ar(:,i) = diag(optim_S);
end

optim_S
S0 + dS
real_S

fig = figure('Color', 'w');
fig.Name = "l";
 hold on;

plot( real_s_ar(1,:), 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'r');
plot( real_s_ar(2,:), 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
plot( real_s_ar(3,:), 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'b');

plot( optim_s_ar(1,:), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'r');
plot( optim_s_ar(2,:), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
plot( optim_s_ar(3,:), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'b');

plot( s0_ar(1,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', 'r');
plot( s0_ar(2,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
plot( s0_ar(3,:), 'LineWidth', 2, 'LineStyle', '--', 'Color', 'b');

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
