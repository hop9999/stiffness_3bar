close all;

r_init = [2; 0];
r1 = [0; 0];

k = 1;
ro = 1;

% get_force_appr = get_force_normal_approx(r_init, r1);
% get_force_appr = get_force_tang_approx(r_init, r1);
get_force_appr = get_force_comb_approx(r_init, r1);


Count = 100; 
range = 4;
r_step = 2*range / Count;
r_min = [-range; -range];

X = zeros(Count, Count);
Y = zeros(Count, Count);
error_array = zeros(Count, Count);
error_arrayX = zeros(Count, Count);
error_arrayY = zeros(Count, Count);

for i = 1:Count
    for j = 1:Count
        
        r = r_min + [i*r_step; j*r_step];
        f = get_force_exact(r, r1, k, ro);
        fn = get_force_appr(r, r1, k, ro);
        
        error_array(i, j) = norm(f - fn);
        error_arrayX(i, j) = f(1) - fn(1);
        error_arrayY(i, j) = f(2) - fn(2);
        X(i, j) = r(1);
        Y(i, j) = r(2);
    end
end

h = 2;

phi_step = 2*pi / Count;
circle = zeros(Count, 3);
for i = 1:Count
    phi = i*phi_step;
    p = r1 + [ro*cos(phi); ro*sin(phi)];
    circle(i, :) = [p', h];
end

figure('Color', 'w');
subplot(1, 2, 1)
surf(X, Y, error_array, 'EdgeAlpha', 0.1); hold on;
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
ax.CLim = [0 2.5];
xlabel_handle = xlabel('$$x$$, m', 'Interpreter', 'latex');
ylabel_handle = ylabel('$$y$$, m', 'Interpreter', 'latex');
zlabel_handle = zlabel('$$z$$, m', 'Interpreter', 'latex');
axis equal;
% colorbar;

% figure('Color', 'w');
subplot(1, 2, 2)
contourf(X, Y, error_array); hold on;
plot3(circle(:, 1), circle(:, 2), circle(:, 3), 'Color', 'r', 'LineWidth', 5);
plot3(r_init(1), r_init(2), h, 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 15);
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
ax.CLim = [0 2.5];
xlabel_handle = xlabel('$$x$$, m', 'Interpreter', 'latex');
ylabel_handle = ylabel('$$y$$, m', 'Interpreter', 'latex');
axis equal;
colorbar;

% 
% figure;
% surf(X, Y, error_arrayX); hold on;
% plot3(circle(:, 1), circle(:, 2), circle(:, 3), 'Color', 'r', 'LineWidth', 5);
% plot3(r_init(1), r_init(2), h, 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 15);
% title("X")
% axis equal;
% 
% 
% figure;
% surf(X, Y, error_arrayY); hold on;
% plot3(circle(:, 1), circle(:, 2), circle(:, 3), 'Color', 'r', 'LineWidth', 5);
% plot3(r_init(1), r_init(2), h, 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 15);
% title("Y")
% axis equal;
% 
