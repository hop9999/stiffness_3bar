clc;
clear;
close all;

robot = get_robot();
tr = [robot.r(:,1), robot.r(:,4), robot.r(:,2), robot.r(:,5), robot.r(:,3), robot.r(:,6)];
t = tr(:)';
rods = {t(1:6) t(7:12) t(13:18)};

points = robot.r'
connect = [1,2
    2,3
    3,1
    4,5
    5,6
    6,4
    1,4
    2,5
    3,6
    1,6
    2,4
    3,5];

figure('Color', 'w');
scale = diag([0.2 0.2 29]);
visualize_rods_stl(rods, {[0.2 0.7 0.3], ...
                          [0.4 0.1 0.5], ...
                          [0.5 0.5 0.20]}, ...
                          1, scale);
visualize_springs(points,{connect},{[0 0 0]},{1,0.5},{0.6,0.2})

campos('manual')
campos([-1,-1,1])

[sphere_x,sphere_y,sphere_z] = sphere;
radius = 0.1;
for i = 1:6
    sphere_xi = sphere_x*radius + robot.r(1,i);
    sphere_yi = sphere_y*radius + robot.r(2,i);
    sphere_zi = sphere_z*radius + robot.r(3,i);
    
    surf(sphere_xi,sphere_yi,sphere_zi, 'EdgeAlpha', 0, 'FaceColor', [0.3 0.2 1], 'SpecularStrength', 0.2)
end


axis equal;
ax = gca;
ax.Visible = 'off';

light('Position',[0 0 0.5],'Style','local');
light('Position',[-2 -2 1.5],'Style','local');
light('Position',[2 2 1.5],'Style','local');

drawnow;

figure('Color', 'w');

visualize_rods_stl(rods, {[1 0 0], ...
                          [0 0.5 0.0], ...
                          [0.0 0.0 0.50]}, ...
                          1, scale);
visualize_springs(points,{connect},{[0 0 0]},{1,0.5},{0.6,0.2})
grid on; grid minor;

[sphere_x,sphere_y,sphere_z] = sphere;
radius = 0.1;
for i = 1:6
    sphere_xi = sphere_x*radius + robot.r(1,i);
    sphere_yi = sphere_y*radius + robot.r(2,i);
    sphere_zi = sphere_z*radius + robot.r(3,i);
    
    surf(sphere_xi,sphere_yi,sphere_zi, 'EdgeAlpha', 0, 'FaceColor', [0.5 0 0.5], 'SpecularStrength', 0.2)
end

campos('manual')
campos([-1,-1,1])
axis equal;

ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 14;
xlabel_handle = xlabel('$$x$$, m', 'Interpreter', 'latex');
ylabel_handle = ylabel('$$y$$, m', 'Interpreter', 'latex');
zlabel_handle = zlabel('$$z$$, m', 'Interpreter', 'latex');

light('Position',[0 0 0.5],'Style','local');
light('Position',[-2 -2 1.5],'Style','local');
light('Position',[2 2 1.5],'Style','local');