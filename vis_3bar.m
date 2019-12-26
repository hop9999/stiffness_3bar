clc;
clear;
close all;

robot = get_robot();
tr = [robot.r(:,1), robot.r(:,4), robot.r(:,2), robot.r(:,5), robot.r(:,3), robot.r(:,6)];
t = tr(:)';
rods = {t(1:6) t(7:12) t(13:18)};
fig = figure('Color','w');
visualize_rods_stl(rods, {[0 0.4470 0.7410],[0.5 0.4470 0.0],[0.5 0.0 0.50]},1);
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
visualize_springs(points,{connect},{[0 0 0]},{1,0.5},{0.6,0.2})

light('Position',[0 0 0.5],'Style','local');
light('Position',[-2 -2 1.5],'Style','local');
light('Position',[2 2 1.5],'Style','local');