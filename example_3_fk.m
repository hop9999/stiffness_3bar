clc;
clear;
close all;

robot = get_robot();

q = fk_tens(robot)
reshape(q,3,3)
r1 = get_node(robot, 1);
r2 = get_node(robot, 2);
r3 = get_node(robot, 3);
r4 = q(1:3)';
r5 = q(4:6)';
r6 = q(7:9)';
