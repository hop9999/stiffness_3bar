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
end