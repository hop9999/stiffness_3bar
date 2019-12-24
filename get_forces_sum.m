function [f] = get_forces_sum(r, robot)

f4 = get_force_exact(r(:,4), r(:,5), robot.k(4), robot.ro(4)) +...
     get_force_exact(r(:,4), r(:,6), robot.k(6), robot.ro(6)) +...
     get_force_exact(r(:,4), r(:,1), robot.k(7), robot.ro(7)) +...
     get_force_exact(r(:,4), r(:,2), robot.k(11), robot.ro(11));

f5 = get_force_exact(r(:,5), r(:,4), robot.k(4), robot.ro(4)) +...
     get_force_exact(r(:,5), r(:,6), robot.k(5), robot.ro(5)) +...
     get_force_exact(r(:,5), r(:,2), robot.k(8), robot.ro(8)) +...
     get_force_exact(r(:,5), r(:,3), robot.k(12), robot.ro(12));

f6 = get_force_exact(r(:,6), r(:,5), robot.k(5), robot.ro(5)) +...
     get_force_exact(r(:,6), r(:,4), robot.k(6), robot.ro(6)) +...
     get_force_exact(r(:,6), r(:,3), robot.k(9), robot.ro(9)) +...
     get_force_exact(r(:,6), r(:,1), robot.k(10), robot.ro(10));

f = [f4 f5 f6];


