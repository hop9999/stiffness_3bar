function r = fk_tens(robot)
x0 = [get_node(robot, 4)'+0.1,get_node(robot, 5)'+0.1, get_node(robot, 6)'+0.1]
[q,~] = fminunc(@(q)potens_energy_numerical(q,robot),x0);
reshape(q,3,3)
r = robot.r;
r(:,4:end) = reshape(q,3,3);

end