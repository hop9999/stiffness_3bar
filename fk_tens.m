function q = fk_tens(robot)
x0 = [get_node(robot, 4)',get_node(robot, 5)', get_node(robot, 6)']
[q,~] = fminunc(@(q)potens_energy_numerical(q,robot),x0);
end