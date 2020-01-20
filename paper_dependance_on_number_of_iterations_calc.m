clear; clc; close all;

robot = get_robot();
r_init = robot.r;
ro_init =  robot.ro;

S0 = g_S(r_init, ro_init);

delta_s = 5;
iteration_steps = 100;

res.S_desired = zeros(3, iteration_steps);
res.S_real    = zeros(3, iteration_steps);
res.forces_error   = zeros(1, iteration_steps);

real_S = S0;
dS = [1  0  0
    0  1  0
    0  0  1]*delta_s;
S_des = real_S + dS;

for i = 1:iteration_steps
    disp(['iteration #', num2str(i)]);
    
    er_S = S_des - real_S;
    [real_S, ~, r, ro] = stiffness_design_node(er_S, robot);
    %         [real_S, ~, r, ro] = stiffness_design_node9(er_S, robot);
    real_S;
    robot.r = r;
    robot.ro = ro;
    
    res.S_desired(:,i) = diag(S_des);
    res.S_real(:,i)    = diag(real_S);
    res.forces_error(:,i) = norm(get_forces_sum(r, robot));
end

save('results_dependance_on_number_of_iterations', 'res');