clear; clc; close all;

robot = get_robot();
r_init = robot.r;
ro_init =  robot.ro;

S0 = g_S(r_init, ro_init);

delta_s_steps = 30;
delta_s_max = 5;
delta_s_min = -5;
delta_s_step = (delta_s_max - delta_s_min) / delta_s_steps;

res.S_desired = zeros(3, delta_s_steps);
res.S_real    = zeros(3, delta_s_steps);
res.delta_S   = zeros(1, delta_s_steps);
res.forces_error   = zeros(1, delta_s_steps);

for j = 1:delta_s_steps
    real_S = S0;
    delta_s = delta_s_min + delta_s_step * (j - 1);
    dS = [1  0  0
          0  1  0
          0  0  1]*delta_s;
    S_des = real_S + dS;
    
    for i = 1:10
        disp(['iteration #', num2str(i), ' experiment #', num2str(j)]);
        
        er_S = S_des - real_S;
        [real_S, ~, r, ro] = stiffness_design_node(er_S, robot);
%         [real_S, ~, r, ro] = stiffness_design_node9(er_S, robot);
        real_S;
        robot.r = r;
        robot.ro = ro;
%         get_forces_sum(r, robot)

    end
    res.S_desired(:,j) = diag(S_des);
    res.S_real(:,j)    = diag(real_S);
    res.delta_S(:,j) = delta_s;
    res.forces_error(:,j) = norm(get_forces_sum(r, robot));
end

save('results_stiffness_design', 'res');