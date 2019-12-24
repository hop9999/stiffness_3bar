clear; clc; close all;

robot = get_robot();
r_init = robot.r;
ro_init =  robot.ro;
S0 = g_S(r_init, ro_init)

n = 7;
s_des_ar = zeros(3,n);
s_real_ar = zeros(3,n);
f_err_arr = zeros(1,n);

for j = 1:n
    real_S = [55.0  0  0
              0  50.0  0
              0  0  45.0];
    dS = [1  0  0
          0  1  0
          0  0  1]*(j-1-(n-1)/2);
    S_des = real_S + dS;
    for i = 1:50
        er_S = S_des - real_S
        [real_S, ~, r, ro] = stiffness_design_node(er_S, robot);
        real_S;
        robot.r = r;
        robot.ro = ro;
        get_forces_sum(r, robot)

    end
    s_des_ar(:,j) = diag(S_des);
    s_real_ar(:,j) = diag(real_S);
    f_err_arr(:,j) = norm(get_forces_sum(r, robot));
end

s_des_ar
s_real_ar
f_err_arr