clear; clc; close all;

robot = get_robot();
r_init = robot.r;
ro_init =  robot.ro;
S0 = g_S(r_init, ro_init)

n = 1;
s_des_ar = zeros(3,n);
s_real_ar = zeros(3,n);
s_norm_err = zeros(1,n);
f_err_arr = zeros(1,n);
ro_des = [0.4 0.6];
for j = ro_des
    for k = ro_des
        robot.ro(3) = j;
        robot.ro(5) = k;

        S_des = [55.0  0  0
                 0  50.0  0
                 0  0  45.0];
        real_S = S0;
        for i = 1:10
            er_S = S_des - real_S
            [real_S, ~, r, ro] = stiffness_design_node(er_S, robot);
            real_S;
            robot.r = r;
            robot.ro = ro;
            get_forces_sum(r, robot)
            
        end
        s_norm_err(:,n) = norm(diag(real_S)- diag(S_des));
        f_err_arr(:,n) = norm(get_forces_sum(r, robot));
        n = n + 1;
    end
end

s_norm_err
f_err_arr