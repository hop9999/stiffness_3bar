clear; clc; close all;

robot = get_robot();
r_init = robot.r;
ro_init =  robot.ro;
S0 = g_S(r_init, ro_init)

ro_array = [0.3 0.4 0.5 0.6];
[ro3_des,ro5_des] = meshgrid(ro_array);
s_norm_err = zeros(size(ro3_des));
f_err_arr = zeros(size(ro3_des));

for j = 1:length(ro_array)
    for k = 1:length(ro_array)
        j
        k
        robot.ro(3) = ro3_des(j,k);
        robot.ro(5) = ro5_des(j,k);

        S_des = [55.0  0  0
                 0  50.0  0
                 0  0  45.0];
        real_S = S0;
        for i = 1:50
            er_S = S_des - real_S;
            [real_S, ~, r, ro] = stiffness_design_node(er_S, robot);
            real_S;
            robot.r = r;
            robot.ro = ro;
            get_forces_sum(r, robot)
            
        end
        s_norm_err(j,k) = norm(diag(real_S)- diag(S_des));
        f_err_arr(j,k) = norm(get_forces_sum(r, robot));
    end
end

s_norm_err
f_err_arr
fig = figure('Color','w');
s = surface(ro3_des,ro5_des,f_err_arr)
colormap(autumn(5))
s.EdgeColor = 'none';
xlabel('')
xlabel_handle = xlabel('$$\rho_3$$');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\rho_5$$');
ylabel_handle.Interpreter = 'latex';
zlabel_handle = zlabel('$$||error||$$');
zlabel_handle.Interpreter = 'latex';
legend_handle = legend('force ballance error');
legend_handle.Interpreter = 'latex';
axis equal
light('Position',[0.2 -1 0.0],'Style','local');
light('Position',[0.2 -1 1.5],'Style','local');
light('Position',[0.2 1 1.5],'Style','local');
axis equal


fig = figure('Color','w');
s = surface(ro3_des,ro5_des,s_norm_err)
colormap(winter(5))
s.EdgeColor = 'none';
xlabel('')
xlabel_handle = xlabel('$$\rho_3$$');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\rho_5$$');
ylabel_handle.Interpreter = 'latex';
zlabel_handle = zlabel('$$||error||$$');
zlabel_handle.Interpreter = 'latex';
legend_handle = legend('stiffness design error');
legend_handle.Interpreter = 'latex';
axis equal
light('Position',[0.2 -1 0.0],'Style','local');
light('Position',[0.2 -1 1.5],'Style','local');
light('Position',[0.2 1 1.5],'Style','local');
axis equal