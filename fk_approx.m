% clear; clc; close all;
function r = fk_approx(r_init)
robot = get_robot();

k  = robot.k;

%get linearization point 
% r_init = [robot.initial_state(1); robot.initial_state(2)];
% r_init = robot.r;
ro_init =  robot.ro;

con = get_connectivity();
      

%calculate linearization normals
s = size(con);
normal = get_normal(r_init);

cvx_begin quiet
    variable delta_r(size(r_init)) %point position
    
    r = delta_r + r_init;
    ro = ro_init;

    %forces
    %f_norm = zeros(3,s(1));
    for i = 1:s(1)
        f_norm(:,i) = -k(con(i,3))*((r(:,con(i,1)) - r(:,con(i,2))) - ro(con(i,3))*normal(:,i));
    end

    for i = 1:s(1)
        f_tang(:,i) = -k(con(i,3))*(dot(normal(:,i), (r(:,con(i,1)) - r(:,con(i,2)))) - ro(con(i,3)))*normal(:,i);
    end

    for i = 1:s(1)
        f_res(:,i) = (f_norm(:,i) + f_tang(:,i))/2;
    end
    
    f1 = f_res(:,1) + f_res(:,2) + f_res(:,3) + f_res(:,4);
    f2 = f_res(:,5) + f_res(:,6) + f_res(:,7) + f_res(:,8);
    f3 = f_res(:,9) + f_res(:,10) + f_res(:,11) + f_res(:,12);
    
    minimize( 1 )
    subject to
        f1 == zeros(3, 1);
        f2 == zeros(3, 1);
        f3 == zeros(3, 1);
        r(:,1) == r_init(:,1);
        r(:,2) == r_init(:,2);
        r(:,3) == r_init(:,3);
cvx_end

robot.ro = ro;

% f1
% f2
% f3

% disp("r:")
% r
% 
% disp("ro:")
% ro'


% disp("get_forces_sum_approx(r, robot):")
% get_forces_sum_approx(r, robot, normal)


% disp("get_forces_sum_approx(r, robot), new normals:")
% normal = get_normal(r);
% get_forces_sum_approx(r, robot, normal)


% disp("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
% disp("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
% disp("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
% disp("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
% disp("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
% disp("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
% disp("get_forces_sum(r, robot):")
% get_forces_sum(r, robot)
end