function [real_S, S] = stiffness_design_node(dS, robot)
% dS = [1 0 0
%       0 1 0 
%       0 0 1]*10;
  
%get constant robot parameters
% robot = get_robot();
k  = robot.k;

%get linearization point 
% r_init = [robot.initial_state(1); robot.initial_state(2)];
r_init = robot.r;%get_node(robot,6);%robot.r(:);
ro_init =  robot.ro;%[robot.ro(5),robot.ro(6),robot.ro(3),robot.ro(11)]';


con = [4 1 7
       4 2 11
       4 5 4
       4 6 6
       5 2 8
       5 3 12
       5 4 4
       5 6 5
       6 1 10
       6 3 9
       6 4 6
       6 5 5];
       
       
S0 = g_S(r_init, ro_init);

S_desired = S0 + dS;

vS_desired = reshape_stiffness(S_desired);

%calculate linearization normals
s = size(con);
normal = zeros(3,s(1));
for i = 1:s(1)
    normal(:,i) = (get_node(robot,con(i,1)) - get_node(robot,con(i,2)))/...
        norm(get_node(robot,con(i,1))-get_node(robot,con(i,2)));
end

% for i = 1:s(1)
%     f_norm(:,i) = -k(con(i,3))*((r_init(:,con(i,1)) - r_init(:,con(i,2))) - ro_init(con(i,3))*normal(:,i));
% end
% 
% for i = 1:s(1)
%     f_tang(:,i) = -k(con(i,3))*(dot(normal(:,i), (r_init(:,con(i,1)) - r_init(:,con(i,2)))) - ro_init(con(i,3)))*normal(:,i);
% end
% 
% for i = 1:s(1)
%     f_res(:,i) = (f_norm(:,i) + f_tang(:,i))/2;
% end
% 
% f_res(:,1) + f_res(:,2) + f_res(:,3) + f_res(:,4)
% 
% f_res(:,5) + f_res(:,6) + f_res(:,7) + f_res(:,8)
% 
% f_res(:,9) + f_res(:,10) + f_res(:,11) + f_res(:,12)
% return


%stiffness
tensor_K_r  = g_tensor_K_r(r_init, ro_init);
tensor_K_ro = g_tensor_K_ro(r_init, ro_init);
vS_0        = g_vS_0(r_init, ro_init);

%r_init_full = robot.r;
%ro_init_full = robot.ro';

cvx_begin
    variable delta_r(size(r_init)) %point position
    variable delta_ro(size(ro_init)) %spring resl lengths changes
    
    r = delta_r + r_init;
    ro = delta_ro + ro_init;

    %forces
    %f_norm = zeros(3,s(1));
    for i = 1:s(1)
        f_norm(:,i) = -k(con(i,3))*((r_init(:,con(i,1)) - r_init(:,con(i,2))) - ro_init(con(i,3))*normal(:,i));
    end

    for i = 1:s(1)
        f_tang(:,i) = -k(con(i,3))*(dot(normal(:,i), (r_init(:,con(i,1)) - r_init(:,con(i,2)))) - ro_init(con(i,3)))*normal(:,i);
    end

    for i = 1:s(1)
        f_res(:,i) = (f_norm(:,i) + f_tang(:,i))/2;
    end

    %stiffness
    vS = tensor_K_r*r(:) + tensor_K_ro*ro + vS_0;
    
%     minimize( norm(vS - vS_desired) )
    minimize( (vS(1)-vS_desired(1))^2 + (vS(5)-vS_desired(5))^2 + (vS(9)-vS_desired(9))^2)
    subject to
        %f_res(:,1) + f_res(:,3) + f_res(:,7) + f_res(:,10) == zeros(3, 1);
        %f_res(:,1) + f_res(:,2) + f_res(:,8) + f_res(:,11) == zeros(3, 1);
        %f_res(:,2) + f_res(:,3) + f_res(:,9) + f_res(:,12) == zeros(3, 1);
        f_res(:,1) + f_res(:,2) + f_res(:,3) + f_res(:,4) == zeros(3, 1);
        f_res(:,5) + f_res(:,6) + f_res(:,7) + f_res(:,8) == zeros(3, 1);
        f_res(:,9) + f_res(:,10) + f_res(:,11) + f_res(:,12) == zeros(3, 1);
        ro(10) == 2.0;
        ro(11) == 2.0;
        ro(12) == 2.0;
        r(:,1) == r_init(:,1);
        r(:,2) == r_init(:,2);
        r(:,3) == r_init(:,3);
cvx_end

r - r_init
delta_ro'

S = reshape_stiffness_back(vS);
real_S = g_S(r, ro);

