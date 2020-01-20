function [real_S, optim_S, r, ro] = stiffness_design_node9(dS, robot)

k  = robot.k;
r_init = robot.r;
ro_init =  robot.ro;

n0 = zeros(3, 4);
n0(:, 1) = r_init(:, 5) - r_init(:, 6);
n0(:, 2) = r_init(:, 4) - r_init(:, 6);
n0(:, 3) = r_init(:, 3) - r_init(:, 6);
n0(:, 4) = r_init(:, 1) - r_init(:, 6);

con = get_connectivity();
      
%calculate linearization normals
s = size(con);
normal = get_normal(r_init);

S0 = g_S(r_init, ro_init);

S_desired = S0 + dS;

vS_desired = reshape_stiffness(S_desired);

%stiffness
tensor_K_r  = g_tensor_K_r(r_init, ro_init);
tensor_K_ro = g_tensor_K_ro(r_init, ro_init);
vS_0        = g_vS_0(r_init, ro_init);


cvx_begin quiet
    variable delta_r(size(r_init)) %point position
    variable delta_ro(size(ro_init)) %spring resl lengths changes
    
    r = delta_r + r_init;
    ro = delta_ro + ro_init;

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

    %stiffness
    vS = tensor_K_r*r(:) + tensor_K_ro*ro + vS_0;

    f1 = f_res(:,1) + f_res(:,2) + f_res(:,3) + f_res(:,4);
    f2 = f_res(:,5) + f_res(:,6) + f_res(:,7) + f_res(:,8);
    f3 = f_res(:,9) + f_res(:,10) + f_res(:,11) + f_res(:,12);
    
    minimize( norm(vS(:)-vS_desired(:)))
    subject to
        f1 == zeros(3, 1);
        f2 == zeros(3, 1);
        f3 == zeros(3, 1);
        r(:,1) == r_init(:,1);
        r(:,2) == r_init(:,2);
        r(:,3) == r_init(:,3);
        ro(10) == 2;
        ro(11) == 2;
        ro(12) == 2;
cvx_end

optim_S = reshape_stiffness_back(vS);
real_S = g_S(r, ro);
% 1
