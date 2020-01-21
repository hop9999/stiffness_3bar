close all; clear; clc;

ro_radius = 0.2;
max_number_of_fk_iterations = 100;
Count = 3;

force_error_threshold = 0.05;

robot = get_robot();
ro_center =  robot.ro;
r_init = robot.r;

result_r = r_init(:, 4:6);
Result.r = zeros(Count, length(result_r(:)));
Result.ro = zeros(Count, length(ro_center));
Result.error = zeros(Count);

index = 1;
for i = 1:Count
    
    ro = ro_center + ro_radius* 2 * (rand(size(ro_center)) - 0.5*ones(size(ro_center)));
    
    r = r_init;
    error = inf;
    j = 1;
    while (j < max_number_of_fk_iterations) && (error > force_error_threshold)
        disp(['calculating ', num2str(j), ...
            '; EXPERIMENT ', num2str(i), ' out of ', num2str(Count)]);
        r = ML_solve_forward_kinematics(r, ro);
        
        f = ML_get_sum_node_forces(r, ro, robot.k);
        error = norm(f(:, 1)) + norm(f(:, 2)) + norm(f(:, 3));
        j = j + 1;
    end
    
    if error < force_error_threshold
        result_r = r(:, 4:6);
        Result.r(index, :) = result_r(:);
        Result.ro(index, :) = ro;
        Result.error(index) = error;
        index = index + 1;
    end
end
index = index - 1;

Result.r = Result.r(1:index, :);
Result.ro = Result.ro(1:index, :);
Result.error = Result.error(1:index);



