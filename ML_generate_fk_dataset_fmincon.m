close all; clear; clc;

ro_radius = 0.2;
Count = 500000;

force_error_threshold = 0.05;

robot = get_robot();
ro_center =  robot.ro;
r_init = robot.r;
k = robot.k;

result_r = r_init(:, 4:6);
Result.r = zeros(Count, length(result_r(:)));
Result.ro = zeros(Count, length(ro_center));
Result.error = zeros(Count, 1);

index = 1;
for i = 1:Count
    disp(['calculating ', num2str(i), ' out of ', num2str(Count)]);

    ro = ro_center + ro_radius* 2 * (rand(size(ro_center)) - 0.5*ones(size(ro_center)));
    
    r = ML_fk_tens(r_init, ro, k);
    f = ML_get_sum_node_forces(r, ro, k);
    error = norm(f(:, 1)) + norm(f(:, 2)) + norm(f(:, 3));
    
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

save('data_ML_fmincon_dataset_500k', 'Result', 'ro_radius', 'force_error_threshold')

