function M = reshape_stiffness_back(v)
M = [v(1, 1), v(4, 1), v(7, 1);
     v(2, 1), v(5, 1), v(8, 1)
     v(3, 1), v(6, 1), v(9, 1)];
end