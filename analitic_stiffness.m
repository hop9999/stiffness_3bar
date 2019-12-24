clc;
clear;
close all;

robot = get_robot();

r  = sym('r', [3, 6]); assume(r, 'real');
ro = sym('ro',[12, 1]); assume(ro, 'real');
k  = robot.k;

%cable# 5 ---> ro(1)
%cable# 6 ---> ro(2)
%cable# 3 ---> ro(3)
%cable# ? ---> ro(4)
%cable# ? ---> ro(5)
%cable# ? ---> ro(6)

% Stiffness = k(1)/(norm(r)^3) * ...
% ( norm(r) * (r*r') + (norm(r) - ro(1)) * (norm(r)^2 *eye(2) - r*r'));

S1 = getStiffness(r(:, 6), r(:, 5), k(5), ro(5))
S2 = getStiffness(r(:, 6), r(:, 4), k(6), ro(6))
S3 = getStiffness(r(:, 6), r(:, 3), k(9), ro(9))
S4 = getStiffness(r(:, 6), r(:, 1), k(11), ro(11))

S = S1 + S2 + S3 + S4

%S = simplify(S);

vS = reshape_stiffness(S);

tensor_K_r  = jacobian(vS, r(:));
tensor_K_ro = jacobian(vS, ro);

%tensor_K_r  = simplify(tensor_K_r);
%tensor_K_ro = simplify(tensor_K_ro);

vS_0 = vS - tensor_K_r*r(:) - tensor_K_ro*ro;
%vS_0 = simplify(vS_0);

matlabFunction(S,           'File', 'g_S',            'Vars', {r, ro});
matlabFunction(tensor_K_r,  'File', 'g_tensor_K_r',   'Vars', {r, ro});
matlabFunction(tensor_K_ro, 'File', 'g_tensor_K_ro',  'Vars', {r, ro});
matlabFunction(vS_0,        'File', 'g_vS_0',         'Vars', {r, ro});

function S = getStiffness(r, r1, k, ro)

x = r1 - r;

S = k/(norm(x)^3) * ...
( norm(x) * (x*x') + (norm(x) - ro) * (norm(x)^2 *eye(3) - x*x') );
%S = simplify(S);
end


