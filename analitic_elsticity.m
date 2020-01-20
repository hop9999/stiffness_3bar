clc;
clear;
close all;

robot = get_robot();

r  = sym('r', [3, 6]);   assume(r, 'real');
ro = sym('ro',[12, 1]);  assume(ro, 'real');
k  = robot.k;

%cable# 5 ---> ro(1)
%cable# 6 ---> ro(2)
%cable# 3 ---> ro(3)
%cable# ? ---> ro(4)
%cable# ? ---> ro(5)
%cable# ? ---> ro(6)

% Stiffness = k(1)/(norm(r)^3) * ...
% ( norm(r) * (r*r') + (norm(r) - ro(1)) * (norm(r)^2 *eye(2) - r*r'));


f11 = get_elastic_force(r(:, 6), r(:, 5), k(5), ro(5));
f12 = get_elastic_force(r(:, 6), r(:, 4), k(6), ro(6));
f13 = get_elastic_force(r(:, 6), r(:, 3), k(9), ro(9));
f14 = get_elastic_force(r(:, 6), r(:, 1), k(11), ro(11));
f1 = f11+f12+f13+f14;
f1 = simplify(f1);

f21 = get_elastic_force(r(:, 5), r(:, 4), k(4), ro(4));
f22 = get_elastic_force(r(:, 5), r(:, 6), k(5), ro(5));
f23 = get_elastic_force(r(:, 5), r(:, 2), k(8), ro(8));
f24 = get_elastic_force(r(:, 5), r(:, 3), k(12), ro(12));
f2 = f21+f22+f23+f24;
f2 = simplify(f2);

f31 = get_elastic_force(r(:, 4), r(:, 5), k(4), ro(4));
f32 = get_elastic_force(r(:, 4), r(:, 6), k(6), ro(6));
f33 = get_elastic_force(r(:, 4), r(:, 1), k(7), ro(7));
f34 = get_elastic_force(r(:, 4), r(:, 2), k(11), ro(11));
f3 = f31+f32+f33+f34;
f3 = simplify(f3);

K_r1 = jacobian(f1, r(:));
K_r1 = simplify(K_r1);
K_r2 = jacobian(f2, r(:));
K_r2 = simplify(K_r2);
K_r3 = jacobian(f3, r(:));
K_r3 = simplify(K_r3);

K_ro1 = jacobian(f1, r(:));
K_ro1 = simplify(K_ro1);
K_ro2 = jacobian(f2, r(:));
K_ro2 = simplify(K_ro2);
K_ro3 = jacobian(f3, r(:));
K_ro3 = simplify(K_ro3);

matlabFunction(K_r1,   'File', 'g_K_r1',            'Vars', {r, ro});
matlabFunction(K_r2,   'File', 'g_K_r2',            'Vars', {r, ro});
matlabFunction(K_r3,   'File', 'g_K_r3',            'Vars', {r, ro});
matlabFunction(K_ro1,  'File', 'g_K_ro1',           'Vars', {r, ro});
matlabFunction(K_ro2,  'File', 'g_K_ro2',           'Vars', {r, ro});
matlabFunction(K_ro3,  'File', 'g_K_ro3',           'Vars', {r, ro});

function f = get_elastic_force(ri, rj, k, ro)

f = k * (norm(rj - ri) - ro) * (rj - ri) / norm(rj - ri);

end


