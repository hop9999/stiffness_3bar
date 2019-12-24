function [f] = get_forces_sum_approx(r, robot)
k  = robot.k;

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

%calculate linearization normals
s = size(con);
normal = zeros(3,s(1));
for i = 1:s(1)
    normal(:,i) = (r(:,con(i,1)) - r(:,con(i,2)))/...
        norm(r(:,con(i,1)) - r(:,con(i,2)));
end

for i = 1:s(1)
    f_norm(:,i) = -k(con(i,3))*((r(:,con(i,1)) - r(:,con(i,2))) - ro_init(con(i,3))*normal(:,i));
end

for i = 1:s(1)
    f_tang(:,i) = -k(con(i,3))*(dot(normal(:,i), (r(:,con(i,1)) - r(:,con(i,2)))) - ro_init(con(i,3)))*normal(:,i);
end

for i = 1:s(1)
    f_res(:,i) = (f_norm(:,i) + f_tang(:,i))/2;
end

f4 = f_res(:,1) + f_res(:,2) + f_res(:,3) + f_res(:,4);

f5 = f_res(:,5) + f_res(:,6) + f_res(:,7) + f_res(:,8);

f6 = f_res(:,9) + f_res(:,10) + f_res(:,11) + f_res(:,12);

f = [f4 f5 f6];