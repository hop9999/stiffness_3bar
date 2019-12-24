function normal = get_normal(r_init)

con = get_connectivity();
normal = zeros(3, size(con, 1));

for i = 1:size(con, 1)
    normal(:,i) = (r_init(:,con(i,1)) - r_init(:,con(i,2)))/...
        norm(r_init(:,con(i,1)) - r_init(:,con(i,2)));
end

end