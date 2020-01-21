%r_init - vector, contaning values for the static nodes.
function P = ML_potens_energy_numerical(q, r_init, ro, k)
    
    r1 = ML_get_node(r_init, 1);
    r2 = ML_get_node(r_init, 2);
    r3 = ML_get_node(r_init, 3);
    r4 = q(1:3)';
    r5 = q(4:6)';
    r6 = q(7:9)';
%     q
    P1 =  0.5*k(1)*(norm(r1 - r2) - ro(1))^2;
    P2 =  0.5*k(2)*(norm(r2 - r3) - ro(2))^2;
    P3 =  0.5*k(3)*(norm(r3 - r1) - ro(3))^2;
    P4 =  0.5*k(4)*(norm(r4 - r5) - ro(4))^2;
    P5 =  0.5*k(5)*(norm(r5 - r6) - ro(5))^2;
    P6 =  0.5*k(6)*(norm(r6 - r4) - ro(6))^2;
    P7 =  0.5*k(7)*(norm(r1 - r4) - ro(7))^2;
    P8 =  0.5*k(8)*(norm(r2 - r5) - ro(8))^2;
    P9 =  0.5*k(9)*(norm(r3 - r6) - ro(9))^2;
    P10 =  0.5*k(10)*(norm(r1 - r6) - ro(10))^2;
    P11 =  0.5*k(11)*(norm(r2 - r4) - ro(11))^2;
    P12 =  0.5*k(12)*(norm(r3 - r5) - ro(12))^2;
    P = P1 + P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9 + P10 + P11 + P12;
end