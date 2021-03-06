function f_o = g_errorflield_f_o(in1)
%G_ERRORFLIELD_F_O
%    F_O = G_ERRORFLIELD_F_O(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    20-Jan-2020 13:35:24

r1 = in1(1,:);
r2 = in1(2,:);
t2 = r1.^2;
t3 = r2.^2;
t4 = t2+t3;
t5 = 1.0./t4.^(3.0./2.0);
t6 = 1.0./t4.^2;
t7 = t2.*t3.*2.0;
t8 = t4.^(3.0./2.0);
t9 = sqrt(t4);
t10 = t2.^2;
t11 = t3.^2;
f_o = reshape([t6.*(t7+t10+t11+r1.*t8-t3.*t9),r2.*t5.*(r1+t2+t3),r1.*t5.*(r2+t2+t3),t6.*(t7+t10+t11+r2.*t8-t2.*t9)],[2,2]);
