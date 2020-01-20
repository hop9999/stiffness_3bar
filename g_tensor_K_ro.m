function tensor_K_ro = g_tensor_K_ro(in1,in2)
%G_TENSOR_K_RO
%    TENSOR_K_RO = G_TENSOR_K_RO(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    20-Jan-2020 18:47:09

r1_1 = in1(1);
r1_3 = in1(7);
r1_4 = in1(10);
r1_5 = in1(13);
r1_6 = in1(16);
r2_1 = in1(2);
r2_3 = in1(8);
r2_4 = in1(11);
r2_5 = in1(14);
r2_6 = in1(17);
r3_1 = in1(3);
r3_3 = in1(9);
r3_4 = in1(12);
r3_5 = in1(15);
r3_6 = in1(18);
t5 = r1_5-r1_6;
t2 = abs(t5);
t7 = r2_5-r2_6;
t3 = abs(t7);
t9 = r3_5-r3_6;
t4 = abs(t9);
t6 = t2.^2;
t8 = t3.^2;
t10 = t4.^2;
t14 = r1_4-r1_6;
t11 = abs(t14);
t16 = r2_4-r2_6;
t12 = abs(t16);
t18 = r3_4-r3_6;
t13 = abs(t18);
t15 = t11.^2;
t17 = t12.^2;
t19 = t13.^2;
t23 = r1_3-r1_6;
t20 = abs(t23);
t25 = r2_3-r2_6;
t21 = abs(t25);
t27 = r3_3-r3_6;
t22 = abs(t27);
t24 = t20.^2;
t26 = t21.^2;
t28 = t22.^2;
t32 = r1_1-r1_6;
t29 = abs(t32);
t34 = r2_1-r2_6;
t30 = abs(t34);
t36 = r3_1-r3_6;
t31 = abs(t36);
t33 = t29.^2;
t35 = t30.^2;
t37 = t31.^2;
t38 = t6+t8+t10;
t39 = 1.0./t38.^(3.0./2.0);
t40 = t15+t17+t19;
t41 = 1.0./t40.^(3.0./2.0);
t42 = t24+t26+t28;
t43 = 1.0./t42.^(3.0./2.0);
t44 = t33+t35+t37;
t45 = 1.0./t44.^(3.0./2.0);
t46 = t5.*t7.*t39.*1.0e1;
t47 = t14.*t16.*t41.*1.0e1;
t48 = t23.*t25.*t43.*1.0e1;
t49 = t32.*t34.*t45.*1.0e2;
t50 = t5.*t9.*t39.*1.0e1;
t51 = t14.*t18.*t41.*1.0e1;
t52 = t23.*t27.*t43.*1.0e1;
t53 = t32.*t36.*t45.*1.0e2;
t54 = t7.*t9.*t39.*1.0e1;
t55 = t16.*t18.*t41.*1.0e1;
t56 = t25.*t27.*t43.*1.0e1;
t57 = t34.*t36.*t45.*1.0e2;
tensor_K_ro = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t39.*(t6+t8+t10-t5.^2).*-1.0e1,t46,t50,t46,t39.*(t6+t8+t10-t7.^2).*-1.0e1,t54,t50,t54,t39.*(t6+t8+t10-t9.^2).*-1.0e1,t41.*(t15+t17+t19-t14.^2).*-1.0e1,t47,t51,t47,t41.*(t15+t17+t19-t16.^2).*-1.0e1,t55,t51,t55,t41.*(t15+t17+t19-t18.^2).*-1.0e1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t43.*(t24+t26+t28-t23.^2).*-1.0e1,t48,t52,t48,t43.*(t24+t26+t28-t25.^2).*-1.0e1,t56,t52,t56,t43.*(t24+t26+t28-t27.^2).*-1.0e1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t45.*(t33+t35+t37-t32.^2).*-1.0e2,t49,t53,t49,t45.*(t33+t35+t37-t34.^2).*-1.0e2,t57,t53,t57,t45.*(t33+t35+t37-t36.^2).*-1.0e2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[9,12]);
