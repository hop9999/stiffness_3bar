function tensor_K_ro = g_tensor_K_ro(in1,in2)
%G_TENSOR_K_RO
%    TENSOR_K_RO = G_TENSOR_K_RO(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    23-Dec-2019 15:45:00

r1 = in1(1,:);
r2 = in1(2,:);
r3 = in1(3,:);
t6 = r3-1.0515;
t2 = abs(t6);
t8 = r2-5.398e-1;
t3 = abs(t8);
t5 = r1+3.117e-1;
t4 = abs(t5);
t7 = t2.^2;
t9 = t3.^2;
t10 = t4.^2;
t11 = abs(r2);
t13 = r1-6.233e-1;
t12 = abs(t13);
t14 = t11.^2;
t15 = t12.^2;
t16 = r2.*2.0;
t17 = r2.^2;
t18 = r3.^2;
t19 = abs(r3);
t23 = sqrt(3.0);
t24 = t23.*(1.0./2.0);
t25 = r1-t24;
t20 = abs(t25);
t27 = r2-1.0./2.0;
t21 = abs(t27);
t22 = t19.^2;
t26 = t20.^2;
t28 = t21.^2;
t29 = t7+t9+t10;
t30 = 1.0./t29.^(3.0./2.0);
t31 = t7+t14+t15;
t32 = 1.0./t31.^(3.0./2.0);
t33 = abs(r1);
t34 = r2+1.0;
t35 = abs(t34);
t36 = t22+t26+t28;
t37 = 1.0./t36.^(3.0./2.0);
t38 = r3.*2.0e3;
t39 = t38-2.103e3;
t40 = r1.*1.0e4;
t41 = t33.^2;
t42 = t35.^2;
t43 = t22+t41+t42;
t44 = 1.0./t43.^(3.0./2.0);
t45 = t5.*t8.*t30.*1.0e1;
t46 = r2.*t13.*t32.*1.0e1;
t47 = r1.*t34.*t44.*1.0e1;
t48 = t25.*t27.*t37.*1.0e2;
t49 = r1.^2;
t50 = t16+t17+t18+t49+1.0;
t51 = 1.0./t50.^(3.0./2.0);
t52 = t40+3.117e3;
t53 = t30.*t39.*t52.*5.0e-7;
t54 = t40-6.233e3;
t55 = t32.*t39.*t54.*5.0e-7;
t56 = r1.*r3.*t44.*1.0e1;
t57 = r1.*r3.*1.0e2;
t58 = t37.*(t57-r3.*t23.*5.0e1);
t59 = r2.*5.0e3;
t60 = t59-2.699e3;
t61 = t30.*t39.*t60.*1.0e-6;
t62 = r2.*t32.*t39.*(1.0./2.0e2);
t63 = r3.*t34.*t44.*1.0e1;
t64 = t16-1.0;
t65 = r3.*t37.*t64.*5.0e1;
t66 = t6.^2;
tensor_K_ro = reshape([t30.*(t7+t9+t10-t5.^2).*-1.0e1,t45,t53,t45,t30.*(t7+t9+t10-t8.^2).*-1.0e1,t61,t53,t61,t30.*(t7+t9+t10-t66).*-1.0e1,t32.*(t7+t14+t15-t13.^2).*-1.0e1,t46,t55,t46,t32.*(t7+t14+t15-t17).*-1.0e1,t62,t55,t62,t32.*(t7+t14+t15-t66).*-1.0e1,t51.*(t16+t17+t18+1.0).*-1.0e1,t47,t56,t47,t51.*(t18+t49).*-1.0e1,t63,t56,t63,t51.*(t16+t17+t49+1.0).*-1.0e1,t37.*(t22+t26+t28-t25.^2).*-1.0e2,t48,t58,t48,t37.*(t22+t26+t28-t27.^2).*-1.0e2,t65,t58,t65,t37.*(-t18+t22+t26+t28).*-1.0e2],[9,4]);
