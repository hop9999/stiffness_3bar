function K_r1 = g_K_r1(in1,in2)
%G_K_R1
%    K_R1 = G_K_R1(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    20-Jan-2020 10:34:53

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
ro5 = in2(5,:);
ro6 = in2(6,:);
ro9 = in2(9,:);
ro11 = in2(11,:);
t2 = r1_1-r1_6;
t3 = r2_1-r2_6;
t4 = r3_1-r3_6;
t5 = t2.^2;
t6 = t3.^2;
t7 = t4.^2;
t8 = t5+t6+t7;
t9 = 1.0./t8.^(3.0./2.0);
t10 = r1_3-r1_6;
t11 = r2_3-r2_6;
t12 = r3_3-r3_6;
t13 = t10.^2;
t14 = t11.^2;
t15 = t12.^2;
t16 = t13+t14+t15;
t17 = r1_6.^2;
t18 = 1.0./t16.^(3.0./2.0);
t19 = r1_4-r1_6;
t20 = r2_4-r2_6;
t21 = r3_4-r3_6;
t22 = t19.^2;
t23 = t20.^2;
t24 = t21.^2;
t25 = t22+t23+t24;
t26 = 1.0./t25.^(3.0./2.0);
t27 = r1_5-r1_6;
t28 = r2_5-r2_6;
t29 = r3_5-r3_6;
t30 = t27.^2;
t31 = t28.^2;
t32 = t29.^2;
t33 = t30+t31+t32;
t34 = 1.0./t33.^(3.0./2.0);
t35 = r1_1.*2.0;
t36 = r1_6.*2.0;
t37 = t35-t36;
t38 = r1_3.*2.0;
t39 = t36-t38;
t40 = r1_4.*2.0;
t41 = t36-t40;
t42 = r1_5.*2.0;
t43 = t36-t42;
t44 = r2_1.*2.0;
t45 = r2_6.*2.0;
t46 = t44-t45;
t47 = r2_3.*2.0;
t48 = t45-t47;
t49 = r2_4.*2.0;
t50 = t45-t49;
t51 = r2_5.*2.0;
t52 = t45-t51;
t53 = r3_1.*2.0;
t54 = r3_6.*2.0;
t55 = t53-t54;
t56 = r3_3.*2.0;
t57 = t54-t56;
t58 = r3_4.*2.0;
t59 = t54-t58;
t60 = r3_5.*2.0;
t61 = t54-t60;
t62 = ro11.*t2.*t3.*t9.*1.0e2;
t63 = t8.^(3.0./2.0);
t64 = t63.*1.0e2;
t65 = ro9.*t10.*t11.*t18.*1.0e1;
t66 = r2_6.^2;
t67 = t16.^(3.0./2.0);
t68 = t67.*1.0e1;
t69 = ro6.*t19.*t20.*t26.*1.0e1;
t70 = t25.^(3.0./2.0);
t71 = t70.*1.0e1;
t72 = ro5.*t27.*t28.*t34.*1.0e1;
t73 = t33.^(3.0./2.0);
t74 = t73.*1.0e1;
t75 = 1.0./sqrt(t8);
t76 = ro11.*t75.*1.0e2;
t77 = 1.0./sqrt(t25);
t78 = ro6.*t77.*1.0e1;
t79 = 1.0./sqrt(t16);
t80 = ro9.*t79.*1.0e1;
t81 = 1.0./sqrt(t33);
t82 = ro5.*t81.*1.0e1;
t83 = ro11.*t2.*t4.*t9.*1.0e2;
t84 = ro11.*t3.*t4.*t9.*1.0e2;
t85 = ro9.*t10.*t12.*t18.*1.0e1;
t86 = ro9.*t11.*t12.*t18.*1.0e1;
t87 = r3_6.^2;
t88 = ro6.*t19.*t21.*t26.*1.0e1;
t89 = ro6.*t20.*t21.*t26.*1.0e1;
t90 = ro5.*t27.*t29.*t34.*1.0e1;
t91 = ro5.*t28.*t29.*t34.*1.0e1;
K_r1 = reshape([t9.*(t64-ro11.*t8.*1.0e2+ro11.*t17.*1.0e2+r1_1.^2.*ro11.*1.0e2-r1_1.*r1_6.*ro11.*2.0e2),t62,t83,t62,t9.*(t64-ro11.*t8.*1.0e2+ro11.*t66.*1.0e2+r2_1.^2.*ro11.*1.0e2-r2_1.*r2_6.*ro11.*2.0e2),t84,t83,t84,t9.*(t64-ro11.*t8.*1.0e2+ro11.*t87.*1.0e2+r3_1.^2.*ro11.*1.0e2-r3_1.*r3_6.*ro11.*2.0e2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t18.*(t68-ro9.*t16.*1.0e1+ro9.*t17.*1.0e1+r1_3.^2.*ro9.*1.0e1-r1_3.*r1_6.*ro9.*2.0e1),t65,t85,t65,t18.*(t68-ro9.*t16.*1.0e1+ro9.*t66.*1.0e1+r2_3.^2.*ro9.*1.0e1-r2_3.*r2_6.*ro9.*2.0e1),t86,t85,t86,t18.*(t68-ro9.*t16.*1.0e1+ro9.*t87.*1.0e1+r3_3.^2.*ro9.*1.0e1-r3_3.*r3_6.*ro9.*2.0e1),t26.*(t71+ro6.*t17.*1.0e1-ro6.*t25.*1.0e1+r1_4.^2.*ro6.*1.0e1-r1_4.*r1_6.*ro6.*2.0e1),t69,t88,t69,t26.*(t71-ro6.*t25.*1.0e1+ro6.*t66.*1.0e1+r2_4.^2.*ro6.*1.0e1-r2_4.*r2_6.*ro6.*2.0e1),t89,t88,t89,t26.*(t71-ro6.*t25.*1.0e1+ro6.*t87.*1.0e1+r3_4.^2.*ro6.*1.0e1-r3_4.*r3_6.*ro6.*2.0e1),t34.*(t74+ro5.*t17.*1.0e1-ro5.*t33.*1.0e1+r1_5.^2.*ro5.*1.0e1-r1_5.*r1_6.*ro5.*2.0e1),t72,t90,t72,t34.*(t74-ro5.*t33.*1.0e1+ro5.*t66.*1.0e1+r2_5.^2.*ro5.*1.0e1-r2_5.*r2_6.*ro5.*2.0e1),t91,t90,t91,t34.*(t74-ro5.*t33.*1.0e1+ro5.*t87.*1.0e1+r3_5.^2.*ro5.*1.0e1-r3_5.*r3_6.*ro5.*2.0e1),t76+t78+t80+t82-r1_1.*ro11.*t9.*t37.*5.0e1+r1_6.*ro11.*t9.*t37.*5.0e1+r1_3.*ro9.*t18.*t39.*5.0-r1_6.*ro9.*t18.*t39.*5.0+r1_4.*ro6.*t26.*t41.*5.0-r1_6.*ro6.*t26.*t41.*5.0+r1_5.*ro5.*t34.*t43.*5.0-r1_6.*ro5.*t34.*t43.*5.0-1.3e2,r2_1.*ro11.*t9.*t37.*-5.0e1+r2_6.*ro11.*t9.*t37.*5.0e1-r2_6.*ro9.*t18.*t39.*5.0-r2_6.*ro6.*t26.*t41.*5.0-r2_6.*ro5.*t34.*t43.*5.0+r2_3.*ro9.*t18.*(t36-t38).*5.0+r2_4.*ro6.*t26.*(t36-t40).*5.0+r2_5.*ro5.*t34.*(t36-t42).*5.0,r3_1.*ro11.*t9.*t37.*-5.0e1+r3_6.*ro11.*t9.*t37.*5.0e1-r3_6.*ro9.*t18.*t39.*5.0-r3_6.*ro6.*t26.*t41.*5.0-r3_6.*ro5.*t34.*t43.*5.0+r3_3.*ro9.*t18.*(t36-t38).*5.0+r3_4.*ro6.*t26.*(t36-t40).*5.0+r3_5.*ro5.*t34.*(t36-t42).*5.0,r1_1.*ro11.*t9.*t46.*-5.0e1+r1_6.*ro11.*t9.*t46.*5.0e1+r1_3.*ro9.*t18.*t48.*5.0-r1_6.*ro9.*t18.*t48.*5.0+r1_4.*ro6.*t26.*t50.*5.0-r1_6.*ro6.*t26.*t50.*5.0+r1_5.*ro5.*t34.*t52.*5.0-r1_6.*ro5.*t34.*t52.*5.0,t76+t78+t80+t82-r2_1.*ro11.*t9.*t46.*5.0e1+r2_6.*ro11.*t9.*t46.*5.0e1-r2_6.*ro9.*t18.*t48.*5.0-r2_6.*ro6.*t26.*t50.*5.0-r2_6.*ro5.*t34.*t52.*5.0+r2_3.*ro9.*t18.*(t45-t47).*5.0+r2_4.*ro6.*t26.*(t45-t49).*5.0+r2_5.*ro5.*t34.*(t45-t51).*5.0-1.3e2,r3_1.*ro11.*t9.*t46.*-5.0e1+r3_6.*ro11.*t9.*t46.*5.0e1-r3_6.*ro9.*t18.*t48.*5.0-r3_6.*ro6.*t26.*t50.*5.0-r3_6.*ro5.*t34.*t52.*5.0+r3_3.*ro9.*t18.*(t45-t47).*5.0+r3_4.*ro6.*t26.*(t45-t49).*5.0+r3_5.*ro5.*t34.*(t45-t51).*5.0,r1_1.*ro11.*t9.*t55.*-5.0e1+r1_6.*ro11.*t9.*t55.*5.0e1+r1_3.*ro9.*t18.*t57.*5.0-r1_6.*ro9.*t18.*t57.*5.0+r1_4.*ro6.*t26.*t59.*5.0-r1_6.*ro6.*t26.*t59.*5.0+r1_5.*ro5.*t34.*t61.*5.0-r1_6.*ro5.*t34.*t61.*5.0,r2_1.*ro11.*t9.*t55.*-5.0e1+r2_6.*ro11.*t9.*t55.*5.0e1-r2_6.*ro9.*t18.*t57.*5.0-r2_6.*ro6.*t26.*t59.*5.0-r2_6.*ro5.*t34.*t61.*5.0+r2_3.*ro9.*t18.*(t54-t56).*5.0+r2_4.*ro6.*t26.*(t54-t58).*5.0+r2_5.*ro5.*t34.*(t54-t60).*5.0,t76+t78+t80+t82-r3_1.*ro11.*t9.*t55.*5.0e1+r3_6.*ro11.*t9.*t55.*5.0e1-r3_6.*ro9.*t18.*t57.*5.0-r3_6.*ro6.*t26.*t59.*5.0-r3_6.*ro5.*t34.*t61.*5.0+r3_3.*ro9.*t18.*(t54-t56).*5.0+r3_4.*ro6.*t26.*(t54-t58).*5.0+r3_5.*ro5.*t34.*(t54-t60).*5.0-1.3e2],[3,18]);
