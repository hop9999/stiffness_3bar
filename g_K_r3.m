function K_r3 = g_K_r3(in1,in2)
%G_K_R3
%    K_R3 = G_K_R3(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    20-Jan-2020 10:35:01

r1_1 = in1(1);
r1_2 = in1(4);
r1_4 = in1(10);
r1_5 = in1(13);
r1_6 = in1(16);
r2_1 = in1(2);
r2_2 = in1(5);
r2_4 = in1(11);
r2_5 = in1(14);
r2_6 = in1(17);
r3_1 = in1(3);
r3_2 = in1(6);
r3_4 = in1(12);
r3_5 = in1(15);
r3_6 = in1(18);
ro4 = in2(4,:);
ro6 = in2(6,:);
ro7 = in2(7,:);
ro11 = in2(11,:);
t2 = r1_1-r1_4;
t3 = r2_1-r2_4;
t4 = r3_1-r3_4;
t5 = t2.^2;
t6 = t3.^2;
t7 = t4.^2;
t8 = t5+t6+t7;
t9 = 1.0./t8.^(3.0./2.0);
t10 = r1_2-r1_4;
t11 = r2_2-r2_4;
t12 = r3_2-r3_4;
t13 = t10.^2;
t14 = t11.^2;
t15 = t12.^2;
t16 = t13+t14+t15;
t17 = r1_4.^2;
t18 = 1.0./t16.^(3.0./2.0);
t19 = r1_4-r1_5;
t20 = r2_4-r2_5;
t21 = r3_4-r3_5;
t22 = t19.^2;
t23 = t20.^2;
t24 = t21.^2;
t25 = t22+t23+t24;
t26 = r1_4-r1_6;
t27 = r2_4-r2_6;
t28 = r3_4-r3_6;
t29 = t26.^2;
t30 = t27.^2;
t31 = t28.^2;
t32 = t29+t30+t31;
t33 = r1_4.*2.0;
t34 = r1_1.*2.0;
t35 = ro7.*1.0e1;
t36 = sqrt(t8);
t58 = t36.*1.0e1;
t37 = t35-t58;
t38 = r1_5.*2.0;
t39 = t33-t38;
t40 = ro4.*1.0e1;
t41 = sqrt(t25);
t61 = t41.*1.0e1;
t42 = t40-t61;
t43 = r1_6.*2.0;
t44 = t33-t43;
t45 = ro6.*1.0e1;
t46 = sqrt(t32);
t65 = t46.*1.0e1;
t47 = t45-t65;
t48 = r1_2.*2.0;
t49 = ro11.*1.0e2;
t50 = sqrt(t16);
t68 = t50.*1.0e2;
t51 = t49-t68;
t52 = 1.0./t8;
t53 = 1.0./t16;
t54 = 1.0./t25;
t55 = r2_4.*2.0;
t56 = 1.0./t32;
t57 = r2_1.*2.0;
t59 = r2_5.*2.0;
t60 = t55-t59;
t62 = 1.0./t25.^(3.0./2.0);
t63 = r2_6.*2.0;
t64 = t55-t63;
t66 = 1.0./t32.^(3.0./2.0);
t67 = r2_2.*2.0;
t69 = r3_4.*2.0;
t70 = r3_1.*2.0;
t71 = r3_5.*2.0;
t72 = t69-t71;
t73 = r3_6.*2.0;
t74 = t69-t73;
t75 = r3_2.*2.0;
t76 = ro7.*t2.*t3.*t9.*1.0e1;
t77 = t8.^(3.0./2.0);
t78 = t77.*1.0e1;
t79 = ro11.*t10.*t11.*t18.*1.0e2;
t80 = r2_4.^2;
t81 = t16.^(3.0./2.0);
t82 = t81.*1.0e2;
t83 = 1.0./sqrt(t8);
t84 = t37.*t83;
t85 = 1.0./sqrt(t25);
t86 = t42.*t85;
t87 = 1.0./sqrt(t32);
t88 = t47.*t87;
t89 = 1.0./sqrt(t16);
t90 = t51.*t89;
t91 = ro4.*t19.*t20.*t62.*1.0e1;
t92 = t25.^(3.0./2.0);
t93 = t92.*1.0e1;
t94 = ro6.*t26.*t27.*t66.*1.0e1;
t95 = t32.^(3.0./2.0);
t96 = t95.*1.0e1;
t97 = ro7.*t2.*t4.*t9.*1.0e1;
t98 = ro7.*t3.*t4.*t9.*1.0e1;
t99 = ro11.*t10.*t12.*t18.*1.0e2;
t100 = ro11.*t11.*t12.*t18.*1.0e2;
t101 = r3_4.^2;
t102 = ro4.*t19.*t21.*t62.*1.0e1;
t103 = ro4.*t20.*t21.*t62.*1.0e1;
t104 = ro6.*t26.*t28.*t66.*1.0e1;
t105 = ro6.*t27.*t28.*t66.*1.0e1;
K_r3 = reshape([t9.*(t78-ro7.*t8.*1.0e1+ro7.*t17.*1.0e1+r1_1.^2.*ro7.*1.0e1-r1_1.*r1_4.*ro7.*2.0e1),t76,t97,t76,t9.*(t78-ro7.*t8.*1.0e1+ro7.*t80.*1.0e1+r2_1.^2.*ro7.*1.0e1-r2_1.*r2_4.*ro7.*2.0e1),t98,t97,t98,t9.*(t78-ro7.*t8.*1.0e1+ro7.*t101.*1.0e1+r3_1.^2.*ro7.*1.0e1-r3_1.*r3_4.*ro7.*2.0e1),t18.*(t82-ro11.*t16.*1.0e2+ro11.*t17.*1.0e2+r1_2.^2.*ro11.*1.0e2-r1_2.*r1_4.*ro11.*2.0e2),t79,t99,t79,t18.*(t82-ro11.*t16.*1.0e2+ro11.*t80.*1.0e2+r2_2.^2.*ro11.*1.0e2-r2_2.*r2_4.*ro11.*2.0e2),t100,t99,t100,t18.*(t82-ro11.*t16.*1.0e2+ro11.*t101.*1.0e2+r3_2.^2.*ro11.*1.0e2-r3_2.*r3_4.*ro11.*2.0e2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t84+t86+t88+t90+t2.*t52.*(r1_4.*2.0-t34).*5.0+t10.*t53.*(r1_4.*2.0-t48).*5.0e1-t19.*t39.*t54.*5.0-t26.*t44.*t56.*5.0-t19.*t39.*t42.*t62.*(1.0./2.0)-t26.*t44.*t47.*t66.*(1.0./2.0)+t2.*t9.*t37.*(t33-t34).*(1.0./2.0)+t10.*t18.*t51.*(t33-t48).*(1.0./2.0),t20.*t39.*t54.*-5.0-t27.*t44.*t56.*5.0+t3.*t52.*(t33-t34).*5.0+t11.*t53.*(t33-t48).*5.0e1-t20.*t39.*t42.*t62.*(1.0./2.0)-t27.*t44.*t47.*t66.*(1.0./2.0)+t3.*t9.*t37.*(t33-t34).*(1.0./2.0)+t11.*t18.*t51.*(t33-t48).*(1.0./2.0),t21.*t39.*t54.*-5.0-t28.*t44.*t56.*5.0+t4.*t52.*(t33-t34).*5.0+t12.*t53.*(t33-t48).*5.0e1-t21.*t39.*t42.*t62.*(1.0./2.0)-t28.*t44.*t47.*t66.*(1.0./2.0)+t4.*t9.*t37.*(t33-t34).*(1.0./2.0)+t12.*t18.*t51.*(t33-t48).*(1.0./2.0),t2.*t52.*(r2_4.*2.0-t57).*5.0+t10.*t53.*(r2_4.*2.0-t67).*5.0e1-t19.*t54.*t60.*5.0-t26.*t56.*t64.*5.0-t19.*t42.*t60.*t62.*(1.0./2.0)-t26.*t47.*t64.*t66.*(1.0./2.0)+t2.*t9.*t37.*(t55-t57).*(1.0./2.0)+t10.*t18.*t51.*(t55-t67).*(1.0./2.0),t84+t86+t88+t90-t20.*t54.*t60.*5.0-t27.*t56.*t64.*5.0+t3.*t52.*(t55-t57).*5.0+t11.*t53.*(t55-t67).*5.0e1-t20.*t42.*t60.*t62.*(1.0./2.0)-t27.*t47.*t64.*t66.*(1.0./2.0)+t3.*t9.*t37.*(t55-t57).*(1.0./2.0)+t11.*t18.*t51.*(t55-t67).*(1.0./2.0),t21.*t54.*t60.*-5.0-t28.*t56.*t64.*5.0+t4.*t52.*(t55-t57).*5.0+t12.*t53.*(t55-t67).*5.0e1-t21.*t42.*t60.*t62.*(1.0./2.0)-t28.*t47.*t64.*t66.*(1.0./2.0)+t4.*t9.*t37.*(t55-t57).*(1.0./2.0)+t12.*t18.*t51.*(t55-t67).*(1.0./2.0),t2.*t52.*(r3_4.*2.0-t70).*5.0+t10.*t53.*(r3_4.*2.0-t75).*5.0e1-t19.*t54.*t72.*5.0-t26.*t56.*t74.*5.0-t19.*t42.*t62.*t72.*(1.0./2.0)-t26.*t47.*t66.*t74.*(1.0./2.0)+t2.*t9.*t37.*(t69-t70).*(1.0./2.0)+t10.*t18.*t51.*(t69-t75).*(1.0./2.0),t20.*t54.*t72.*-5.0-t27.*t56.*t74.*5.0+t3.*t52.*(t69-t70).*5.0+t11.*t53.*(t69-t75).*5.0e1-t20.*t42.*t62.*t72.*(1.0./2.0)-t27.*t47.*t66.*t74.*(1.0./2.0)+t3.*t9.*t37.*(t69-t70).*(1.0./2.0)+t11.*t18.*t51.*(t69-t75).*(1.0./2.0),t84+t86+t88+t90-t21.*t54.*t72.*5.0-t28.*t56.*t74.*5.0+t4.*t52.*(t69-t70).*5.0+t12.*t53.*(t69-t75).*5.0e1-t21.*t42.*t62.*t72.*(1.0./2.0)-t28.*t47.*t66.*t74.*(1.0./2.0)+t4.*t9.*t37.*(t69-t70).*(1.0./2.0)+t12.*t18.*t51.*(t69-t75).*(1.0./2.0),t62.*(t93+ro4.*t17.*1.0e1-ro4.*t25.*1.0e1+r1_5.^2.*ro4.*1.0e1-r1_4.*r1_5.*ro4.*2.0e1),t91,t102,t91,t62.*(t93-ro4.*t25.*1.0e1+ro4.*t80.*1.0e1+r2_5.^2.*ro4.*1.0e1-r2_4.*r2_5.*ro4.*2.0e1),t103,t102,t103,t62.*(t93-ro4.*t25.*1.0e1+ro4.*t101.*1.0e1+r3_5.^2.*ro4.*1.0e1-r3_4.*r3_5.*ro4.*2.0e1),t66.*(t96+ro6.*t17.*1.0e1-ro6.*t32.*1.0e1+r1_6.^2.*ro6.*1.0e1-r1_4.*r1_6.*ro6.*2.0e1),t94,t104,t94,t66.*(t96-ro6.*t32.*1.0e1+ro6.*t80.*1.0e1+r2_6.^2.*ro6.*1.0e1-r2_4.*r2_6.*ro6.*2.0e1),t105,t104,t105,t66.*(t96-ro6.*t32.*1.0e1+ro6.*t101.*1.0e1+r3_6.^2.*ro6.*1.0e1-r3_4.*r3_6.*ro6.*2.0e1)],[3,18]);
