function S = g_S(in1,in2)
%G_S
%    S = G_S(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    23-Dec-2019 17:50:19

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
t3 = abs(t2);
t7 = r2_1-r2_6;
t4 = abs(t7);
t9 = r3_1-r3_6;
t5 = abs(t9);
t6 = t3.^2;
t8 = t4.^2;
t10 = t5.^2;
t11 = t6+t8+t10;
t12 = sqrt(t11);
t13 = t2.^2;
t14 = r1_3-r1_6;
t15 = abs(t14);
t19 = r2_3-r2_6;
t16 = abs(t19);
t21 = r3_3-r3_6;
t17 = abs(t21);
t18 = t15.^2;
t20 = t16.^2;
t22 = t17.^2;
t23 = t18+t20+t22;
t24 = sqrt(t23);
t25 = t14.^2;
t26 = r1_4-r1_6;
t27 = abs(t26);
t31 = r2_4-r2_6;
t28 = abs(t31);
t33 = r3_4-r3_6;
t29 = abs(t33);
t30 = t27.^2;
t32 = t28.^2;
t34 = t29.^2;
t35 = t30+t32+t34;
t36 = sqrt(t35);
t37 = t26.^2;
t38 = r1_5-r1_6;
t39 = abs(t38);
t43 = r2_5-r2_6;
t40 = abs(t43);
t45 = r3_5-r3_6;
t41 = abs(t45);
t42 = t39.^2;
t44 = t40.^2;
t46 = t41.^2;
t47 = t42+t44+t46;
t48 = sqrt(t47);
t49 = t38.^2;
t50 = ro11-t12;
t51 = 1.0./t11.^(3.0./2.0);
t52 = ro9-t24;
t53 = 1.0./t23.^(3.0./2.0);
t54 = ro6-t36;
t55 = 1.0./t35.^(3.0./2.0);
t56 = ro5-t48;
t57 = 1.0./t47.^(3.0./2.0);
t58 = t2.*t7.*t12;
t59 = t2.*t7.*t50;
t60 = t58+t59;
t61 = t51.*t60.*1.0e2;
t62 = t14.*t19.*t24;
t63 = t14.*t19.*t52;
t64 = t62+t63;
t65 = t53.*t64.*1.0e1;
t66 = t26.*t31.*t36;
t67 = t26.*t31.*t54;
t68 = t66+t67;
t69 = t55.*t68.*1.0e1;
t70 = t38.*t43.*t48;
t71 = t38.*t43.*t56;
t72 = t70+t71;
t73 = t57.*t72.*1.0e1;
t74 = t61+t65+t69+t73;
t75 = t7.^2;
t76 = t19.^2;
t77 = t31.^2;
t78 = t43.^2;
t79 = t2.*t9.*t12;
t80 = t2.*t9.*t50;
t81 = t79+t80;
t82 = t51.*t81.*1.0e2;
t83 = t14.*t21.*t24;
t84 = t14.*t21.*t52;
t85 = t83+t84;
t86 = t53.*t85.*1.0e1;
t87 = t26.*t33.*t36;
t88 = t26.*t33.*t54;
t89 = t87+t88;
t90 = t55.*t89.*1.0e1;
t91 = t38.*t45.*t48;
t92 = t38.*t45.*t56;
t93 = t91+t92;
t94 = t57.*t93.*1.0e1;
t95 = t82+t86+t90+t94;
t96 = t7.*t9.*t12;
t97 = t7.*t9.*t50;
t98 = t96+t97;
t99 = t51.*t98.*1.0e2;
t100 = t19.*t21.*t24;
t101 = t19.*t21.*t52;
t102 = t100+t101;
t103 = t53.*t102.*1.0e1;
t104 = t31.*t33.*t36;
t105 = t31.*t33.*t54;
t106 = t104+t105;
t107 = t55.*t106.*1.0e1;
t108 = t43.*t45.*t48;
t109 = t43.*t45.*t56;
t110 = t108+t109;
t111 = t57.*t110.*1.0e1;
t112 = t99+t103+t107+t111;
t113 = t9.^2;
t114 = t21.^2;
t115 = t33.^2;
t116 = t45.^2;
S = reshape([t51.*(t12.*t13-t50.*(t6+t8+t10-t13)).*1.0e2+t53.*(t24.*t25-t52.*(t18+t20+t22-t25)).*1.0e1+t55.*(t36.*t37-t54.*(t30+t32+t34-t37)).*1.0e1+t57.*(t48.*t49-t56.*(t42+t44+t46-t49)).*1.0e1,t74,t95,t74,t51.*(t12.*t75-t50.*(t6+t8+t10-t75)).*1.0e2+t53.*(t24.*t76-t52.*(t18+t20+t22-t76)).*1.0e1+t55.*(t36.*t77-t54.*(t30+t32+t34-t77)).*1.0e1+t57.*(t48.*t78-t56.*(t42+t44+t46-t78)).*1.0e1,t112,t95,t112,t51.*(t12.*t113-t50.*(t6+t8+t10-t113)).*1.0e2+t53.*(t24.*t114-t52.*(t18+t20+t22-t114)).*1.0e1+t55.*(t36.*t115-t54.*(t30+t32+t34-t115)).*1.0e1+t57.*(t48.*t116-t56.*(t42+t44+t46-t116)).*1.0e1],[3,3]);
