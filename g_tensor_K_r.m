function tensor_K_r = g_tensor_K_r(in1,in2)
%G_TENSOR_K_R
%    TENSOR_K_R = G_TENSOR_K_R(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    23-Dec-2019 17:52:04

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
t5 = r1_1-r1_6;
t2 = abs(t5);
t7 = r2_1-r2_6;
t3 = abs(t7);
t9 = r3_1-r3_6;
t4 = abs(t9);
t6 = t2.^2;
t8 = t3.^2;
t10 = t4.^2;
t11 = t6+t8+t10;
t12 = sqrt(t11);
t13 = r1_1.*2.0;
t14 = r1_6.*2.0;
t15 = sign(t5);
t16 = t5.^2;
t17 = 1.0./sqrt(t11);
t18 = ro11-t12;
t19 = t6+t8+t10-t16;
t20 = 1.0./t11.^(3.0./2.0);
t21 = sign(t7);
t22 = t12.*t16;
t26 = t18.*t19;
t23 = t22-t26;
t24 = 1.0./t11.^(5.0./2.0);
t25 = sign(t9);
t30 = r1_3-r1_6;
t27 = abs(t30);
t32 = r2_3-r2_6;
t28 = abs(t32);
t34 = r3_3-r3_6;
t29 = abs(t34);
t31 = t27.^2;
t33 = t28.^2;
t35 = t29.^2;
t36 = t31+t33+t35;
t37 = sqrt(t36);
t38 = r1_3.*2.0;
t39 = sign(t30);
t40 = t30.^2;
t41 = 1.0./sqrt(t36);
t42 = ro9-t37;
t43 = t31+t33+t35-t40;
t44 = 1.0./t36.^(3.0./2.0);
t45 = sign(t32);
t46 = t37.*t40;
t50 = t42.*t43;
t47 = t46-t50;
t48 = 1.0./t36.^(5.0./2.0);
t49 = sign(t34);
t54 = r1_4-r1_6;
t51 = abs(t54);
t56 = r2_4-r2_6;
t52 = abs(t56);
t58 = r3_4-r3_6;
t53 = abs(t58);
t55 = t51.^2;
t57 = t52.^2;
t59 = t53.^2;
t60 = t55+t57+t59;
t61 = sqrt(t60);
t62 = r1_4.*2.0;
t63 = sign(t54);
t64 = t54.^2;
t65 = 1.0./sqrt(t60);
t66 = ro6-t61;
t67 = t55+t57+t59-t64;
t68 = 1.0./t60.^(3.0./2.0);
t69 = sign(t56);
t70 = t61.*t64;
t74 = t66.*t67;
t71 = t70-t74;
t72 = 1.0./t60.^(5.0./2.0);
t73 = sign(t58);
t78 = r1_5-r1_6;
t75 = abs(t78);
t80 = r2_5-r2_6;
t76 = abs(t80);
t82 = r3_5-r3_6;
t77 = abs(t82);
t79 = t75.^2;
t81 = t76.^2;
t83 = t77.^2;
t84 = t79+t81+t83;
t85 = sqrt(t84);
t86 = r1_5.*2.0;
t87 = sign(t78);
t88 = t78.^2;
t89 = 1.0./sqrt(t84);
t90 = ro5-t85;
t91 = t79+t81+t83-t88;
t92 = 1.0./t84.^(3.0./2.0);
t93 = sign(t80);
t94 = t85.*t88;
t98 = t90.*t91;
t95 = t94-t98;
t96 = 1.0./t84.^(5.0./2.0);
t97 = sign(t82);
t99 = t13-t14;
t100 = t12.*t99;
t101 = t2.*t15.*2.0;
t102 = -t13+t14+t101;
t103 = t2.*t15.*t17.*t19;
t104 = t2.*t15.*t16.*t17;
t105 = t100+t103+t104-t18.*t102;
t106 = t20.*t105.*1.0e2;
t107 = t14-t38;
t108 = t37.*t107;
t109 = t27.*t39.*2.0;
t110 = t14-t38+t109;
t111 = t42.*t110;
t112 = t14-t62;
t113 = t61.*t112;
t114 = t51.*t63.*2.0;
t115 = t14-t62+t114;
t116 = t66.*t115;
t117 = t14-t86;
t118 = t85.*t117;
t119 = t75.*t87.*2.0;
t120 = t14-t86+t119;
t121 = t90.*t120;
t122 = t3.*t17.*t19.*t21;
t123 = t3.*t16.*t17.*t21;
t370 = t3.*t18.*t21.*2.0;
t124 = t122+t123-t370;
t125 = t20.*t124.*1.0e2;
t126 = t28.*t41.*t43.*t45;
t127 = t28.*t40.*t41.*t45;
t379 = t28.*t42.*t45.*2.0;
t128 = t126+t127-t379;
t129 = t44.*t128.*1.0e1;
t130 = t52.*t65.*t67.*t69;
t131 = t52.*t64.*t65.*t69;
t387 = t52.*t66.*t69.*2.0;
t132 = t130+t131-t387;
t133 = t68.*t132.*1.0e1;
t134 = t76.*t89.*t91.*t93;
t135 = t76.*t88.*t89.*t93;
t395 = t76.*t90.*t93.*2.0;
t136 = t134+t135-t395;
t137 = t92.*t136.*1.0e1;
t138 = t4.*t17.*t19.*t25;
t139 = t4.*t16.*t17.*t25;
t247 = t4.*t18.*t25.*2.0;
t140 = t138+t139-t247;
t141 = t20.*t140.*1.0e2;
t142 = t29.*t41.*t43.*t49;
t143 = t29.*t40.*t41.*t49;
t254 = t29.*t42.*t49.*2.0;
t144 = t142+t143-t254;
t145 = t44.*t144.*1.0e1;
t146 = t53.*t65.*t67.*t73;
t147 = t53.*t64.*t65.*t73;
t261 = t53.*t66.*t73.*2.0;
t148 = t146+t147-t261;
t149 = t68.*t148.*1.0e1;
t150 = t77.*t89.*t91.*t97;
t151 = t77.*t88.*t89.*t97;
t268 = t77.*t90.*t97.*2.0;
t152 = t150+t151-t268;
t153 = t92.*t152.*1.0e1;
t154 = t5.*t7.*t12;
t155 = t5.*t7.*t18;
t156 = t154+t155;
t157 = t30.*t32.*t37;
t158 = t30.*t32.*t42;
t159 = t157+t158;
t160 = t54.*t56.*t61;
t161 = t54.*t56.*t66;
t162 = t160+t161;
t163 = t78.*t80.*t85;
t164 = t78.*t80.*t90;
t165 = t163+t164;
t166 = t7.*t18;
t167 = t7.*t12;
t168 = t166+t167;
t169 = t20.*t168.*1.0e2;
t170 = t32.*t42;
t171 = t32.*t37;
t172 = t170+t171;
t173 = t44.*t172.*1.0e1;
t174 = t56.*t66;
t175 = t56.*t61;
t176 = t174+t175;
t177 = t68.*t176.*1.0e1;
t178 = t80.*t90;
t179 = t80.*t85;
t180 = t178+t179;
t181 = t92.*t180.*1.0e1;
t182 = t5.*t18;
t183 = t5.*t12;
t184 = t182+t183;
t185 = t20.*t184.*1.0e2;
t186 = t30.*t42;
t187 = t30.*t37;
t188 = t186+t187;
t189 = t44.*t188.*1.0e1;
t190 = t54.*t66;
t191 = t54.*t61;
t192 = t190+t191;
t193 = t68.*t192.*1.0e1;
t194 = t78.*t90;
t195 = t78.*t85;
t196 = t194+t195;
t197 = t92.*t196.*1.0e1;
t198 = t5.*t9.*t12;
t199 = t5.*t9.*t18;
t200 = t198+t199;
t201 = t30.*t34.*t37;
t202 = t30.*t34.*t42;
t203 = t201+t202;
t204 = t54.*t58.*t61;
t205 = t54.*t58.*t66;
t206 = t204+t205;
t207 = t78.*t82.*t85;
t208 = t78.*t82.*t90;
t209 = t207+t208;
t210 = t9.*t18;
t211 = t9.*t12;
t212 = t210+t211;
t213 = t20.*t212.*1.0e2;
t214 = t34.*t42;
t215 = t34.*t37;
t216 = t214+t215;
t217 = t44.*t216.*1.0e1;
t218 = t58.*t66;
t219 = t58.*t61;
t220 = t218+t219;
t221 = t68.*t220.*1.0e1;
t222 = t82.*t90;
t223 = t82.*t85;
t224 = t222+t223;
t225 = t92.*t224.*1.0e1;
t226 = t2.*t15.*t24.*t156.*3.0e2;
t227 = t3.*t21.*t24.*t156.*3.0e2;
t228 = t4.*t24.*t25.*t156.*3.0e2;
t229 = t27.*t39.*t48.*t159.*3.0e1;
t230 = t28.*t45.*t48.*t159.*3.0e1;
t231 = t29.*t48.*t49.*t159.*3.0e1;
t232 = t51.*t63.*t72.*t162.*3.0e1;
t233 = t52.*t69.*t72.*t162.*3.0e1;
t234 = t53.*t72.*t73.*t162.*3.0e1;
t235 = t75.*t87.*t96.*t165.*3.0e1;
t236 = t76.*t93.*t96.*t165.*3.0e1;
t237 = t77.*t96.*t97.*t165.*3.0e1;
t238 = -t169-t173-t177-t181+t226+t229+t232+t235;
t239 = -t185-t189-t193-t197+t227+t230+t233+t236;
t240 = t228+t231+t234+t237;
t241 = t7.^2;
t242 = t6+t8+t10-t241;
t243 = r2_1.*2.0;
t244 = r2_6.*2.0;
t245 = t12.*t241;
t248 = t18.*t242;
t246 = t245-t248;
t249 = t32.^2;
t250 = t31+t33+t35-t249;
t251 = r2_3.*2.0;
t252 = t37.*t249;
t255 = t42.*t250;
t253 = t252-t255;
t256 = t56.^2;
t257 = t55+t57+t59-t256;
t258 = r2_4.*2.0;
t259 = t61.*t256;
t262 = t66.*t257;
t260 = t259-t262;
t263 = t80.^2;
t264 = t79+t81+t83-t263;
t265 = r2_5.*2.0;
t266 = t85.*t263;
t269 = t90.*t264;
t267 = t266-t269;
t270 = t2.*t15.*t17.*t242;
t271 = t2.*t15.*t17.*t241;
t367 = t2.*t15.*t18.*2.0;
t272 = t270+t271-t367;
t273 = t20.*t272.*1.0e2;
t274 = t27.*t39.*t41.*t250;
t275 = t27.*t39.*t41.*t249;
t376 = t27.*t39.*t42.*2.0;
t276 = t274+t275-t376;
t277 = t44.*t276.*1.0e1;
t278 = t51.*t63.*t65.*t257;
t279 = t51.*t63.*t65.*t256;
t384 = t51.*t63.*t66.*2.0;
t280 = t278+t279-t384;
t281 = t68.*t280.*1.0e1;
t282 = t75.*t87.*t89.*t264;
t283 = t75.*t87.*t89.*t263;
t392 = t75.*t87.*t90.*2.0;
t284 = t282+t283-t392;
t285 = t92.*t284.*1.0e1;
t286 = t243-t244;
t287 = t12.*t286;
t288 = t3.*t21.*2.0;
t289 = -t243+t244+t288;
t290 = t3.*t17.*t21.*t242;
t291 = t3.*t17.*t21.*t241;
t292 = t287+t290+t291-t18.*t289;
t293 = t20.*t292.*1.0e2;
t294 = t244-t251;
t295 = t37.*t294;
t296 = t28.*t45.*2.0;
t297 = t244-t251+t296;
t298 = t42.*t297;
t299 = t244-t258;
t300 = t61.*t299;
t301 = t52.*t69.*2.0;
t302 = t244-t258+t301;
t303 = t66.*t302;
t304 = t244-t265;
t305 = t85.*t304;
t306 = t76.*t93.*2.0;
t307 = t244-t265+t306;
t308 = t90.*t307;
t309 = t4.*t17.*t25.*t242;
t310 = t4.*t17.*t25.*t241;
t311 = -t247+t309+t310;
t312 = t20.*t311.*1.0e2;
t313 = t29.*t41.*t49.*t250;
t314 = t29.*t41.*t49.*t249;
t315 = -t254+t313+t314;
t316 = t44.*t315.*1.0e1;
t317 = t53.*t65.*t73.*t257;
t318 = t53.*t65.*t73.*t256;
t319 = -t261+t317+t318;
t320 = t68.*t319.*1.0e1;
t321 = t77.*t89.*t97.*t264;
t322 = t77.*t89.*t97.*t263;
t323 = -t268+t321+t322;
t324 = t92.*t323.*1.0e1;
t325 = t7.*t9.*t12;
t326 = t7.*t9.*t18;
t327 = t325+t326;
t328 = t32.*t34.*t37;
t329 = t32.*t34.*t42;
t330 = t328+t329;
t331 = t56.*t58.*t61;
t332 = t56.*t58.*t66;
t333 = t331+t332;
t334 = t80.*t82.*t85;
t335 = t80.*t82.*t90;
t336 = t334+t335;
t337 = t2.*t15.*t24.*t200.*3.0e2;
t338 = t3.*t21.*t24.*t200.*3.0e2;
t339 = t4.*t24.*t25.*t200.*3.0e2;
t340 = t27.*t39.*t48.*t203.*3.0e1;
t341 = t28.*t45.*t48.*t203.*3.0e1;
t342 = t29.*t48.*t49.*t203.*3.0e1;
t343 = t51.*t63.*t72.*t206.*3.0e1;
t344 = t52.*t69.*t72.*t206.*3.0e1;
t345 = t53.*t72.*t73.*t206.*3.0e1;
t346 = t75.*t87.*t96.*t209.*3.0e1;
t347 = t76.*t93.*t96.*t209.*3.0e1;
t348 = t77.*t96.*t97.*t209.*3.0e1;
t349 = -t213-t217-t221-t225+t337+t340+t343+t346;
t350 = t338+t341+t344+t347;
t351 = -t185-t189-t193-t197+t339+t342+t345+t348;
t352 = t2.*t15.*t24.*t327.*3.0e2;
t353 = t3.*t21.*t24.*t327.*3.0e2;
t354 = t4.*t24.*t25.*t327.*3.0e2;
t355 = t27.*t39.*t48.*t330.*3.0e1;
t356 = t28.*t45.*t48.*t330.*3.0e1;
t357 = t29.*t48.*t49.*t330.*3.0e1;
t358 = t51.*t63.*t72.*t333.*3.0e1;
t359 = t52.*t69.*t72.*t333.*3.0e1;
t360 = t53.*t72.*t73.*t333.*3.0e1;
t361 = t75.*t87.*t96.*t336.*3.0e1;
t362 = t76.*t93.*t96.*t336.*3.0e1;
t363 = t77.*t96.*t97.*t336.*3.0e1;
t364 = t352+t355+t358+t361;
t365 = -t213-t217-t221-t225+t353+t356+t359+t362;
t366 = -t169-t173-t177-t181+t354+t357+t360+t363;
t368 = t9.^2;
t369 = t6+t8+t10-t368;
t371 = t12.*t368;
t375 = t18.*t369;
t372 = t371-t375;
t373 = r3_1.*2.0;
t374 = r3_6.*2.0;
t377 = t34.^2;
t378 = t31+t33+t35-t377;
t380 = t37.*t377;
t383 = t42.*t378;
t381 = t380-t383;
t382 = r3_3.*2.0;
t385 = t58.^2;
t386 = t55+t57+t59-t385;
t388 = t61.*t385;
t391 = t66.*t386;
t389 = t388-t391;
t390 = r3_4.*2.0;
t393 = t82.^2;
t394 = t79+t81+t83-t393;
t396 = t85.*t393;
t399 = t90.*t394;
t397 = t396-t399;
t398 = r3_5.*2.0;
t400 = t2.*t15.*t17.*t369;
t401 = t2.*t15.*t17.*t368;
t402 = -t367+t400+t401;
t403 = t20.*t402.*1.0e2;
t404 = t27.*t39.*t41.*t378;
t405 = t27.*t39.*t41.*t377;
t406 = -t376+t404+t405;
t407 = t44.*t406.*1.0e1;
t408 = t51.*t63.*t65.*t386;
t409 = t51.*t63.*t65.*t385;
t410 = -t384+t408+t409;
t411 = t68.*t410.*1.0e1;
t412 = t75.*t87.*t89.*t394;
t413 = t75.*t87.*t89.*t393;
t414 = -t392+t412+t413;
t415 = t92.*t414.*1.0e1;
t416 = t3.*t17.*t21.*t369;
t417 = t3.*t17.*t21.*t368;
t418 = -t370+t416+t417;
t419 = t20.*t418.*1.0e2;
t420 = t28.*t41.*t45.*t378;
t421 = t28.*t41.*t45.*t377;
t422 = -t379+t420+t421;
t423 = t44.*t422.*1.0e1;
t424 = t52.*t65.*t69.*t386;
t425 = t52.*t65.*t69.*t385;
t426 = -t387+t424+t425;
t427 = t68.*t426.*1.0e1;
t428 = t76.*t89.*t93.*t394;
t429 = t76.*t89.*t93.*t393;
t430 = -t395+t428+t429;
t431 = t92.*t430.*1.0e1;
t432 = t373-t374;
t433 = t12.*t432;
t434 = t4.*t25.*2.0;
t435 = -t373+t374+t434;
t436 = t4.*t17.*t25.*t369;
t437 = t4.*t17.*t25.*t368;
t438 = t433+t436+t437-t18.*t435;
t439 = t20.*t438.*1.0e2;
t440 = t374-t382;
t441 = t37.*t440;
t442 = t29.*t49.*2.0;
t443 = t374-t382+t442;
t444 = t42.*t443;
t445 = t374-t390;
t446 = t61.*t445;
t447 = t53.*t73.*2.0;
t448 = t374-t390+t447;
t449 = t66.*t448;
t450 = t374-t398;
t451 = t85.*t450;
t452 = t77.*t97.*2.0;
t453 = t374-t398+t452;
t454 = t90.*t453;
tensor_K_r = reshape([t106-t2.*t15.*t23.*t24.*3.0e2,t169-t2.*t15.*t24.*t156.*3.0e2,t213-t2.*t15.*t24.*t200.*3.0e2,t169-t226,t273-t2.*t15.*t24.*t246.*3.0e2,t2.*t15.*t24.*t327.*-3.0e2,t213-t337,-t352,t403-t2.*t15.*t24.*t372.*3.0e2,t125-t3.*t21.*t23.*t24.*3.0e2,t185-t3.*t21.*t24.*t156.*3.0e2,t3.*t21.*t24.*t200.*-3.0e2,t185-t227,t293-t3.*t21.*t24.*t246.*3.0e2,t213-t3.*t21.*t24.*t327.*3.0e2,-t338,t213-t353,t419-t3.*t21.*t24.*t372.*3.0e2,t141-t4.*t23.*t24.*t25.*3.0e2,t4.*t24.*t25.*t156.*-3.0e2,t185-t4.*t24.*t25.*t200.*3.0e2,-t228,t312-t4.*t24.*t25.*t246.*3.0e2,t169-t4.*t24.*t25.*t327.*3.0e2,t185-t339,t169-t354,t439-t4.*t24.*t25.*t372.*3.0e2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t44.*(t108+t111-t27.*t39.*t40.*t41-t27.*t39.*t41.*t43).*-1.0e1-t27.*t39.*t47.*t48.*3.0e1,t173-t27.*t39.*t48.*t159.*3.0e1,t217-t27.*t39.*t48.*t203.*3.0e1,t173-t229,t277-t27.*t39.*t48.*t253.*3.0e1,t27.*t39.*t48.*t330.*-3.0e1,t217-t340,-t355,t407-t27.*t39.*t48.*t381.*3.0e1,t129-t28.*t45.*t47.*t48.*3.0e1,t189-t28.*t45.*t48.*t159.*3.0e1,t28.*t45.*t48.*t203.*-3.0e1,t189-t230,t44.*(t295+t298-t28.*t41.*t45.*t249-t28.*t41.*t45.*t250).*-1.0e1-t28.*t45.*t48.*t253.*3.0e1,t217-t28.*t45.*t48.*t330.*3.0e1,-t341,t217-t356,t423-t28.*t45.*t48.*t381.*3.0e1,t145-t29.*t47.*t48.*t49.*3.0e1,t29.*t48.*t49.*t159.*-3.0e1,t189-t29.*t48.*t49.*t203.*3.0e1,-t231,t316-t29.*t48.*t49.*t253.*3.0e1,t173-t29.*t48.*t49.*t330.*3.0e1,t189-t342,t173-t357,t44.*(t441+t444-t29.*t41.*t49.*t377-t29.*t41.*t49.*t378).*-1.0e1-t29.*t48.*t49.*t381.*3.0e1,t68.*(t113+t116-t51.*t63.*t64.*t65-t51.*t63.*t65.*t67).*-1.0e1-t51.*t63.*t71.*t72.*3.0e1,t177-t51.*t63.*t72.*t162.*3.0e1,t221-t51.*t63.*t72.*t206.*3.0e1,t177-t232,t281-t51.*t63.*t72.*t260.*3.0e1,t51.*t63.*t72.*t333.*-3.0e1,t221-t343,-t358,t411-t51.*t63.*t72.*t389.*3.0e1,t133-t52.*t69.*t71.*t72.*3.0e1,t193-t52.*t69.*t72.*t162.*3.0e1,t52.*t69.*t72.*t206.*-3.0e1,t193-t233,t68.*(t300+t303-t52.*t65.*t69.*t256-t52.*t65.*t69.*t257).*-1.0e1-t52.*t69.*t72.*t260.*3.0e1,t221-t52.*t69.*t72.*t333.*3.0e1,-t344,t221-t359,t427-t52.*t69.*t72.*t389.*3.0e1,t149-t53.*t71.*t72.*t73.*3.0e1,t53.*t72.*t73.*t162.*-3.0e1,t193-t53.*t72.*t73.*t206.*3.0e1,-t234,t320-t53.*t72.*t73.*t260.*3.0e1,t177-t53.*t72.*t73.*t333.*3.0e1,t193-t345,t177-t360,t68.*(t446+t449-t53.*t65.*t73.*t385-t53.*t65.*t73.*t386).*-1.0e1-t53.*t72.*t73.*t389.*3.0e1,t92.*(t118+t121-t75.*t87.*t88.*t89-t75.*t87.*t89.*t91).*-1.0e1-t75.*t87.*t95.*t96.*3.0e1,t181-t75.*t87.*t96.*t165.*3.0e1,t225-t75.*t87.*t96.*t209.*3.0e1,t181-t235,t285-t75.*t87.*t96.*t267.*3.0e1,t75.*t87.*t96.*t336.*-3.0e1,t225-t346,-t361,t415-t75.*t87.*t96.*t397.*3.0e1,t137-t76.*t93.*t95.*t96.*3.0e1,t197-t76.*t93.*t96.*t165.*3.0e1,t76.*t93.*t96.*t209.*-3.0e1,t197-t236,t92.*(t305+t308-t76.*t89.*t93.*t263-t76.*t89.*t93.*t264).*-1.0e1-t76.*t93.*t96.*t267.*3.0e1,t225-t76.*t93.*t96.*t336.*3.0e1,-t347,t225-t362,t431-t76.*t93.*t96.*t397.*3.0e1,t153-t77.*t95.*t96.*t97.*3.0e1,t77.*t96.*t97.*t165.*-3.0e1,t197-t77.*t96.*t97.*t209.*3.0e1,-t237,t324-t77.*t96.*t97.*t267.*3.0e1,t181-t77.*t96.*t97.*t336.*3.0e1,t197-t348,t181-t363,t92.*(t451+t454-t77.*t89.*t97.*t393-t77.*t89.*t97.*t394).*-1.0e1-t77.*t96.*t97.*t397.*3.0e1,-t106+t44.*(t108+t111-t27.*t39.*t40.*t41-t27.*t39.*t41.*t43).*1.0e1+t68.*(t113+t116-t51.*t63.*t64.*t65-t51.*t63.*t65.*t67).*1.0e1+t92.*(t118+t121-t75.*t87.*t88.*t89-t75.*t87.*t89.*t91).*1.0e1+t2.*t15.*t23.*t24.*3.0e2+t27.*t39.*t47.*t48.*3.0e1+t51.*t63.*t71.*t72.*3.0e1+t75.*t87.*t95.*t96.*3.0e1,t238,t349,t238,-t273-t277-t281-t285+t2.*t15.*t24.*t246.*3.0e2+t27.*t39.*t48.*t253.*3.0e1+t51.*t63.*t72.*t260.*3.0e1+t75.*t87.*t96.*t267.*3.0e1,t364,t349,t364,-t403-t407-t411-t415+t2.*t15.*t24.*t372.*3.0e2+t27.*t39.*t48.*t381.*3.0e1+t51.*t63.*t72.*t389.*3.0e1+t75.*t87.*t96.*t397.*3.0e1,-t125-t129-t133-t137+t3.*t21.*t23.*t24.*3.0e2+t28.*t45.*t47.*t48.*3.0e1+t52.*t69.*t71.*t72.*3.0e1+t76.*t93.*t95.*t96.*3.0e1,t239,t350,t239,-t293+t44.*(t295+t298-t28.*t41.*t45.*t249-t28.*t41.*t45.*t250).*1.0e1+t68.*(t300+t303-t52.*t65.*t69.*t256-t52.*t65.*t69.*t257).*1.0e1+t92.*(t305+t308-t76.*t89.*t93.*t263-t76.*t89.*t93.*t264).*1.0e1+t3.*t21.*t24.*t246.*3.0e2+t28.*t45.*t48.*t253.*3.0e1+t52.*t69.*t72.*t260.*3.0e1+t76.*t93.*t96.*t267.*3.0e1,t365,t350,t365,-t419-t423-t427-t431+t3.*t21.*t24.*t372.*3.0e2+t28.*t45.*t48.*t381.*3.0e1+t52.*t69.*t72.*t389.*3.0e1+t76.*t93.*t96.*t397.*3.0e1,-t141-t145-t149-t153+t4.*t23.*t24.*t25.*3.0e2+t29.*t47.*t48.*t49.*3.0e1+t53.*t71.*t72.*t73.*3.0e1+t77.*t95.*t96.*t97.*3.0e1,t240,t351,t240,-t312-t316-t320-t324+t4.*t24.*t25.*t246.*3.0e2+t29.*t48.*t49.*t253.*3.0e1+t53.*t72.*t73.*t260.*3.0e1+t77.*t96.*t97.*t267.*3.0e1,t366,t351,t366,-t439+t44.*(t441+t444-t29.*t41.*t49.*t377-t29.*t41.*t49.*t378).*1.0e1+t68.*(t446+t449-t53.*t65.*t73.*t385-t53.*t65.*t73.*t386).*1.0e1+t92.*(t451+t454-t77.*t89.*t97.*t393-t77.*t89.*t97.*t394).*1.0e1+t4.*t24.*t25.*t372.*3.0e2+t29.*t48.*t49.*t381.*3.0e1+t53.*t72.*t73.*t389.*3.0e1+t77.*t96.*t97.*t397.*3.0e1],[9,18]);