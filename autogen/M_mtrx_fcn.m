function M_mtrx_sym = M_mtrx_fcn(in1,in2)
%M_MTRX_FCN
%    M_MTRX_SYM = M_MTRX_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    14-Nov-2019 08:53:56

q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
xi1 = in2(1,:);
xi2 = in2(2,:);
xi3 = in2(3,:);
xi4 = in2(4,:);
xi5 = in2(5,:);
xi6 = in2(6,:);
xi7 = in2(7,:);
xi8 = in2(8,:);
xi9 = in2(9,:);
xi10 = in2(10,:);
xi11 = in2(11,:);
xi12 = in2(12,:);
xi13 = in2(13,:);
xi14 = in2(14,:);
xi15 = in2(15,:);
xi16 = in2(16,:);
xi17 = in2(17,:);
xi18 = in2(18,:);
xi19 = in2(19,:);
xi20 = in2(20,:);
xi21 = in2(21,:);
xi22 = in2(22,:);
xi23 = in2(23,:);
xi24 = in2(24,:);
xi25 = in2(25,:);
xi26 = in2(26,:);
xi27 = in2(27,:);
xi28 = in2(28,:);
xi29 = in2(29,:);
xi30 = in2(30,:);
xi31 = in2(31,:);
xi32 = in2(32,:);
xi33 = in2(33,:);
xi34 = in2(34,:);
xi35 = in2(35,:);
xi36 = in2(36,:);
xi37 = in2(37,:);
xi38 = in2(38,:);
xi39 = in2(39,:);
xi40 = in2(40,:);
t2 = sin(1.57079632679);
t3 = cos(q2);
t4 = t2.^2;
t5 = sin(q2);
t6 = sin(q5);
t7 = cos(q4);
t8 = t3.^2;
t9 = sin(q3);
t10 = cos(q3);
t11 = t5.^2;
t12 = sin(q4);
t13 = t10.^2;
t14 = t9.^2;
t15 = t3.*t4.*t5.*t9.*t10.*2.0;
t16 = t7.^2;
t17 = t4.*t8.*t13;
t18 = t4.*t11.*t14;
t19 = -t15+t17+t18;
t20 = t12.^2;
t21 = t4.*t8.*t14;
t22 = t4.*t11.*t13;
t23 = t15+t21+t22;
t24 = t4.*t8.*t9.*t10.*2.0;
t25 = t13-t14;
t26 = t3.*t4.*t5.*t25.*2.0;
t29 = t4.*t9.*t10.*t11.*2.0;
t27 = t24+t26-t29;
t28 = t4.*t7.*t12.*t27;
t30 = t4.*t8.*t9.*(6.13e2./5.0e2);
t31 = t3.*t4.*t5.*t10.*(6.13e2./5.0e2);
t32 = t30+t31;
t33 = t4.*t8.*t10.*(6.13e2./5.0e2);
t50 = t3.*t4.*t5.*t9.*(6.13e2./5.0e2);
t34 = t33-t50;
t35 = cos(q5);
t36 = cos(q6);
t37 = t4.*t16.*t23;
t38 = t4.*t19.*t20;
t39 = t28+t37+t38;
t40 = sin(q6);
t41 = t4.*t16;
t46 = t4.*t20;
t42 = t41-t46;
t43 = t27.*t42;
t44 = t4.*t7.*t12.*t19.*2.0;
t47 = t4.*t7.*t12.*t23.*2.0;
t45 = t43+t44-t47;
t48 = t2.*t12.*t19.*(5.71e2./5.0e2);
t49 = t2.*t7.*t32;
t51 = t2.*t12.*t34;
t52 = t2.*t7.*t27.*(5.71e2./1.0e3);
t53 = t48+t49+t51+t52;
t54 = t4.*t8.*(1.37e2./5.0e2);
t55 = t4.*t11.*(1.37e2./5.0e2);
t56 = t54+t55-4.4e1./1.25e2;
t57 = t6.*t56;
t58 = t4.*t16.*t19;
t59 = t4.*t20.*t23;
t60 = -t28+t58+t59;
t61 = t2.*t7.*t19.*(5.71e2./5.0e2);
t62 = t2.*t7.*t34;
t70 = t2.*t12.*t32;
t71 = t2.*t12.*t27.*(5.71e2./1.0e3);
t63 = t61+t62-t70-t71;
t64 = t35.*t63;
t68 = t6.*t39.*(2.7e1./1.0e2);
t69 = t6.*t60.*(2.7e1./1.0e2);
t65 = t57+t64-t68-t69;
t66 = t35.^2;
t67 = t6.^2;
t72 = t40.^2;
t73 = t36.^2;
t74 = t2.*t3.*t10;
t81 = t2.*t5.*t9;
t75 = t74-t81;
t76 = t2.*t3.*t9;
t77 = t2.*t5.*t10;
t78 = t76+t77;
t79 = t66-t67;
t80 = t2.*t7.*t78;
t82 = t2.*t12.*t75;
t83 = t80+t82;
t84 = t2.*t7.*t75;
t91 = t2.*t12.*t78;
t85 = t84-t91;
t86 = t2.*t3.*t9.*(3.9e1./1.0e3);
t87 = t2.*t5.*t10.*(3.9e1./1.0e3);
t88 = t86+t87;
t89 = t2.*t3.*t10.*(3.9e1./1.0e3);
t93 = t2.*t5.*t9.*(3.9e1./1.0e3);
t90 = t89-t93;
t92 = t2.*t7.*t88;
t94 = t2.*t12.*t90;
t95 = t92+t94;
t96 = t2.*t7.*t90;
t97 = t2.*t7.*t75.*(2.7e1./2.0e2);
t118 = t2.*t12.*t88;
t126 = t2.*t12.*t78.*(2.7e1./2.0e2);
t98 = t96+t97-t118-t126;
t99 = t35.*t83.*(2.7e1./2.0e2);
t100 = t35.*t95;
t101 = t2.*t5.*(6.13e2./1.0e3);
t102 = t2.*t3.*t9.*(5.71e2./1.0e3);
t103 = t2.*t5.*t10.*(5.71e2./1.0e3);
t104 = t101+t102+t103;
t107 = t6.*t104;
t105 = t99+t100-t107;
t106 = t72-t73;
t108 = t36.*t79.*t83;
t109 = t35.*t40.*t85;
t110 = t108+t109;
t111 = t110.*xi31;
t112 = t35.*t36.*t85;
t136 = t40.*t79.*t83;
t113 = t112-t136;
t114 = t102+t103;
t115 = t6.*t83.*(2.7e1./2.0e2);
t116 = t6.*t95;
t117 = t83.*xi17;
t119 = t6.*t36.*t40.*t85;
t120 = t6.*t35.*t73.*t83;
t121 = t119+t120;
t122 = t121.*xi30;
t123 = t6.*t85.*t106;
t124 = t6.*t35.*t36.*t40.*t83.*2.0;
t125 = xi32.*(t123+t124);
t127 = t40.*t98;
t130 = t6.*t114;
t128 = t99+t100-t130;
t129 = t36.*t98;
t131 = t6.*t40.*t85.*(3.0./2.5e1);
t132 = t90.*xi14;
t133 = t88.*xi15;
t134 = t79.*t83.*xi24;
t135 = t6.*t35.*t83.*xi23;
t137 = t96-t118;
t138 = t137.*xi22;
t139 = t99+t100;
t140 = t2.*t3.*(6.13e2./1.0e3);
t141 = t2.*t3.*t10.*(5.71e2./1.0e3);
t144 = t2.*t5.*t9.*(5.71e2./1.0e3);
t142 = t140+t141-t144;
t143 = t6.*t98;
t146 = t35.*t142;
t145 = t143-t146;
t147 = t35.*t98;
t148 = t6.*t142;
t149 = t2.*t7.*t78.*(2.7e1./2.0e2);
t150 = t2.*t12.*t75.*(2.7e1./2.0e2);
t151 = t92+t94+t149+t150;
t270 = t6.*t83.*(3.0./2.5e1);
t152 = t147+t148-t270;
t153 = t36.*t105;
t199 = t6.*t36.*t85.*(3.0./2.5e1);
t154 = t127+t153-t199;
t155 = t129+t131-t40.*t105;
t156 = t155.*xi36;
t157 = t35.*t104;
t158 = t115+t116+t157;
t159 = t158.*xi29;
t160 = t2.*t5.*xi8.*(2.2e1./1.25e2);
t161 = t2.*t3.*xi7.*(2.2e1./1.25e2);
t162 = t2.*t12.*(5.71e2./5.0e2);
t163 = t2.*t10.*t12.*(6.13e2./5.0e2);
t164 = t2.*t7.*t9.*(6.13e2./5.0e2);
t165 = t162+t163+t164;
t166 = t2.*t7.*(5.71e2./5.0e2);
t167 = t2.*t7.*t10.*(6.13e2./5.0e2);
t169 = t2.*t9.*t12.*(6.13e2./5.0e2);
t168 = t166+t167-t169;
t170 = t66.*xi33;
t171 = t67.*xi23;
t172 = t36.*t66.*(6.0./2.5e1);
t173 = t36.*t67.*(6.0./2.5e1);
t174 = t2.*t10.*t12.*(6.13e2./1.0e3);
t175 = t2.*t7.*t9.*(6.13e2./1.0e3);
t176 = t162+t174+t175;
t177 = t2.*t7.*t10.*(6.13e2./1.0e3);
t181 = t2.*t9.*t12.*(6.13e2./1.0e3);
t178 = t166+t177-t181;
t179 = t40.*t66.*(6.0./2.5e1);
t180 = t40.*t67.*(6.0./2.5e1);
t182 = t67.*t73.*xi30;
t183 = t6.*t35.*xi24.*2.0;
t184 = t6.*t35.*t36.*xi31.*2.0;
t185 = t6.*t35.*t40.*xi34.*2.0;
t186 = t36.*t40.*t67.*xi32.*2.0;
t187 = t10.*(6.13e2./1.0e3);
t188 = t187+5.71e2./1.0e3;
t189 = t2.*t12.*t188;
t190 = t175+t189;
t192 = t2.*t7.*t188;
t191 = t181-t192;
t193 = t35.*(3.0./2.5e1);
t277 = t35.*t190;
t194 = t193-t277;
t195 = t35.*t114;
t196 = t115+t116+t195;
t197 = t196.*xi29;
t198 = t36.*t128;
t200 = t129+t131-t40.*t128;
t201 = t200.*xi36;
t202 = t10.*xi15.*(6.13e2./1.0e3);
t203 = t172+t173-t36.*t176-t35.*t40.*t178;
t204 = t203.*xi36;
t205 = t35.*t36.*t178;
t206 = t179+t180+t205-t40.*t176;
t207 = t6.*t178.*xi29;
t208 = t170+t171+t182+t183+t184+t185+t186+t202+t204+t207+xi12+xi19-t9.*xi14.*(6.13e2./1.0e3)-t176.*xi22-t178.*xi21-t206.*xi35-t35.*t178.*xi28;
t209 = t6.*t36.*(3.0./2.5e1);
t210 = t35.*xi26;
t211 = t6.*t40.*(3.0./2.5e1);
t212 = t6.*xi25;
t213 = t35.*t36.*xi34;
t214 = t35.*xi33;
t283 = t2.*t12.*t35.*(5.71e2./1.0e3);
t215 = t193-t283;
t216 = t6.*t36.*xi31;
t217 = t6.*t40.*xi34;
t218 = t115+t116;
t219 = t218.*xi29;
t220 = t36.*t139;
t221 = t129+t131-t40.*t139;
t222 = t221.*xi36;
t223 = t35.*t40.*t191;
t224 = xi36.*(t172+t173+t223-t36.*t190);
t225 = t179+t180-t40.*t190-t35.*t36.*t191;
t226 = xi21.*(t181-t192);
t227 = t35.*xi28.*(t181-t192);
t228 = t170+t171+t182+t183+t184+t185+t186+t224+t226+t227+xi19-t190.*xi22-t225.*xi35-t6.*t191.*xi29;
t229 = t172+t173-t2.*t12.*t36.*(5.71e2./1.0e3)-t2.*t7.*t35.*t40.*(5.71e2./1.0e3);
t230 = t229.*xi36;
t231 = t2.*t7.*t35.*t36.*(5.71e2./1.0e3);
t232 = t179+t180+t231-t2.*t12.*t40.*(5.71e2./1.0e3);
t233 = t2.*t6.*t7.*xi29.*(5.71e2./1.0e3);
t234 = t170+t171+t182+t183+t184+t185+t186+t230+t233+xi19-t232.*xi35-t2.*t7.*xi21.*(5.71e2./1.0e3)-t2.*t12.*xi22.*(5.71e2./1.0e3)-t2.*t7.*t35.*xi28.*(5.71e2./1.0e3);
t235 = t36.*t40.*t85.*2.0;
t236 = t235-t35.*t83.*t106;
t237 = t236.*xi32;
t238 = t72.*t85;
t239 = t35.*t36.*t40.*t83;
t240 = t238+t239;
t241 = t147+t148;
t242 = t36.*t145;
t243 = t35.*t36.*t83.*(3.0./2.5e1);
t244 = t242+t243;
t245 = t40.*t145;
t246 = t35.*t40.*t83.*(3.0./2.5e1);
t247 = t245+t246;
t248 = t35.*t83.*xi25;
t249 = t6.*t40.*t83.*xi31;
t250 = t237+t248+t249-t85.*xi27-t145.*xi28-t240.*xi30-t241.*xi29-t244.*xi35-t247.*xi36-t6.*t83.*xi26-t6.*t36.*t83.*xi34;
t251 = t209-t6.*t36.*t190;
t252 = t211-t6.*t40.*t190;
t253 = t35.*t190.*xi29;
t254 = t6.*t190.*xi28;
t258 = t35.*t40.*xi31;
t259 = t6.*t106.*xi32;
t260 = t6.*t36.*t40.*xi30;
t255 = t210+t212+t213+t253+t254-t258-t259-t260-t251.*xi35-t252.*xi36;
t256 = t209-t2.*t6.*t12.*t36.*(5.71e2./1.0e3);
t257 = t211-t2.*t6.*t12.*t40.*(5.71e2./1.0e3);
t261 = t2.*t12.*t35.*xi29.*(5.71e2./1.0e3);
t262 = t2.*t6.*t12.*xi28.*(5.71e2./1.0e3);
t263 = t40.*t85;
t264 = t35.*t36.*t83;
t265 = t263+t264;
t266 = t265.*xi31;
t267 = t36.*t85;
t268 = t267-t35.*t40.*t83;
t269 = t36.*t151;
t271 = t40.*t152;
t272 = t269+t271;
t273 = t36.*t152;
t274 = xi36.*(t273-t40.*t151);
t275 = t266+t274-t268.*xi34-t272.*xi35-t6.*t83.*xi33;
t276 = t36.*t191;
t278 = xi35.*(t276-t40.*t194);
t279 = t40.*t191;
t280 = t36.*t194;
t281 = xi36.*(t279+t280);
t282 = t214+t216+t217+t278+t281;
t284 = t40.*t215;
t285 = t2.*t7.*t36.*(5.71e2./1.0e3);
t286 = t284+t285;
t287 = t36.*t215;
t288 = t287-t2.*t7.*t40.*(5.71e2./1.0e3);
t289 = t288.*xi36;
t290 = t214+t216+t217+t289-t286.*xi35;
t291 = t35.*t36.*xi36.*(3.0./2.5e1);
t292 = t214+t216+t217+t291-t35.*t40.*xi35.*(3.0./2.5e1);
t293 = t36.*xi34;
t294 = t293-t40.*xi31;
M_mtrx_sym = reshape([xi1-xi31.*(t6.*t40.*t45+t6.*t35.*t36.*t39.*2.0)+xi34.*(t6.*t36.*t45-t6.*t35.*t39.*t40.*2.0)+t19.*xi9+t27.*xi11-t32.*xi14+t34.*xi15+t39.*xi16-t45.*xi18-t53.*xi22-t63.*xi21+t60.*xi27-t65.*xi28+xi29.*(t6.*t63+t35.*t39.*(2.7e1./1.0e2)-t35.*t56+t35.*t60.*(2.7e1./1.0e2))+xi30.*(t60.*t72+t39.*t66.*t73+t35.*t36.*t40.*t45)+xi32.*(t36.*t40.*t60.*-2.0+t35.*t45.*t106+t36.*t39.*t40.*t66.*2.0)+xi36.*(-t36.*t53-t40.*t65+t35.*t40.*t45.*(3.0./2.5e1)+t36.*t39.*t66.*(6.0./2.5e1)+t36.*t39.*t67.*(6.0./2.5e1))-xi35.*(-t40.*t53+t36.*t65-t35.*t36.*t45.*(3.0./2.5e1)+t39.*t40.*t66.*(6.0./2.5e1)+t39.*t40.*t67.*(6.0./2.5e1))+t4.*t8.*xi2+t6.*t45.*xi26-t35.*t45.*xi25+t39.*t66.*xi23+t39.*t67.*xi33+t3.*t4.*t5.*xi4.*2.0-t6.*t35.*t39.*xi24.*2.0,t111+t117+t122+t125+t132+t133+t134+t135+t138+t156+t159+t160+t161-t75.*xi10-t78.*xi13-t85.*xi20-t95.*xi21-t105.*xi28-t113.*xi34-t154.*xi35-t2.*t3.*xi3-t2.*t5.*xi6-t6.*t85.*xi25-t35.*t85.*xi26-t6.*t35.*t83.*xi33,t111+t117+t122+t125+t132+t133+t134+t135+t138+t197+t201-t75.*xi10-t78.*xi13-t85.*xi20-t95.*xi21-t113.*xi34-t128.*xi28-xi35.*(t127+t198-t199)-t6.*t85.*xi25-t35.*t85.*xi26-t6.*t35.*t83.*xi33,t111+t117+t122+t125+t134+t135+t138+t219+t222-t85.*xi20-t95.*xi21-t113.*xi34-t139.*xi28-xi35.*(t127-t199+t220)-t6.*t85.*xi25-t35.*t85.*xi26-t6.*t35.*t83.*xi33,t250,t275,t111+t117+t122+t125+t132+t133+t134+t135+t156+t159+t160+t161-t75.*xi10-t78.*xi13-t85.*xi20-t95.*xi21-t105.*xi28-t113.*xi34-t154.*xi35+xi22.*(t96-t2.*t12.*t88)-t2.*t3.*xi3-t2.*t5.*xi6-t6.*t85.*xi25-t35.*t85.*xi26-t6.*t35.*t83.*xi33,t170+t171+t182+t183+t184+t185+t186+xi5+xi12+xi19+xi36.*(t172+t173-t36.*t165-t35.*t40.*t168)-xi35.*(t179+t180-t40.*t165+t35.*t36.*t168)-t9.*xi14.*(6.13e2./5.0e2)+t10.*xi15.*(6.13e2./5.0e2)-t165.*xi22-t168.*xi21+t6.*t168.*xi29-t35.*t168.*xi28,t208,t228,t255,t282,t111+t117+t122+t125+t132+t133+t134+t135+t138+t197+t201-t75.*xi10-t78.*xi13-t85.*xi20-t95.*xi21-t113.*xi34-t128.*xi28-xi35.*(t127+t198-t6.*t36.*t85.*(3.0./2.5e1))-t6.*t85.*xi25-t35.*t85.*xi26-t6.*t35.*t83.*xi33,t208,t170+t171+t182+t183+t184+t185+t186+xi12+xi19+xi37+xi36.*(t172+t173-t2.*t12.*t36.*(5.71e2./5.0e2)-t2.*t7.*t35.*t40.*(5.71e2./5.0e2))-xi35.*(t179+t180-t2.*t12.*t40.*(5.71e2./5.0e2)+t2.*t7.*t35.*t36.*(5.71e2./5.0e2))-t2.*t7.*xi21.*(5.71e2./5.0e2)-t2.*t12.*xi22.*(5.71e2./5.0e2)+t2.*t6.*t7.*xi29.*(5.71e2./5.0e2)-t2.*t7.*t35.*xi28.*(5.71e2./5.0e2),t234,t210+t212+t213-t258-t259-t260+t261+t262-t256.*xi35-t257.*xi36,t290,t111+t117+t122+t125+t134+t135+t138+t219+t222-t85.*xi20-t95.*xi21-t113.*xi34-t139.*xi28-xi35.*(t127+t220-t6.*t36.*t85.*(3.0./2.5e1))-t6.*t85.*xi25-t35.*t85.*xi26-t6.*t35.*t83.*xi33,t228,t234,t170+t171+t182+t183+t184+t185+t186+xi19+xi38+xi36.*(t172+t173)-xi35.*(t179+t180),t210+t212+t213-t258-t259-t260-t6.*t36.*xi35.*(3.0./2.5e1)-t6.*t40.*xi36.*(3.0./2.5e1),t292,t250,t255,t210+t212+t213+t261+t262-t256.*xi35-t257.*xi36-t35.*t40.*xi31-t6.*t106.*xi32-t6.*t36.*t40.*xi30,t210+t212+t213-t6.*t36.*xi35.*(3.0./2.5e1)-t6.*t40.*xi36.*(3.0./2.5e1)-t35.*t40.*xi31-t6.*t106.*xi32-t6.*t36.*t40.*xi30,xi27+xi39+t72.*xi30-t36.*t40.*xi32.*2.0,t294,t275,t282,t290,t292,t294,xi33+xi40],[6,6]);