function L = getLambda(R,P)
% Get transformation for kinetic energy derivatives
%   R - rotation from i-1 to i frame
%   P - position of i frame
% Return:
%   lambda matrix

    p1 = P(1);
    p2 = P(2);
    p3 = P(3);
    r1 = R(1,1:3);
    r2 = R(2,1:3);
    r3 = R(3,1:3);
    DS = [2*(p2*r2'+p3*r3'),-p1*r2'-p2*r1',-p1*r3'-p3*r1',2*(p1*r1'+p3*r3'),-p2*r3'-p3*r2',2*(p2*r2'+p1*r1')];
    Dm = [p2^2+p3^2,-p1*p2,-p1*p3,p1^2+p3^2,-p2*p3,p1^2+p2^2];
    
    DJ = [r1(1)^2,       r1(1)*r2(1),              r1(1)*r3(1),             r2(1)^2,       r2(1)*r3(1),             r3(1)^2;
          2*r1(1)*r1(2), r1(1)*r2(2)+r2(1)*r1(2),  r1(1)*r3(2)+r3(1)*r1(2), 2*r2(1)*r2(2), r2(1)*r3(2)+r3(1)*r2(2), 2*r3(1)*r3(2);
          2*r1(1)*r1(3), r2(1)*r1(3)+r1(1)*r2(3),  r3(1)*r1(3)+r1(1)*r3(3), 2*r2(1)*r2(3), r2(1)*r3(3)+r3(1)*r2(3), 2*r3(1)*r3(3);
          r1(2)^2,       r1(2)*r2(2),              r1(2)*r3(2),             r2(2)^2,       r2(2)*r3(2),             r3(2)^2;
          2*r1(2)*r1(3), r2(2)*r1(3)+r1(2)*r2(3),  r3(2)*r1(3)+r1(2)*r3(3), 2*r2(2)*r2(3), r3(2)*r2(3)+r2(2)*r3(3), 2*r3(2)*r3(3);
          r1(3)^2,       r1(3)*r2(3),              r1(3)*r3(3),             r2(3)^2,       r2(3)*r3(3),             r3(3)^2];
    
    % result
    L = [DJ, zeros(6,4);
         DS, R', zeros(3,1);
         Dm, P', 1];
end