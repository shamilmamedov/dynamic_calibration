function m = getMu(R,P)
% Get transformation for potential energy derivatives
%   R - rotation from i-1 to i frame
%   P - position of i frame
% Return:
%   transformation matrix

    m = [zeros(6,10);
         zeros(3,6),R',zeros(3,1);
         zeros(1,6),P',1];
end