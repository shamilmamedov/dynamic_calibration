function out = inertiaMatrix2Vector(I)
% -----------------------------------------------------------------------
% This function conerts ineria matrix of the link to vector in 
% 'Gautier-Khalil' notation (because it is used in all their papers)
% Input:
%   I - 3x3 inertia matrix of the link
% Output:
%   out - [Ixx Ixy Ixz Iyy Iyz Izz]
% -----------------------------------------------------------------------
    out = [I(1,1) I(1,2) I(1,3) I(2,2) I(2,3) I(3,3)]';
