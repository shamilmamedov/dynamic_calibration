function [Rpr,Rcur] = group(Ppr,Pcur,lam,zi)
% Regroup inertial parameters
%   Ppr - parameters for link i-1
%   Pcur - parameters for link i
%   lam - lambda matrix for link i
%   zi  - axis of rotation for lin i
% Return:
%   Rpr - regrouped parameters for link i-1
%   Rcur - regrouped parameters for link i
    
    Rcur = Pcur;
% --------------------------------------------------------------------    
%   mass
% --------------------------------------------------------------------
    Rpr = Ppr + Pcur(10)*lam(10,:)';
    Rcur(10) = 0;
% --------------------------------------------------------------------    
%   mass center and inertia 
% --------------------------------------------------------------------
    if zi(3) == 1  % Z rotation, add mZ, combine XX and YY
        sum = simplify(lam(1,:) + lam(4,:));
        Rpr = Rpr + sum' * Pcur(4) + lam(9,:)' * Pcur(9);
        Rcur(1) = Pcur(1) - Pcur(4);
        Rcur(4) = 0;
        Rcur(9) = 0;
    elseif zi(2) == 1 % Y rotation, add mY, combine XX and ZZ
        sum = simplify(lam(1,:) + lam(6,:));
        Rpr = Rpr + sum' * Pcur(6) + lam(8,:)' * Pcur(8);
        Rcur(1) = Pcur(1) - Pcur(6);
        Rcur(6) = 0;
        Rcur(8) = 0;
    elseif zi(1) == 1 % X rotation, add mZ, combine YY and ZZ
        sum = simplify(lam(4,:) + lam(6,:));
        Rpr = Rpr + sum' * Pcur(6) + lam(7,:)' * Pcur(7);
        Rcur(4) = Pcur(4) - Pcur(6);
        Rcur(6) = 0;
        Rcur(7) = 0;
    else
        disp("Wrong axis definition");
    end
end 