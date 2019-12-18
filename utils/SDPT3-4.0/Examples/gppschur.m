%%****************************************************
%% gppschur: compute schur matrix of HKM direction
%%           for GPP problems.  
%%
%% schur = gppschur(X,Zinv,schurfun_par); 
%%
%%  Ak = -e*e'   if k = 1
%%     = -ek*ek' if k > 1. 
%%*****************************************************************
%% SDPT3: version 4.0
%% Copyright (c) 1997 by
%% Kim-Chuan Toh, Michael J. Todd, Reha H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*****************************************************************
  function schur = gppschur(X,Zinv,schurfun_par); 
    
   n = size(X,1); 
   e = ones(n,1); 
   m = n+1; 
   schur = zeros(m); 
   schur(2:m,2:m) = X .* Zinv;  
   tmp = (X*e) .* (Zinv*e); 
   schur(2:m,1) = tmp; 
   schur(1,2:m) = tmp'; 
   schur(1,1)   = (e'*X*e) * (e'*Zinv*e);
%%****************************************************
