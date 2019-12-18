%%*******************************************************************
%% schurmat_lblk: 
%%
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%*******************************************************************

   function [schur,UU,VV] = schurmat_lblk(blk,At,schur,UU,VV,p,dd);
   
   n = sum(blk{p,2});  
   decolidx = checkdense(At{p}'); 
   ddsch = dd{p}; 
   if ~isempty(decolidx); 
      len = length(decolidx); 
      Ad = (spdiags(sqrt(ddsch(decolidx)),0,len,len)*At{p}(decolidx,:))'; 
      UU = [UU Ad]; 
      VV = [VV Ad]; 
      ddsch(decolidx) = zeros(len,1); 
   end
   schur = schur + At{p}' *spdiags(ddsch,0,n,n) *At{p}; 
%%*******************************************************************
