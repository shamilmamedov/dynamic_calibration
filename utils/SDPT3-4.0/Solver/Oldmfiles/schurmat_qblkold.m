%%*******************************************************************
%% schurmat_qblk: compute schur matrix corresponding to SOCP blocks.
%%
%% HKM direction: output = schur + Ax*Ae' + Ae*Ax' - Ad*Ad'
%% NT  direction: output = schur + Ae*Ae' - Ad*Ad'
%%
%% where schur = A*D*A', and Ad is the modification to ADA'
%% so that the latter is positive definite.
%%
%% [schur,UU,EE] = schurmat_qblk(blk,At,schur,UU,EE,p,dd,ee,xx);
%%
%% UU: stores the dense columns of Ax, Ae, Ad, and possibly
%%     those of A*D^{1/2}. It has the form UU = [Ax Ae Ad].
%% EE: stores the assocaited (2,2) block matrix when the
%%     output matrix is expressed as an augmented matrix.
%%     It has the form EE = [0 -lam 0; -lam 0 0; 0 0 I].
%%
%% options = 0, HKM
%%         = 1, NT
%% 
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 10 Jun 02
%%*******************************************************************

   function [schur,UU,VV] = schurmat_qblk(blk,At,schur,UU,VV,p,dd,ee,xx);
   
   if (nargin == 9); options = 0; else; options = 1; end; 
      
   pblk = blk(p,:); n = sum(pblk{2});  numblk = length(pblk{2});
   
   Ae = qprod(pblk,At{p}',ee{p}); 
   if (options == 0) 
      Ax = qprod(pblk,At{p}',xx{p}); 
   end; 
   decolidx = checkdense(Ae);
   ddsch = dd{p};    
   if ~isempty(decolidx);        
      spcolidx = setdiff([1:numblk],decolidx); 
      s = 1 + [0 cumsum(pblk{2})];
      idx = s(decolidx); 
      tmp = zeros(n,1); 
      tmp(idx) = sqrt(2*abs(ddsch(idx))); 
      Ad = qprod(pblk,At{p}',tmp); 
      ddsch(idx) = abs(ddsch(idx)); 
      if (options == 0) 
         UU = [UU Ax(:,decolidx) Ae(:,decolidx)  Ad]; 
         VV = [VV Ae(:,decolidx) Ax(:,decolidx) -Ad]; 
         Ax = Ax(:,spcolidx); Ae = Ae(:,spcolidx); 
         schur = schur + Ax*Ae'+ Ae*Ax';         
      else
         UU = [UU Ae(:,decolidx)  Ad]; 
         VV = [VV Ae(:,decolidx) -Ad]; 
         Ae = Ae(:,spcolidx);      
         schur = schur + Ae*Ae';
      end
   else
      if (options == 0)
         schur = schur + Ax*Ae'+ Ae*Ax';
      else 
         schur = schur + Ae*Ae';
      end
   end
   decolidx = checkdense(At{p}'); 
   if ~isempty(decolidx); 
      len = length(decolidx);               
      tmp = (spdiags(sqrt(abs(ddsch(decolidx))),0,len,len)*At{p}(decolidx,:))'; 
      UU = [UU tmp]; 
      VV = [VV tmp*spdiags(sign(ddsch(decolidx)),0,len,len)]; 
      ddsch(decolidx) = zeros(len,1); 
   end  
   schur = schur + At{p}' *spdiags(ddsch,0,n,n) *At{p}; 
%%*******************************************************************
