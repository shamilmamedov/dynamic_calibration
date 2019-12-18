%%*******************************************************************
%% schurmat_sblk: compute Schur complement matrix corresponding to 
%%                SDP blocks. 
%%
%% symm = 0, HKM
%%      = 1, NT
%%
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%*******************************************************************

   function schur = schurmat_sblk(blk,At,par,schur,p,X,Y); 

   global smallblkdim
 
   if (nargin == 7); symm = 0; else; symm = 1; Y = X; end; 
   m = length(schur);    
   pblk = blk(p,:); 
%%
   if (max(pblk{2}) > smallblkdim) 
      %%
      %% compute schur for matrices that are very sparse. 
      %%
      if issparse(schur); schur = full(schur); end;  
      J = max(find(par.nzlistA{p,1} < inf)) -1; 
      if (J > 0)
         if issparse(X{p}) & ~issparse(Y{p}); X{p} = full(X{p}); end
         if ~issparse(X{p}) & issparse(Y{p}); Y{p} = full(Y{p}); end
         nnzspschur = mexschur(pblk,At{p},par.nzlistA{p,1},...
             par.nzlistA{p,2},par.permA(p,:),Y{p},X{p},J,symm,schur);
      end
      %%
      %% compute schur for matrices that are not so sparse or dense.
      %% 
      L = max(find(par.nzlistAsum{p,1} < inf)) -1;  
      if (J < L)
         len = par.nzlistAsum{p,1}(J+1); list = par.nzlistAsum{p,2}(1:len,:); 
      end
      for k = J+1:m 
         isspAk = par.isspA(p,k);
         Ak = mexsmat(blk,At,isspAk,p,k); 
         if (k <= L)
            idx1 = par.nzlistAsum{p,1}(k)+1; idx2 = par.nzlistAsum{p,1}(k+1);
            list = [list; par.nzlistAsum{p,2}(idx1:idx2,:)]; 
            list = sortrows(list,[2 1]); 
            tmp = Prod3(pblk,X{p},Ak,Y{p},symm,list); 
         else
            tmp = Prod3(pblk,X{p},Ak,Y{p},symm);
         end
         if ~symm
            tmp = 0.5*(mexsvec(pblk,tmp) + mexsvec(pblk,tmp,[],1));
         else
            tmp = mexsvec(pblk,tmp); 
         end
         permk = par.permA(p,k);   
         idx  = par.permA(p,1:k);  
         tmp2 = schur(idx,permk) + mexinprod(blk,At,tmp,k,p); 
         schur(idx,permk) = tmp2; 
         schur(permk,idx) = tmp2';  
      end
   else
      if issparse(X{p}) & ~issparse(Y{p}); Y{p} = sparse(Y{p}); end
      if ~issparse(X{p}) & issparse(Y{p}); X{p} = sparse(X{p}); end
      tmp  = mexskron(pblk,X{p},Y{p});
      Perm = spconvert([(1:m)' par.permA(p,:)' ones(m,1)]); 
      schurtmp = At{p}'*tmp*At{p};      
      schurtmp = 0.5*(schurtmp + schurtmp');
      schur = schur + Perm'*schurtmp*Perm;
   end
%%*******************************************************************
