%%*******************************************************************
%% schurmat_sblk: compute Schur complement matrix corresponding to 
%%                SDP blocks. 
%%
%% symm = 0, HKM
%%      = 1, NT
%%*****************************************************************
%% SDPT3: version 4.0
%% Copyright (c) 1997 by
%% Kim-Chuan Toh, Michael J. Todd, Reha H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*****************************************************************

   function schur = schurmat_sblk(blk,At,par,schur,p,X,Y); 

   global  nnzschur nzlistschur 

   iter = par.iter; 
   smallblkdim = par.smallblkdim; 

   if isempty(smallblkdim); smallblkdim = 50; end
   if (nargin == 7); symm = 0; else; symm = 1; Y = X; end; 
   m = length(schur);    
   pblk = blk(p,:); 
   if (iter == 1)
      nnzschur(size(blk,1),1) = m*m; 
      nzlistschur = cell(size(blk,1),1); 
   end
%%
   if (max(pblk{2}) > smallblkdim) 
      %%
      %% compute schur for matrices that are very sparse. 
      %%
      m1 = size(At{p,1},2); 
      J = min(m1, max(find(par.nzlistA{p,1} < inf))-1); 
      if (J > 0)
         if issparse(X{p}) & ~issparse(Y{p}); X{p} = full(X{p}); end
         if ~issparse(X{p}) & issparse(Y{p}); Y{p} = full(Y{p}); end
         idx = max(find(par.nzlistA{p,1} == 0));  
         if ~isempty(idx); 
            maxnzschur = (m1-idx+1)^2; 
         else
            maxnzschur = m1^2; 
         end
         if (iter <= 3) 
            [schurtmp,nzcount,nzlisttmp] = mexschur_new(pblk,At{p,1},par.nzlistA{p,1},...
             par.nzlistA{p,2},par.permA(p,:),Y{p},X{p},J,symm,maxnzschur); 
            nnzschur(p)  = nzcount(1);
            if (nnzschur(p) == mexnnz(nzlisttmp)) 
               nzlistschur{p} = nzlisttmp;
            else
               nzlistschur{p} = []; 
            end
         else
            if isempty(nzlistschur{p})
               [schurtmp,nzcount] = mexschur_new(pblk,At{p,1},par.nzlistA{p,1},...
                par.nzlistA{p,2},par.permA(p,:),Y{p},X{p},J,symm,maxnzschur);
            else
	       maxnzschur = 2*mexnnz(nzlistschur{p}); 
               [schurtmp,nzcount] = mexschur_new(pblk,At{p,1},par.nzlistA{p,1},...
                par.nzlistA{p,2},par.permA(p,:),Y{p},X{p},J,symm,maxnzschur,nzlistschur{p});
            end
         end
	 schurtmp = [schurtmp(1:nzcount(2),:); m,m,0]; 
         schur = schur + spconvert(schurtmp); 
      end
      %%
      %% compute schur for matrices that are not so sparse or dense.
      %% 
      if (m1 < m) %% for low rank constraints
         ss = [0, cumsum(pblk{3})]; 
         len = sum(pblk{3});
         dd = At{p,3};
         DD = spconvert([dd(:,2:4); len,len,0]);
         XVD = X{p}*At{p,2}*DD; 
         YVD = Y{p}*At{p,2}*DD;
      end
      L = max(find(par.nzlistAsum{p,1} < inf)) -1;  
      if (J < L)
         len = par.nzlistAsum{p,1}(J+1); 
         list = par.nzlistAsum{p,2}(1:len,:); 
      end
      if (m1 > 0)
         if (J < 0.8*m)
            schurdense = 1;  
            if issparse(schur); schur = full(schur); end; 
	 else
            schurdense = 0; 
         end
	 count = 0; 
         schurtmp = [];
         for k = J+1:m 
            if (k<=m1) 
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
            else %%--- for low rank constraints
               idx = [ss(k-m1)+1 :ss(k-m1+1)]; 
               tmp = XVD(:,idx)* (Y{p}*At{p,2}(:,idx))';
            end
            if (~symm)
               tmp = 0.5*(mexsvec(pblk,tmp) + mexsvec(pblk,tmp'));
            else
               tmp = mexsvec(pblk,tmp);                
            end 
            permk = par.permA(p,k);   
            idx  = par.permA(p,1:min(k,m1));  
            schurcol = mexinprod(blk,At,tmp,min(k,m1),p); 
            if (schurdense)
               tmp2 = schur(idx,permk) + schurcol; 
               schur(idx,permk) = tmp2; 
               schur(permk,idx) = tmp2';
            else
               len = length(idx); 
               schurtmp(count+[1:len],:) = [idx,permk*ones(len,1),schurcol]; 
               count = count+len; 
            end
         end
         if (~schurdense) & (count > 0)
            schursub = spconvert([schurtmp; m,m,0]); 
            diagschur = diag(schursub); 
            schursub = schursub + schursub' - spdiags(diagschur,0,m,m); 
            schur = schur + schursub;  
         end
      end
      if (m1 < m) %%--- for low rank constraints
         m2 = m - m1;
         XVtmp = XVD'*At{p,2};
         YVtmp = At{p,2}'*YVD;
         for k = 1:m2
            idx0 = [ss(k)+1 : ss(k+1)]; 
            tmp = XVtmp(:,idx0) .* YVtmp(:,idx0);  
            tmp = tmp*ones(length(idx0),1); 
            schurcol = mexqops(pblk{3},tmp,ones(length(tmp),1),1); 
            tmp3 = schur(m1+[1:m2],m1+k) + schurcol; 
            schur(m1+[1:m2],m1+k) = tmp3; 
         end
      end
   else  
      %%--- for SDP block where each sub-block is small dimensional
      if issparse(X{p}) & ~issparse(Y{p}); Y{p} = sparse(Y{p}); end
      if ~issparse(X{p}) & issparse(Y{p}); X{p} = sparse(X{p}); end
      tmp = mexskron(pblk,X{p},Y{p});
      schurtmp = At{p,1}'*tmp*At{p,1};      
      %%--- schurtmp = 0.5*(schurtmp + schurtmp');
      if (norm(par.permA(p,:)-[1:m]) > 0)
         Perm = spconvert([(1:m)', par.permA(p,:)', ones(m,1)]); 
         schur = schur + Perm'*schurtmp*Perm;
      else
         schur = schur + schurtmp;
      end
   end
%%*******************************************************************


