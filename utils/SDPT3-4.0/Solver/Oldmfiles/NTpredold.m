%%**********************************************************************
%% NTpred: Compute (dX,dy,dZ) for NT direction. 
%%                       
%% compute SVD of Xchol*Zchol via eigenvalue decompostion of
%%     Zchol * X * Zchol' = V * diag(sv2) * V'. 
%% compute W satisfying W*Z*W = X. 
%%     W = G'*G,  where G = diag(sqrt(sv)) * (invZchol*V)'
%%
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%**********************************************************************

 function [dX,dy,dZ,schur,L,UU,VV,E,SMWmat,hRd] = ...
          NTpred(blk,At,par,rp,Rd,sigmu,X,Z,Zchol,invZchol);

    global matlabversion ispc_hp_ibm
    global spdensity iter solve_ok existMA47symb existspcholsymb
    global nnzmat nnzmatold matfct_options matfct_options_old use_SCM
    global Lsymb cachesize  
    global W G sv gamx gamz ff dd ee 
%%
%% compute NT scaling matrix
%%
    [W,G,sv,gamx,gamz,dd,ee,ff] = NTscaling(blk,X,Z,Zchol,invZchol);
%%
%% compute schur matrix
%%
    m = length(rp); 
    schur = sparse(m,m); 
    UU = []; VV = []; E = []; SMWmat = []; Afree = []; 
    existsdpblk = 0; exist_free_vars = 0;
    dX = cell(size(blk,1),1); dy = []; dZ = cell(size(blk,1),1); 
%%
    for p = 1:size(blk,1)
        pblk = blk(p,:); 
        if strcmp(pblk{1},'l')
           [schur,UU,VV] = schurmat_lblkold(blk,At,schur,UU,VV,p,dd);
        elseif strcmp(pblk{1},'q');       
           [schur,UU,VV] = schurmat_qblkold(blk,At,schur,UU,VV,p,dd,ee);
        elseif strcmp(pblk{1},'s')   
           existsdpblk = 1; 
           schur = schurmat_sblk(blk,At,par,schur,p,W); 
        elseif strcmp(pblk{1},'u') 
           exist_free_vars = 1; 
           Afree = [Afree At{p}']; 
        end
    end
    diagschur = diag(schur); 
    if (existsdpblk)
       pertdiag = 1e-16*sqrt(m)*norm(diagschur);  
       for k=1:m; schur(k,k) = abs(schur(k,k)) + pertdiag; end; 
    else 
       pertdiag = 1e-16*max(abs(diagschur));    
       for k=1:m; schur(k,k) = abs(schur(k,k)) + pertdiag; end; 
    end
%%
    [rhs,EinvRc,hRd] = NTrhsfun(blk,At,par,X,Z,rp,Rd,sigmu);
%%
%%  determine matrix factorization to use 
%%
    if (exist_free_vars)
       tmp = [zeros(size(Afree,2),1); ones(size(UU,2),1)];  
       UU = [Afree UU]; VV = [Afree VV]; 
       E = spdiags(tmp,0,length(tmp),length(tmp)); 
    else
       E = speye(size(UU,2)); 
    end
    rhs = [rhs; zeros(m+size(UU,2)-length(rhs),1)]; 
    if (iter <= 2)
       if (size(UU,2) < min(500,0.1*m)); 
          use_SCM = 1; 
       else
          use_SCM = 0; 
          if (norm(UU-VV,'fro') < 1e-13) & (size(UU,2) < 0.25*m) & (existMA47symb)
             matfct_options = 'MA47';
          else
             matfct_options = 'splu';
          end
       end
    end
%%
%% Cholesky factorization
%%
    if (iter == 1); existMA47symb = 1; nnzmatold = 0; end
    nnzmat = mexnnz(schur); 
    nnzmatdiff = (nnzmat ~= nnzmatold); 
    if use_SCM
       if (nnzmat > spdensity*m^2)
          if issparse(schur); schur = full(schur); end;
          matfct_options = 'chol';
       else 
          if ~issparse(schur); schur = sparse(schur); end; 
          matfct_options = 'spchol'; 
       end
       if strcmp(matfct_options,'chol') 
          L = []; 
          if (matlabversion >= 6)
             if (ispc_hp_ibm), [schur,indef] = chol(schur); 
             else, indef = mexchol_(schur); 
             end     
          else
             [schur,indef] = chol(schur); 
          end
          if indef; 
             solve_ok = -2; 
             fprintf('  chol: Schur complement matrix not positive definite.\n');  
          else 
             [xx,solve_ok,SMWmat] = schursysolve(schur,schur,UU,VV,E,rhs,matfct_options);
          end
       elseif strcmp(matfct_options,'spchol')
          if (nnzmatdiff | ~strcmp(matfct_options,matfct_options_old))
             [Lsymb,flag] = symbcholfun(schur,cachesize);
             if (flag) 
                solve_ok = -2; 
                existspcholsymb = 0; 
                fprintf('  spchol: symbolic factorization fails.\n'); 
             else 
                existspcholsymb = 1;
             end
          end      
          if (existspcholsymb)     
             L = sparcholfun(Lsymb,schur);
             L.d(find(L.skip)) = inf;  
             if ~isempty(UU) & any(L.skip)
                solve_ok = -3; 
                fprintf('  spchol: fail to solve Schur complement equation\n'); 
                fprintf('          via the Sherman-Morrison update.'); 
             else
                [xx,solve_ok,SMWmat] = schursysolve(schur,L,UU,VV,E,rhs,matfct_options);
             end
          end
       end
    end
    if (solve_ok <= 0) 
       if ~(existsdpblk & size(UU,2)==0)
          use_SCM = 0; 
          matfct_options = 'splu';
          if (size(UU,2) > 0) 
             fprintf('  switch to augmented system.\n');
          else
             fprintf('  switch to LU factorization.\n'); 
          end
       else
          return;
       end
    end
%%
%% LU or symmetric indefinite factorization
%%
    if ~use_SCM
       raugmat = [schur UU; VV' -E];  
       if (nnzmat > spdensity*m^2); 
          matfct_options = 'lu';
       end
       %%
       if strcmp(matfct_options,'lu') 
          if issparse(raugmat); raugmat = full(raugmat); end  
          [L.l,L.u,L.p] = lu(raugmat);
          xx = L.u \ (L.l \ (L.p*rhs));
       end
       if strcmp(matfct_options,'MA47')
          if ~issparse(raugmat); raugmat = sparse(raugmat); end      
          if (nnzmatdiff | ~strcmp(matfct_options,matfct_options_old))
             nnztmp = (nnz(raugmat) + nnz(diag(raugmat)))/2; 
             [keep,Jcn,flag,info] = mexMA47syb(nnztmp,raugmat);
             if (flag)
                existMA47symb = 0; 
                fprintf('  MA47: symbolic factorization fails, switch to LU.\n'); 
                matfct_options = 'splu';
             else 
                existMA47symb = 1;  
                Lsymb.keep = keep;    Lsymb.Jcn = Jcn; 
                Lsymb.La = 2*info(1); Lsymb.Liw = 2*info(2);  
             end 
          end
	  if (existMA47symb)
             [L.a,L.iw,flag] = mexMA47fct(raugmat, ...
             Lsymb.keep,Lsymb.Jcn,Lsymb.La,Lsymb.Liw,[1e-10,0]); 
             xx = mexMA47slv(length(rhs),L.a,L.iw,rhs);
             if (flag == 0)
                solve_ok = 1; 
             else
  	        existMA47symb = 0; 
                fprintf('  MA47: solver fails, switch to LU. \n');
                matfct_options = 'splu';
             end 
          end 
       end
       if strcmp(matfct_options,'splu')
          if ~issparse(raugmat); raugmat = sparse(raugmat); end
          if (nnzmatdiff | ~strcmp(matfct_options,matfct_options_old))
             Lsymb.perm = symmmd(raugmat); 
          end        
          L.perm = Lsymb.perm;
          if (matlabversion >= 6.5)
             [L.l,L.u,L.p,L.q] = lu(raugmat(L.perm,L.perm));
	  elseif (exist('umfpack2')==3)
             [L.l,L.u,pp,qq] = umfpack2(raugmat(L.perm,L.perm));
             len = length(pp); 
             L.p = spconvert([(1:len)' pp' ones(len,1)]); 
             L.q = spconvert([qq' (1:len)' ones(len,1)]);
          else
             [L.l,L.u,L.p] = lu(raugmat(L.perm,L.perm));
             L.q = speye(length(raugmat)); 
          end
          xx(L.perm,1) = L.q*( L.u \ (L.l \ (L.p*rhs(L.perm))));
          if any(xx == inf)  
             solve_ok = 0;
             fprintf('  LU: fails to solve the augmented system.\n');
          else
   	     solve_ok = 1; 
          end
       end
    end
    nnzmatold = nnzmat; matfct_options_old = matfct_options; 
%%
%% compute (dX,dZ)
%%
    if (any(isnan(xx)) | any(isinf(xx)))
       error('  solution of linear system contains NaN or inf'); 
    end
    [dX,dy,dZ] = NTdirfun(blk,At,par,Rd,EinvRc,xx); 
%%**********************************************************************

