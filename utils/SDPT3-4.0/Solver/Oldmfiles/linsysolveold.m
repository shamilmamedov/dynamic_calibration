%%***************************************************************
%% linsysolve: solve linear system to get dy, and direction
%%             corresponding to unrestricted variables. 
%%
%% [xx,coeff,L,resnrm] = linsysolve(schur,UU,Afree,EE,rhs); 
%%
%% child functions: symqmr.m, mybicgstable.m, linsysolvefun.m
%%
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%***************************************************************
   
   function [xx,coeff,L,resnrm] = linsysolve(par,schur,UU,Afree,EE,rhs); 
   
    global solve_ok existspcholsymb
    global nnzmat nnzmatold matfct_options matfct_options_old use_LU
    global Lsymb 
    %%global switch2LU existMA47symb 

    spdensity  = par.spdensity;
    printlevel = par.printlevel; 
    iter       = par.iter; 

    if (iter==1); use_LU = 0; end
    %%if (iter==1); existMA47symb = 1; switch2LU = 0; end
    if isempty(nnzmatold); nnzmatold = 0; end
%%
%% schur = schur + rho*diagschur + lam*AAt
%%
    m = length(schur); 
    diagschur = max(0,full(diag(schur))); 
    minrho(1) = max(1e-15, 1e-4/3.0^iter); 
    minlam(1) = max(1e-10, 1e-2/2.0^iter); 
    minrho(2) = max(1e-04, 1/1.5^iter);
    rho = min(minrho(1),minrho(2)*(1+norm(rhs))/(1+norm(diagschur.*par.y)));
    lam = min(minlam(1),0.1*rho*norm(diagschur)/par.normAAt);
    mexschurfun(schur,rho*diagschur); 
    mexschurfun(schur,lam*par.AAt); 
    fprintf(' %2.1e %2.1e ',rho,lam); 
    %%
    %% old strategy
    %%
    %%diagschur = full(abs(diag(schur)));
    %%if (par.depconstr)  
    %%   pertdiag = 1e-15*max(1,diagschur); 
    %%else
    %%   pertdiag = 1e-15*max(1e-4,diagschur); 
    %%end
    %%mexschurfun(schur,pertdiag); 
%%
%% assemble coefficient matrix
%% 
    len = size(Afree,2);
    if ~isempty(EE)
       EE(:,[1 2]) = len + EE(:,[1 2]); %% adjust for ublk
    end
    EE = [(1:len)' (1:len)' zeros(len,1); EE]; 
    if isempty(EE)
       coeff.mat22 = []; 
    else
       coeff.mat22 = spconvert(EE);
    end
    coeff.mat12 = [Afree, UU]; 
    coeff.mat11 = schur; %% important to use perturbed schur matrix
    ncolU = size(coeff.mat12,2); 
%%
%% pad rhs with zero vector
%% decide which solution methods to use
%%
    rhs = [rhs; zeros(m+ncolU-length(rhs),1)]; 
    if (ncolU > 300); use_LU = 1; end
%%
%% Cholesky factorization
%%
    L = []; resnrm = []; xx = inf*ones(m,1);
    if (~use_LU)
       nnzmat = mexnnz(coeff.mat11);
       nnzmatdiff = (nnzmat ~= nnzmatold);     
       solve_ok = 1;  solvesys = 1;    
       if (nnzmat > spdensity*m^2) | (m < 500) 
          matfct_options = 'chol';
       else
          matfct_options = 'spchol'; 
       end
       if (printlevel>2); fprintf(' %s',matfct_options); end 
       if strcmp(matfct_options,'chol')
          if issparse(schur); schur = full(schur); end;           
          L.matfct_options = 'chol';
          L.perm = [1:m]; 
          if (iter<=5); %% to fix strange anonmaly in Matlab
             mexschurfun(schur,1e-20,2); 
          end 
          [L.L,indef] = chol(schur); 
          if (indef)
 	     solve_ok = -2; solvesys = 0;
             fprintf('\n  chol: Schur complement matrix not pos. def.');
          end
       elseif strcmp(matfct_options,'spchol')
          if ~issparse(schur), schur = sparse(schur); end;
          if (nnzmatdiff | ~strcmp(matfct_options,matfct_options_old))
             [Lsymb,flag] = symbcholfun(schur,par.cachesize);
             if (flag) 
                solve_ok = -2;  solvesys = 0;
                existspcholsymb = 0;
                fprintf('\n  spchol: symbolic factorization fails.'); 
                use_LU = 1; 
             else 
                existspcholsymb = 1;
             end
          end 
          if (existspcholsymb)
             L = sparcholfun(Lsymb,schur);
             L.matfct_options  = 'spchol';  
             L.d(find(L.skip)) = 1e20;  
             if any(L.skip) & (ncolU)
                solve_ok = -3; solvesys = 0; 
                existspcholsymb = 0; 
                use_LU = 1; 
                if (printlevel)
                   fprintf('\n  spchol: L.skip exists but ncolU > 0.'); 
                   fprintf('\n  switch to symmetric indefinite or LU factor.');
                end
             end          
          end
       end    
       if (solvesys)
          if (ncolU)
             tmp = coeff.mat12'*linsysolvefun(L,coeff.mat12)-coeff.mat22; 
	     if issparse(tmp); tmp = full(tmp); end
             [L.Ml,L.Mu,L.Mp] = lu(tmp);
             pertdiag = zeros(ncolU,1); 
             tol = 1e-16; 
             idx = find(abs(diag(L.Mu)) < tol);
             if ~isempty(idx); fprintf('**'); end
          end
          [xx,resnrm,solve_ok] = symqmr(coeff,rhs,L);
          if (solve_ok<=0) & (printlevel)
             fprintf('\n  warning: symqmr fails: %3.1f.',solve_ok); 
          end
       end
       if (solve_ok < 0) 
          if (m < 5000 & strcmp(matfct_options,'chol')) | ...
             (m < 10000 & strcmp(matfct_options,'spchol'))
             use_LU = 1;
             %%switch2LU = 1; 
             if (printlevel)
                fprintf('\n  switch to symmetric indefinite or LU factor.');
             end
          end
       end
    end
%%
%% symmetric indefinite or LU factorization
%%
    existMA47symb = 0; %% to avoid using MA47
    if (use_LU)
       nnzmat = mexnnz(coeff.mat11)+mexnnz(coeff.mat12); 
       nnzmatdiff = (nnzmat ~= nnzmatold);  
       solve_ok = 1; solvesys = 1; 
       if ~isempty(coeff.mat22)
          raugmat = [coeff.mat11, coeff.mat12; coeff.mat12', coeff.mat22]; 
       else
          raugmat = coeff.mat11; 
       end
       if (nnzmat > spdensity*m^2) | (m+ncolU < 500) 
          matfct_options = 'lu';     
       elseif (existMA47symb) & (ncolU < m)
          matfct_options = 'MA47';
       else
          matfct_options = 'splu';
       end
       if (printlevel>2); fprintf(' %s ',matfct_options); end 
       if strcmp(matfct_options,'lu') 
          if issparse(raugmat); raugmat = full(raugmat); end
          [L.l,L.u,L.p] = lu(raugmat); 
          L.matfct_options = 'lu'; 
          L.p = sparse(L.p); 
          idx = find(abs(diag(L.u)) < 1e-20); 
          if ~isempty(idx)
             if (printlevel); fprintf('\n  lu: matrix is singular'); end
             solvesys = 0; 
          end
          [ii,jj] = find(L.p); [dummy,idx] = sort(ii); L.perm = jj(idx); 
       end
       if strcmp(matfct_options,'MA47')      
          if ~issparse(raugmat); raugmat = sparse(raugmat); end  
          if (nnzmatdiff | ~strcmp(matfct_options,matfct_options_old))
             [Lsymb,flag] = symbMA47(raugmat); 
             if (flag)
                existMA47symb = 0; 
                fprintf('\n  MA47: symbolic factor fails, switch to splu.');
                matfct_options = 'splu';
             else 
                existMA47symb = 1;  
             end 
          end
	  if (existMA47symb)
             [L,flag] = MA47fct(Lsymb,raugmat);
             L.matfct_options = 'MA47';  
             if (flag)
                existMA47symb = 0; 
                fprintf('\n  MA47: solver fails, switch to splu.');
                matfct_options = 'splu';
             end 
          end       
       end
       if strcmp(matfct_options,'splu') 
          if ~issparse(raugmat); raugmat = sparse(raugmat); end  
          if (nnzmatdiff | ~strcmp(matfct_options,matfct_options_old))
             Lsymb.perm = symamd(raugmat);
          end 
          L.perm = Lsymb.perm;  
          L.matfct_options = 'splu';  
          if (par.matlabversion >= 6.5)
             [L.l,L.u,L.p,L.q] = lu(raugmat(L.perm,L.perm));
          else
             [L.l,L.u,L.p] = lu(raugmat(L.perm,L.perm));
             L.q = speye(length(raugmat)); 
          end
       end
       if (solvesys)
          [xx,resnrm,solve_ok] = mybicgstab(coeff,rhs,L);
          if (solve_ok<=0) & (printlevel)
             fprintf('\n  warning: bicgstab fails: %3.1f,',solve_ok); 
          end
       end
    end
    if (printlevel>=3); fprintf(' %2.0d',length(resnrm)-1); end
%%
    nnzmatold = nnzmat; matfct_options_old = matfct_options; 
%%***************************************************************
