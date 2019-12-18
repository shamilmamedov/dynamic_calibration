%%*********************************************************************
%% schursysolve: solve 
%%             [H   U][x1] = [b1]
%%             [V' -E][x2]   [b2]
%%  via the Schur complement method as follows: 
%%  SMWmat*x2 = V'*inv(H)*b1 - b2, where SMWmat = V'*inv(H)*U + E, 
%%         x1 = inv(H)*(b1 - U*x2).  
%%       
%%  Here H is symmetric positive definite
%%  and U, V are low-rank, possibly []. 
%%
%%  [x,solve_ok,SMWmat] = schursysolve(H,L,U,V,E,b,matfct_options,SMWmat)
%%
%%  if matfct_options == 'chol'
%%     L is the Cholesky factor of H computed by Matlab's rountine
%%     Note H = L'*L
%%     On entry, H has already been overwritten by L. 
%%  elseif matfct_options == 'spchol'
%%     L contains the sparse LDL' factorization of H computed
%%     via the sparse Cholesky routines in Spchol
%%     Note H = L*D*L' 
%%  end
%%
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%*********************************************************************

  function [x,solve_ok,SMWmat] = schursysolve(H,L,U,V,E,b,matfct_options,SMWmat)

  if (nargin <= 7); SMWmat = []; end;
  ismtU = isempty(U);
  m = length(H); n = size(U,2); 
  if (length(b) < m+n)
     b = [b; zeros(m+n-length(b),1)]; 
  end
%%
  if ismtU      
     solve_ok = 1; 
     x = linslv(L,b,matfct_options);  
  else
     solve_ok = -1;     
     if isempty(SMWmat) 
        n = size(U,2); 
        HinvU = zeros(size(U));
        for i = 1:n
            HinvU(:,i) = linslv(L,U(:,i),matfct_options);
        end   
	[SMWmat.l,SMWmat.u,SMWmat.p] = lu(E+V'*HinvU); 
     end    
     b1 = b(1:m); b2 = b(m+[1:n]); 
     btmp = V'*linslv(L,b1,matfct_options) - b2; 
     
     x2 = SMWmat.u \ (SMWmat.l \ (SMWmat.p*btmp));
     x1 = linslv(L,b1-U*x2,matfct_options);
     x = [x1; x2]; 
     %%
     %% Iterative refinement. This step is crucial to ensure 
     %% that computed solution is sufficiently accuraete. 
     %%
     rtol = 1e-4; 
     if length(b) > 5000; 
        maxit = 50; 
     else
        maxit = 25; 
     end
     bnorm = max(1,norm(b)); 
     for k = 1:maxit
        x0 = x; 
        if strcmp(matfct_options,'chol');
           tmp = L*x1; Hx1 = (tmp'*L)'; 
        elseif strcmp(matfct_options,'spchol');
           Hx1 = H*x1;
        end
        r1 = b1 - (Hx1 + U*x2);  
        r2 = b2 - (V'*x1 - E*x2); 
        %%
        rrnrm(k) = sqrt(norm(r1)^2 + norm(r2)^2)/bnorm;      
        if (rrnrm(k) < rtol); solve_ok = 1; break; end; 
        if (k > 1) & (rrnrm(k)/rrnrm(k-1) > 0.8);
            x = x0; break; 
        end
        rtmp = V'*linslv(L,r1,matfct_options) -r2;
        d2 = SMWmat.u \ (SMWmat.l \ (SMWmat.p*rtmp));
        d1 = linslv(L,r1-U*d2,matfct_options);
        x1 = x1 + d1;
        x2 = x2 + d2; 
        x = [x1; x2]; 
     end
  end
%%
%%*************************************************************************
%% linslv: Solve a linear system H*x = b
%%         where H is symmetric positive definite.
%%
%% L is the Cholesky factor of H. 
%%*************************************************************************
 
  function x = linslv(L,b,matfct_options)

  if strcmp(matfct_options,'chol')
     if issparse(b); b = full(b); end;
     tmp = mexbsolve(L,b,2); 
     x   = mexbsolve(L,tmp,1); 
  elseif strcmp(matfct_options,'spchol')
     x = bwblkslvfun(L, fwblkslvfun(L,b) ./ L.d);
  end
%%
%%*************************************************************************
