%%***************************************************************************
%% lanczos: find the largest eigenvalue of 
%%          invXchol'*dX*invXchol via the lanczos iteration.
%%
%% [lam,delta] = lanczos(Xchol,dX,maxit,tol,v)
%%
%% lam:  an estimate of the largest eigenvalue.
%% lam2: an estimate of the second largest eigenvalue.
%% res:  residual norm of the largest eigen-pair.
%% res2: residual norm of the second largest eigen-pair.
%%***************************************************************************

   function [lam,delta,res] = lanczos(Xchol,dX,maxit,tol,v) 
                                            
   if (norm(dX,'fro') < 1e-13) 
      lam = 0; delta = 0; res = 0; 
      return;
   end
   n = length(dX); 
   if (nargin < 5); 
      state = randn('state'); 
      randn('state',0); 
      v = randn(n,1); 
      randn('state',state); 
   end
   if (nargin < 4); maxit = 30; end
   if (nargin < 3); tol = 1e-3; end
   V = zeros(n,maxit+1); H = zeros(maxit+1,maxit); 
   v = v/norm(v); 
   V(:,1) = v; 
   if issparse(Xchol); Xcholtransp = Xchol'; end
%%
%% lanczos iteration. 
%%
   for k = 1:maxit
      if issparse(Xchol)
         w =  dX*mextriangsp(Xcholtransp,v,1); 
         w =  mextriangsp(Xchol,w,2); 
      else         
         w =  dX*mextriang(Xchol,v,1); 
         w =  mextriang(Xchol,w,2); 
      end      
      wold = w;
      if (k > 1); 
         w = w - H(k,k-1)*V(:,k-1); 
      end;
      alp = w'*V(:,k); 
      w   = w - alp*V(:,k); 
      H(k,k) = alp; 
      %%
      %% one step of iterative refinement if necessary. 
      %%
      if (norm(w) <= 0.8*norm(wold));
         s = (w'*V(:,1:k))'; 
         w = w - V(:,1:k)*s;
         H(1:k,k) = H(1:k,k) + s;
      end; 
      nrm = norm(w); 
      v = w/nrm; 
      V(:,k+1) = v; 
      H(k+1,k) = nrm;  H(k,k+1) = nrm; 
      %%
      %% compute ritz pairs and test for convergence
      %%
      if (rem(k,5) == 0) | (k == maxit); 
         Hk = H(1:k,1:k); Hk = 0.5*(Hk+Hk'); 
         [Y,D] = eig(Hk); 
         eigH  = real(diag(D)); 
         [dummy,idx] = sort(eigH);
         res_est = abs(H(k+1,k)*Y(k,idx(k)));
         if (res_est <= 0.1*tol) | (k == maxit);
            lam = eigH(idx(k));  
            lam2 = eigH(idx(k-1)); 
            z   = V(:,1:k)*Y(:,idx(k));
            z2   = V(:,1:k)*Y(:,idx(k-1));
            if issparse(Xchol) 
               tmp = dX*mextriangsp(Xcholtransp,z,1); 
               res = norm(mextriangsp(Xchol,tmp,2) -lam*z); 
               tmp = dX*mextriangsp(Xcholtransp,z2,1); 
               res2 = norm(mextriangsp(Xchol,tmp,2) -lam*z2);   
            else
               tmp = dX*mextriang(Xchol,z,1); 
               res = norm(mextriang(Xchol,tmp,2) -lam*z); 
               tmp = dX*mextriang(Xchol,z2,1); 
               res2 = norm(mextriang(Xchol,tmp,2) -lam*z2);   
            end
            tmp = lam-lam2 -res2; 
            if (tmp > 0); beta = tmp; else; beta = eps; end;  
            delta = min(res,res^2/beta); 
            if (delta <= tol); break; end;
         end 
      end 
   end
%%***************************************************************************
