%%*************************************************************************
%% mybicgstab: bicgstab
%%  
%% [xx,resnrm,solve_ok] = mybicgstab(A,b,M1,tol,maxit)
%%
%% iterate on  bb - (M1)*AA*x
%%
%% r = b-A*xtrue; 
%%
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*************************************************************************

  function [xx,resnrm,flag] = mybicgstab(A,b,M1,tol,maxit)

  N = length(b); 
  if (nargin < 5); maxit = min(50,max(5,length(A.mat22))); end;
  if (nargin < 4); tol = 1e-8; end; 
  tolb = min(1e-4,tol*norm(b));
  flag = 1;
 
  x = zeros(N,1); 
  if isstruct(A); r = b-matvec(A,x); else; r = b-mexMatvec(A,x); end;     
  err = norm(r); resnrm(1) = err;  minresnrm = err; xx = x;  
  %% if (err < tolb); return; end

  omega  = 1.0;
  r_tld = r;
%%
%%
%%
  smtol = 1e-40; 
  for iter = 1:maxit,           

     rho   = (r_tld'*r);                        
     if (abs(rho) < smtol)
        flag = 2;  fprintf('*'); break; 
     end
     if (iter > 1)
        beta  = (rho/rho_1)* (alp/omega);
        p = r + beta*(p - omega*v);
     else
        p = r;
     end
     p_hat = linsysolvefun(M1,p); 
     if isstruct(A); v = matvec(A,p_hat); else; v = mexMatvec(A,p_hat); end;     
     alp = rho / (r_tld'*v);
     s = r - alp*v;

     s_hat = linsysolvefun(M1,s);
     if isstruct(A); t = matvec(A,s_hat); else; t = mexMatvec(A,s_hat); end;     
     omega = (t'*s) / (t'*t);
     x = x + alp*p_hat + omega*s_hat;              
     r = s - omega*t;
     rho_1 = rho;
     %%
     %% check convergence
     %%
     err = norm(r); resnrm(iter+1) = err;     
     minresnrm = min(minresnrm,err); 
     if (err < tolb)
        break;  
     end
     if (err > 1e5*minresnrm) & (iter > 50); 
        flag = -0.5; break; 
     end       
     if (abs(omega) < smtol)
        flag = 2; fprintf('*'); break; 
     end
  end
  xx = x; 
%%*************************************************************************
%%*************************************************************************
%% matvec: matrix-vector multiply.
%% matrix = [A.mat11 A.mat12; A.mat12' A.mat22]
%%*************************************************************************

   function Ax = matvec(A,x);

   m = length(A.mat11); m2 = length(x)-m; 
   if (m2 > 0)
      x1 = full(x(1:m)); 
   else
      x1 = full(x); 
   end
   Ax = mexMatvec(A.mat11,x1);
   if (m2 > 0)
      x2 = full(x(m+[1:m2]));
      Ax = Ax + mexMatvec(A.mat12,x2); 
      Ax2 = mexMatvec(A.mat12,x1,1) + mexMatvec(A.mat22,x2);
      Ax = [Ax; Ax2];  
   end
   return;
%%*************************************************************************
