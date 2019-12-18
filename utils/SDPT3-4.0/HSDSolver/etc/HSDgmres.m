%%****************************************************************
%%  GMRES
%%
%%****************************************************************

  function [x,resnrm,flag] = HSDgmres(A,b,M,tol,maxit,printlevel)

  N = length(b); 
  if (nargin < 6); printlevel = 1; end
  if (nargin < 5) | isempty(maxit); maxit = min(20,max(5,length(A.mat22))); end;
  if (nargin < 4) | isempty(tol); tol = 1e-8; end; 
  tolb = min(1e-4,tol*norm(b));
  flag = 1;

  m = maxit; 
  V   = zeros(N,m+1);
  H   = zeros(m+1,m);
  V(:,1) = b/norm(b);
  resnrm(1) = norm(b); 
 %% 
  for k = 1:m    
      tmp = precond(A,M,V(:,k)); 
      if isstruct(A); v = matvec(A,tmp); else; v = mexMatvec(A,tmp); end;     
      for j = 1:k                  
          H(j,k) = (V(:,j))'*v;     
          v  =  v - H(j,k)*V(:,j);  
      end;
      H(k+1,k) = norm(v);
      V(:,k+1) = v/H(k+1,k);

      d = norm(b)*eye(k+1,1);
      y = H(1:k+1,1:k)\d;
      err = norm(H(1:k+1,1:k)*y - d);  
      resnrm(k+1) = err; 
      if (err < tolb); break; end; 
   end;
   x = precond(A,M,V(:,1:k)*y);   
%%
%%*************************************************************************
%% matvec: matrix-vector multiply.
%% matrix = [A.mat11, A.mat12; A.mat12', A.mat22]
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
%%*************************************************************************
%% precond: 
%%*************************************************************************

   function Mx = precond(A,L,x)

   m = length(L.perm); m2 = length(x)-m;
   if (m2 > 0)
      x1 = full(x(1:m)); 
   else
      x1 = full(x); 
   end
   if (m2 > 0)
      x2 = x(m+[1:m2]);
      w = linsysolvefun(L,x1); 
      z = mexMatvec(A.mat12,w,1) -x2;
      z = L.Mu \ (L.Ml \ (L.Mp*z));
      x1 = x1 - mexMatvec(A.mat12,z); 
   end
%% 
   Mx = linsysolvefun(L,x1);  
%%
   if (m2 > 0)
      Mx = [Mx; z];
   end
%%*************************************************************************



