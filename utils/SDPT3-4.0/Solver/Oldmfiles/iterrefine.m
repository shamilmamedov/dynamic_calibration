%%*************************************************************************
%% iterrefine: Iterative refinement. 
%% This step is crucial to ensure that computed solution 
%% is sufficiently accuraete. 
%%
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*************************************************************************

   function [x,resnrm,solve_ok] = iterrefine(A,b,L,x0); 

     tol = 1e-6; 
     maxit = 10; 
     bnorm = max(1,norm(b)); 
     tolb = tol*bnorm;
     x = x0; 
     solve_ok = 1; 
     resnrm(1) = bnorm;
%%
     for iter = 1:maxit
        x0 = x; 
        if isstruct(A); r = b-matvec(A,x); else; r=b-A*x; end;         
        err = norm(r);       
        resnrm(iter+1) = err; 
        
        if (err < tolb); break; end; 
        if (iter > 1) & (resnrm(iter+1)/resnrm(iter) > 0.9)
           x = x0; solve_ok = 0; break; 
        end
	d = linsysolvefun(L,r); 
        x = x + d;             
     end
%%*************************************************************************
%% matvec: matrix-vector multiply.
%% matrix = [A.mat11 A.mat12; A.mat12' A.mat22]
%%*************************************************************************

   function Ax = matvec(A,x);

   m = length(A.mat11); m2 = length(x)-m; 
   x1 = x(1:m); 
   Ax = A.mat11*x1;
   if (m2 > 0)
      x2 = x(m+[1:m2]);
      Ax = Ax + A.mat12*x2; 
      Ax2 = (x1'*A.mat12)' + A.mat22*x2;
      Ax = [Ax; Ax2];  
   end
   return;
%%*************************************************************************
