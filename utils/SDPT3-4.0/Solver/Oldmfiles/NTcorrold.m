%%************************************************************************
%% NTcorr: corrector step for the NT direction. 
%%
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%************************************************************************

  function [dX,dy,dZ] = NTcorr(blk,At,par,rp,Rd,sigmu,hRd,...
            dX,dZ,schur,L,UU,VV,E,SMWmat,X,Z); 

    global matlabversion
    global matfct_options solve_ok; 
%%
    [rhs,EinvRc]  = NTrhsfun(blk,At,par,X,Z,rp,Rd,sigmu,hRd,dX,dZ);
    rhs = [rhs; zeros(length(rp)+size(UU,2)-length(rhs),1)]; 
    if strcmp(matfct_options,'chol') 
       [xx,solve_ok] = schursysolve(schur,schur,UU,VV,E,rhs,matfct_options,SMWmat);
    elseif strcmp(matfct_options,'spchol')
       [xx,solve_ok] = schursysolve(schur,L,UU,VV,E,rhs,matfct_options,SMWmat);
    elseif strcmp(matfct_options,'MA47')
       xx = mexMA47slv(length(rhs),L.a,L.iw,rhs);
    elseif strcmp(matfct_options,'splu')
       xx(L.perm,1) = L.q*( L.u \ (L.l \ (L.p*rhs(L.perm))));
    elseif strcmp(matfct_options,'lu')      
       xx = L.u \ (L.l \ (L.p*rhs));
    end   
    [dX,dy,dZ] = NTdirfun(blk,At,par,Rd,EinvRc,xx); 
%%************************************************************************


