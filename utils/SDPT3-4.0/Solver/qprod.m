%%*****************************************************************
%% qprod: 
%%       
%% Input: A = [A1 A2 ... An]
%%        x = [x1; x2; ...; xn]
%% Output: [A1*x1 A2*x2 ... An*xn]
%%*****************************************************************
%% SDPT3: version 4.0
%% Copyright (c) 1997 by
%% Kim-Chuan Toh, Michael J. Todd, Reha H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*****************************************************************

  function Ax = qprod(pblk,A,x); 

  if (size(pblk,1) > 1)
     error('qprod: pblk can only have 1 row'); 
  end
  if issparse(x); x = full(x); end; %% for spconvert
  n = length(x); 
  ii = [1:n]'; 
  jj = mexexpand(pblk{2},[1:length(pblk{2})]'); 
  X = spconvert([ii, jj, x]);
  Ax = A*X;
%%***************************************************
