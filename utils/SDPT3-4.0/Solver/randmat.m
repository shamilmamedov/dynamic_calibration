%%*****************************************************************
%% randmat: generate an mxn matrix using matlab's
%%          rand or randn functions using state = k.
%%*****************************************************************
%% SDPT3: version 4.0
%% Copyright (c) 1997 by
%% Kim-Chuan Toh, Michael J. Todd, Reha H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*****************************************************************

function v = randmat(m,n,k,randtype)

try 
   s = rng; 
   rng(k);
   if strcmp(randtype,'n')
      v = randn(m,n); 
   elseif strcmp(randtype,'u')
      v = rand(m,n); 
   end
   rng(s); 
catch
   if strcmp(randtype,'n')
      s = randn('state');
      randn('state',k);
      v = randn(m,n); 
      randn('state',s);
   elseif strcmp(randtype,'u')
      s = rand('state');
      rand('state',k);
      v = rand(m,n); 
      rand('state',s);
   end
end
%%******************************************************