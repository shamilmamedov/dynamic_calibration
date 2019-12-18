%%****************************************************
%% mcpschur: compute schur matrix of HKM or NT direction
%%           for MCP and maxG problems.  
%%
%% [schurmat] = mcpschur(X,Zinv,schurfun_par); 
%%
%%   Ak= -ek*ek';
%%*****************************************************************
%% SDPT3: version 4.0
%% Copyright (c) 1997 by
%% Kim-Chuan Toh, Michael J. Todd, Reha H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*****************************************************************

  function [schurmat] = mcpschur(X,Zinv,schurfun_pars); 
  
  schurmat = X .* Zinv;
%%****************************************************
