%%*********************************************************************
%% gdcomp: Compute gd = 1/td in Equation (15) of FOT's paper.
%%
%% [gd,info,blk2,At2,C2,b2] = gdcomp(blk,At,C,b,OPTIONS);
%%
%%*********************************************************************

  function [gd,info,blk2,At2,C2,b2] = gdcomp(blk,At,C,b,OPTIONS);

  if (nargin == 4)
     OPTIONS = sqlparameters; 
     OPTIONS.vers = 1; 
     OPTIONS.printlevel = 3; 
  end
  if ~iscell(C); tmp = C; clear C; C{1} = tmp; end
%%
  m = length(b); 
  blk2 = blk;
  At2 = cell(size(blk,1),1); 
  C2 = cell(size(blk,1),1); 
  b2 = [zeros(m,1); 1; 0]; 
%%
%% 
%%
  for p = 1:size(blk,1)
     pblk = blk(p,:); 
     n = sum(pblk{2}); 
     if strcmp(pblk{1},'s')
        C2{p,1} = sparse(n,n); 
     else
        C2{p,1} = zeros(n,1);
     end
  end
%%
%% New multipliers in dual problem: tt, theta.
%% [v; tt; theta].
%%
   ss = 0; cc = 0; aa = zeros(1,m); 
   exist_ublk = 0; 
   for p = 1:size(blk,1)
      pblk = blk(p,:); 
      n = sum(pblk{2}); 
      if strcmp(pblk{1},'s')
         At2{p} = [At{p}, svec(pblk,speye(n,n),1), -svec(pblk,C{p},1)]; 
         ss = ss + n; 
         cc = cc + trace(C{p}); 
         aa = aa + svec(pblk,speye(n),1)'*At{p}; 
      elseif strcmp(pblk{1},'q')
         eq = zeros(n,1); 
         idx1 = 1+[0,cumsum(pblk{2})]; 
         idx1 = idx1(1:length(idx1)-1);          
         eq(idx1) = ones(length(idx1),1);
         At2{p} = [At{p}, 2*sparse(eq), -sparse(C{p})];          
         ss = ss + 2*length(pblk{2}); 
         cc = cc + sum(C{p}(idx1)); 
         aa = aa + eq'*At{p}; 
      elseif strcmp(pblk{1},'l')
         el = ones(n,1); 
         At2{p} = [At{p}, sparse(el), -sparse(C{p})]; 
         ss = ss + n;
         cc = cc + el'*C{p}; 
         aa = aa + el'*At{p}; 
      elseif strcmp(pblk{1},'u')
         At2{p} = [At{p}, sparse(n,1), -sparse(C{p})]; 
         exist_ublk = 1; 
      end
   end
%%
%% 3 additional inequality constraints in dual problem.
%%
   alp = max(1,sqrt(sum(abs(aa)))); 
   numblk = size(blk,1); 
   blk2{numblk+1,1} = 'l'; blk2{numblk+1,2} = 3; 
   C2{numblk+1,1}  = [1; alp; 0]; 
   At2{numblk+1,1} = [-aa,        0,   cc; 
		     zeros(1,m),  0,   alp;
		     zeros(1,m), alp, -alp];
%%
%% Solve SDP
%%
   OPTIONS.gaptol = 1e-10;
   [obj,X,y,Z,info] = HSDsqlp(blk2,At2,C2,b2,OPTIONS); 
   gd = 1/abs(obj(2));
   err = max([info.gap/(1+mean(abs(obj))), info.pinfeas, info.dinfeas]);
   if (OPTIONS.printlevel)
      fprintf('\n ******** gd = %3.1e, err = %3.1e\n',gd,err); 
      if (err > 1e-6);
         fprintf('\n----------------------------------------------------')
         fprintf('\n gd problem is not solved to sufficient accuracy');
         fprintf('\n----------------------------------------------------\n')
      end
   end
%%*********************************************************************
