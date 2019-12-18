%%*****************************************************************************
%% misc: 
%% unscale and produce infeasibility certificates if appropriate
%%
%%*****************************************************************************

   function [X,y,Z,Xiter,yiter,Ziter,resid,reldist,param] = misc(blk,At,C,b,X,y,Z,permZ,param);

   global printlevel

   obj         = param.obj;
   rel_gap     = param.rel_gap; 
   prim_infeas = param.prim_infeas;
   dual_infeas = param.dual_infeas;
   ZpATynorm   = param.ZpATynorm; 
   inftol      = param.inftol;
   normA       = param.normA;        
   normC       = param.normC;
   normb       = param.normb;
   keepiter    = param.keepiter; 
   m0          = param.m0;    
   indeprows   = param.indeprows;
   termcode    = param.termcode;
   AX          = param.AX;
   scale_data  = param.scale_data;
   normX0      = param.normX0;
   normZ0      = param.normZ0;
%%
   if (~keepiter); Xiter = []; yiter = []; Ziter = []; end
   resid = []; reldist = [];
%%
   Anorm = ops(At,'norm'); xnorm = ops(X,'norm'); ynorm = norm(y);
   infeas_meas = max(prim_infeas,dual_infeas); 
   if (termcode <= 0)
      %%
      %% To detect near-infeasibility when the algorithm provides 
      %% a "better" certificate of infeasibility than of optimality.
      %%
      err = max(infeas_meas,rel_gap);
      iflag = 0;
      if (obj(2) > 0)
         homRd = ZpATynorm/(obj(2)/(normb*normC));
         if (homRd < sqrt(err*inftol))
            iflag = 1;
            if (printlevel); 
               fprintf('\n prim_inf,dual_inf,rel_gap = %3.2e, %3.2e, %3.2e',...
               prim_infeas,dual_infeas,rel_gap); 
            end
            termcode = 1;
            param.termcode = 1;
         end
      elseif (obj(1) < 0)
         homrp = norm(AX)/(-obj(1)/(normb*normC)); 
         if (homrp < sqrt(err*inftol)) 
            if (printlevel); 
               fprintf('\n prim_inf,dual_inf,rel_gap = %3.2e, %3.2e, %3.2e',...
               prim_infeas,dual_infeas,rel_gap); 
            end
            iflag = 1; 
            termcode = 2;
            param.termcode = 2;
         end
      end
      if (iflag == 0)
         if (scale_data == 1)
            X = ops(normb,'*',X); y = normC*y./normA; Z = ops(normC,'*',Z);
         end
         if (keepiter); Xiter = X; yiter = y; Ziter = Z; end; 
      end
   end
   if (termcode == 1)
      if (printlevel); 
         fprintf('\n Stop: primal problem is suspected of being infeasible'); 
      end
      if (scale_data == 1)
         b = normb*b; b = b.*normA; y = y./normA;
         X = ops(normb,'*',X);
         if (keepiter)
            yiter = normC*y; Ziter = ops(normC,'*',Z); Xiter = X; 
         end 
      else
         if (keepiter); Xiter = X; yiter = y; Ziter = Z; end;
      end
      rby = 1/(b'*y); y = rby*y; Z = ops(Z,'*',rby);
      resid = ZpATynorm * rby;
      reldist = ZpATynorm/(Anorm*ynorm);
   end  
   if (termcode == 2)
      if (printlevel); 
         fprintf('\n Stop: dual problem is suspected of being infeasible'); 
      end
      if (scale_data == 1)
         C = ops(C,'*',normC);
         y = normC*y./normA; Z = ops(normC,'*',Z);
         if (keepiter)
            Xiter = ops(normb,'*',X); yiter = y; Ziter = Z;
         end 
      else
         if (keepiter);  Xiter = X; yiter = y; Ziter = Z; end; 
      end
      tCX = blktrace(blk,C,X);
      X = ops(X,'*',1/(-tCX));
      resid = norm(normA.*AX)/(-tCX);
      reldist = norm(AX)/(Anorm*xnorm);
   end
   if (termcode == 3)
      maxblowup = max(ops(X,'norm')/normX0,ops(Z,'norm')/normZ0);
      if (printlevel)
         fprintf('\n Stop: primal or dual is diverging, %3.1e',maxblowup); 
      end
   end
   if (keepiter)
      [X,Z,Xiter,Ziter] = unperm(blk,permZ,X,Z,Xiter,Ziter);
   else
      [X,Z] = unperm(blk,permZ,X,Z);
   end
   if ~isempty(indeprows)
      ytmp = zeros(m0,1); 
      ytmp(indeprows) = y;
      y = ytmp; 
      if (keepiter)
         ytmp = zeros(m0,1);  
         ytmp(indeprows) = yiter; 
         yiter = ytmp; 
      end
   end
%%*****************************************************************************
%% unperm: undo the permutations applied in validate.
%%
%% [X,Z,Xiter,Ziter] = unperm(blk,permZ,X,Z,Xiter,Ziter);
%%
%% undoes the permutation introduced in validate.
%% can also be called if Xiter and Ziter have not been set as
%%
%% [X,Z] = unperm(blk,permZ,X,Z);
%%
%% SDPT3: version 3.0
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%*****************************************************************************
 
  function [X,Z,Xiter,Ziter] = unperm(blk,permZ,X,Z,Xiter,Ziter);
%%
  for p = 1:size(blk,1)
     if (strcmp(blk{p,1},'s') & ~isempty(permZ{p}))
        per = permZ{p};
        X{p} = X{p}(per,per);
        Z{p} = Z{p}(per,per);
        if (nargin >= 5)
           Xiter{p} = Xiter{p}(per,per);
           Ziter{p} = Ziter{p}(per,per);
        end
     end
  end
%%*****************************************************************************
