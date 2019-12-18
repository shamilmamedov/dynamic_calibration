%%********************************************************************
%% scaling: scale the SDP data so that A_k,C,b have unit norm. 
%%
%%  [At,C,b,normA,normC,normb,X0,y0,Z0] = scaling(blk,At,C,b,X0,y0,Z0); 
%%
%%  Because of scaling, the objective function is modified: 
%%
%%  old obj function = (normb*normC) (new obj function).
%%
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%********************************************************************

   function [At,C,b,normA,normC,normb,X0,y0,Z0] = scaling(blk,At,C,b,X0,y0,Z0); 

   m = length(b); 
   numblk = size(blk,1);  

   normA = zeros(m,1);  
   for p=1:numblk
      pblk = blk(p,:);
      if strcmp(pblk{1},'s')
         m1 = size(At{p,1},2); 
         normA(1:m1) = normA(1:m1) + sqrt(sum(At{p,1}.*At{p,1}))'; 
         if (length(pblk) > 2) %% for low rank constraints
            dd = At{p,3}; 
            m2 = m-m1; 
            ss = [0,cumsum(pblk{3})]; 
            for k=1:m2
               idx = [ss(k)+1:ss(k+1)]; 
               V = At{p,2}(:,idx); 
               ii = dd(idx,1)-ss(k); %% undo cumulative indexing
               jj = dd(idx,2)-ss(k); 
               len = pblk{3}(k); 
               D = spconvert([ii,jj,dd(idx,3); len,len,0]);
               normA(m1+k) = normA(m1+k) + norm(V'*V*D,'fro'); 
            end
         end
      else
         normA = normA + sqrt(sum(At{p,1}.*At{p,1}))'; 
      end
   end
   normA = max(1,normA);
%% 
   for p=1:numblk
      pblk = blk(p,:);
      if strcmp(pblk{1},'s')
         m1 = size(At{p,1},2);
         m2 = m - m1;
         At{p,1} = At{p,1}*spdiags(1./normA(1:m1),0,m1,m1);
         if (m2 > 0) %% for low rank constaints
            yy2 = mexexpand(pblk{3},normA(m1+[1:m2])); 
            At{p,3}(:,3) = At{p,3}(:,3)./yy2;
         end
      else
         At{p,1} = At{p,1}*spdiags(1./normA,0,m,m);
      end
   end
   b = b./normA; normb = max(1,norm(b)); 
   b = b/normb; 
 
   normC = 0; 
   for p=1:numblk
       normC = normC + sum(sum(C{p}.*C{p})); 
   end 
   normC = sqrt(normC); 
   normC = max(1,normC);
   y0 = y0.*normA/normC; 
   for p=1:numblk
      C{p}  = C{p}/normC; 
      X0{p} = X0{p}/normb; 
      Z0{p} = Z0{p}/normC; 
   end
%%********************************************************************
