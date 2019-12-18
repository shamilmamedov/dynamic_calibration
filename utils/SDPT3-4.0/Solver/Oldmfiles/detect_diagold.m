%%*******************************************************************
%%  detect_diag: detect diagonal blocks in the SDP data. 
%%
%%  [blk,At,C,b,iidxdiag] = detect_diag(blk,At,C,b); 
%%
%% SDPT3: version 3.0 
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last modified: 2 Feb 01
%%******************************************************************

   function [blk,At,C,b,iidxdiag] = detect_diag(blk,At,C,b);

   m = length(b); numblk = size(blk,1);  
   idx_keepblk = [];  
%%
%% convert At{p}(:,k) to A{p,k}
%%
   if size(At,2) ~=m
      A = cell(size(blk,1),m); 
      for p = 1:size(blk,1); 
         pblk = blk(p,:); 
         if strcmp(pblk{1},'s');
            for k = 1:m
               A{p,k} = mexsmat(blk,At,1,p,k); 
            end
         elseif strcmp(pblk{1},'l');            
            for k = 1:m
               A{p,k} = At{p}(:,k);
            end
         end
      end
      conversion = 1; 
   else
      A = At; 
      conversion = 0; 
   end
%%    
%% dectect diagonal sub-block in nondiagonal blocks. 
%%
   blkchanged = 0; 
   for p = 1:size(blk,1);  
       blktmp = blk{p,2}; 
       n = sum(blktmp);
       if strcmp(blk{p,1},'s');
          Acum = abs(C{p});  
          for k = 1:m 
              Acum = Acum + abs(A{p,k});
          end;
          Acum = Acum - spdiags(diag(Acum),0,n,n); 
          rownorm = sum(Acum.*Acum);
          idxdiag = find(rownorm < 1e-13);
          idxnondiag  = setdiff([1:n],idxdiag); 
          iidxdiag{p} = idxdiag; 
          if ~isempty(idxdiag); 
             if (blkchanged == 0) 
                blkchanged = 1;
                diagblk = numblk+1; 
                A(diagblk,:) = cell(1,size(A,2)); 
                C(diagblk,1) = cell(1,1); 
                blk{diagblk,1} = 'l'; 
                blk{diagblk,2} = 0; 
                idx_keepblk = [idx_keepblk, diagblk]; 
             end;
             blk{diagblk,2} = blk{diagblk,2} + length(idxdiag); 
             blk{p,2} = length(idxnondiag); 
             for k = 1:size(A,2);
                tmp = diag(A{p,k}); 
                A{diagblk,k} = [A{diagblk,k} ; tmp(idxdiag)]; 
                A{p,k} = A{p,k}(idxnondiag,idxnondiag);
             end; 
             tmp = diag(C{p,1}); 
             C{diagblk,1} = [C{diagblk,1}; tmp(idxdiag)]; 
             C{p,1} = C{p,1}(idxnondiag,idxnondiag);
          end;   
          if ~isempty(idxnondiag); 
             idx_keepblk = [idx_keepblk, p]; 
          end; 
       elseif strcmp(blk{p,1},'l'); 
          idx_keepblk = [idx_keepblk, p];
       end;
   end;          
   if (blkchanged); 
      blk = blk(idx_keepblk,:); 
      A = A(idx_keepblk,:);
      C = C(idx_keepblk,:); 
   end;
%%
%% concatenate diagonal blocks.
%%
   diagblk = [];  idx_keepblk = []; 
   for p = 1:size(blk,1)
      if strcmp(blk{p,1},'l'); 
         diagblk = [diagblk, p];  
      else;
         idx_keepblk = [idx_keepblk, p]; 
      end; 
   end;
   if ~isempty(diagblk); 
      idx1 = diagblk(1); 
      idx_keepblk = [idx_keepblk, idx1];
   end;
   if (length(diagblk) >= 2); 
      for p = 2:length(diagblk); 
         idxp = diagblk(p); 
         for k = 1:size(A,2);
            A{idx1,k} = [A{idx1,k}; A{idxp,k}]; 
         end; 
         C{idx1,1} = [C{idx1,1}; C{idxp,1}]; 
         blk{idx1,2} = blk{idx1,2} + blk{idxp,2};             
      end;
      blk = blk(idx_keepblk,:); 
      A = A(idx_keepblk,:);
      C = C(idx_keepblk,:); 
   end; 
%%
%%
%% 
   if conversion; 
      At = svec(blk,A,ones(size(blk,1),1)); 
   else 
      At = A; 
   end
%%
%%******************************************************************



