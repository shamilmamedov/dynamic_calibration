%%***********************************************************************
%% validate: validate data
%%
%% [blk,At,C,dim,numblk,X0,Z0] = validate(blk,At,C,b,X0,y0,Z0);
%%
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%***********************************************************************

   function  [blk,At,C,dim,numblk,X0,Z0] = validate(blk,At,C,b,X0,y0,Z0);

   global spdensity smallblkdim

   if isempty(spdensity); spdensity = 0.5; end; 
%%
   if ~iscell(blk); 
      error('validate: blk must be a cell array'); end; 
   if (size(blk,2) < 2)
      error('validate: blk must be a cell array with at least 2 columns');       
   end 
   if ~iscell(At) | ~iscell(C); 
      error('validate: At, C must be cell arrays'); end;
   if (size(At,1) ~= size(blk,1)) | (size(C,1) ~= size(blk,1))
      error('validate: size of At or C are not compatible with blk'); end;
   if (nargin == 7)
      if ~iscell(X0) | ~iscell(Z0); 
         error('validate: X0, Z0 must be cell arrays'); 
      end
      if (min(size(X0))~=1 | min(size(Z0))~=1); 
         error('validate: cell array X, Z can only have 1 column or row'); 
      end
      if (size(X0,2) > size(X0,1)); X0 = X0'; end;
      if (size(Z0,2) > size(Z0,1)); Z0 = Z0'; end;
   end
%%
%% 
   m = length(b);  numblk = size(blk,1);
   for p=1:size(blk,1)
      pblk = blk(p,:); 
      n = sum(pblk{2}); 
      numblk = length(pblk{2}); 
      if strcmp(pblk{1},'s');
         m1 = size(At{p,1},2); 
         n2 = sum(pblk{2}.*pblk{2});  n22 = sum(pblk{2}.*(pblk{2}+1))/2; 
         if ~all(size(C{p}) == n) 
            error('validate: blk and C are not compatible'); end; 
         if (norm(C{p}-C{p}',inf) > 1e-13); 
            error('validate: C is not symmetric'); end;
         if (size(At{p,1}) == [m1, n22] & m1~=n22); 
            At{p,1} = At{p,1}'; end
         if (~isempty(At{p,1})) & (size(At{p,1},1) ~= n22)
            error('validate: blk and At not compatible'); end; 
         if (nnz(At{p,1}) < spdensity*n22*m1)
            if ~issparse(At{p,1}); At{p,1} = sparse(At{p,1}); end 
         end
         if (length(pblk) > 2)
            smallblkdim = 0;
            if (length(At(p,:)) > 2) 
               if (min(size(At{p,3})) > 1)
                  error(' low rank structure in At{p,3} not specified correctly')
               else
                  if (size(At{p,3},1) < size(At{p,3},2)); At{p,3} = At{p,3}'; end  
               end
               if (sum(pblk{3}) ~= length(At{p,3})) 
                  error(' low rank structure specified in blk and At not compatible') 
               end
            end
            if (sum(pblk{3}) ~= size(At{p,2},2))
               error(' low rank structure specified in blk and At not compatible') 
            end
            if (size(pblk{1,3},2) < size(pblk{1,3},1))
               blk{p,3} = blk{p,3}'; 
            end
         end
         if (nnz(C{p}) < spdensity*n2) | (numblk > 1); 
            if ~issparse(C{p}); C{p} = sparse(C{p}); end;
         else
            if issparse(C{p}); C{p} = full(C{p}); end; 
         end
	     if (nargin == 7)
            if ~all(size(X0{p}) == n) | ~all(size(Z0{p}) == n); 
               error('validate: blk and X0,Z0 are not compatible'); end; 
            if (length(y0) ~= m); 
                error('validate: length of b and y0 not compatible'); end;  
            if (norm([X0{p}-X0{p}' Z0{p}-Z0{p}'],inf) > 2e-13); 
                error('validate: X0,Z0 not symmetric'); end;
            if (nnz(X0{p}) < spdensity*n2) | (numblk > 1) ; 
               if ~issparse(X0{p}); X0{p} = sparse(X0{p}); end; 
            else
               if issparse(X0{p}); X0{p} = full(X0{p}); end;
            end;
            if (nnz(Z0{p}) < spdensity*n2) | (numblk > 1); 
               if ~issparse(Z0{p}); Z0{p} = sparse(Z0{p}); end; 
            else
               if issparse(Z0{p}); Z0{p} = full(Z0{p}); end;
            end;
         end
      elseif strcmp(pblk{1},'q') | strcmp(pblk{1},'l') | strcmp(pblk{1},'u'); 
         if (size(C{p},2) ~= 1); 
            error(['validate: ',num2str(p),'-th block of C must be column vectors']); 
         end;
         if (size(C{p},1) ~= n); 
            error(['validate: blk and C are not compatible']); 
         end; 
         if (size(At{p,1}) == [m n] & m~=n); 
            At{p,1} = At{p,1}'; end
         if ~all(size(At{p,1}) == [n,m]); 
            error('validate: blk and At not compatible'); end;
         if ~issparse(At{p,1}); 
            At{p,1} = sparse(At{p,1}); end; 
         if (nnz(C{p}) < spdensity*n); 
            if ~issparse(C{p}); C{p} = sparse(C{p}); end; 
         else
            if issparse(C{p}); C{p} = full(C{p}); end;
         end;
         if (nargin == 7)
            if ~all([size(X0{p},2) size(Z0{p},2)]==1); 
               error(['validate: ',num2str(p),'-th block of X0,Z0 must be column vectors']);
            end
            if ~all([size(X0{p},1) size(Z0{p},1)]==n); 
               error(['validate: blk, and X0,Z0, are not compatible']); 
            end              
            if (nnz(X0{p}) < spdensity*n); 
               if ~issparse(X0{p}); X0{p} = sparse(X0{p}); end; 
            else
               if issparse(X0{p}); X0{p} = full(X0{p}); end;
            end
            if (nnz(Z0{p}) < spdensity*n); 
               if ~issparse(Z0{p}); Z0{p} = sparse(Z0{p}); end; 
            else
               if issparse(Z0{p}); Z0{p} = full(Z0{p}); end;
            end
            if strcmp(pblk{1},'u') 
               Z0{p} = sparse(n,1); 
            end
         end
      else
         error(' blk: some fields are not specified correctly'); 
      end
   end
%%
%%-----------------------------------------
%% problem dimension
%%-----------------------------------------
%%
   dim = zeros(1,4);  numblk = zeros(1,2);  
   for p = 1:size(blk,1)
      pblk = blk(p,:);
      if strcmp(pblk{1},'s')
         dim(1) = dim(1) + sum(pblk{2}); 
         numblk(1) = numblk(1) + length(pblk{2});
         nn(p) = sum(pblk{2}); 
      elseif strcmp(pblk{1},'q')
         dim(2) = dim(2) + sum(pblk{2}); 
         numblk(2) = numblk(2) + length(pblk{2});
         nn(p) = length(pblk{2}); 
      elseif strcmp(pblk{1},'l')
         dim(3) = dim(3) + sum(pblk{2}); 
         nn(p) = sum(pblk{2}); 
      elseif strcmp(pblk{1},'u')
         dim(4) = dim(4) + sum(pblk{2}); 
         nn(p) = sum(pblk{2}); 
      end
   end
%%
%%***********************************************************************
