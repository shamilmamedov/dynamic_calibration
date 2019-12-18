%%*******************************************************************
%% solve min { t | Ax+y=b, norm(y) <= t, x\in K }
%% to detect primal infeasible problem
%%*******************************************************************

    function [dist] = detect_infeas(blk,At,C,b)

    m = length(b); 
    AAt = At; bblk = blk; CC = ops(C,'zeros'); 
    numblk = size(blk,1); 
    dd = zeros(1,m); 
    for p=1:numblk
       dd = dd + sqrt(sum(At{p}.*At{p}));        
    end
    dd = 1./min(1e4,max(1,dd'));
    D = spdiags(dd,0,m,m); 
    for p=1:numblk
       AAt{p}= AAt{p}*D; %%important to scale
    end
    b  = b.*dd;    
    AAt{numblk+1,1} = [zeros(1,m); speye(m)];
    CC{numblk+1,1} = [1; zeros(m,1)];
    bblk{numblk+1,1} = 'q'; bblk{numblk+1,2} = m+1;
    OPTIONS.gaptol = 1e-12;
    [obj,X,y,Z,info] = sqlp(bblk,AAt,CC,b,OPTIONS);
    err = max(info.dimacs); 
    dist = mean(obj);    
    if (err < 1e-6) && (dist/err > 20)
       fprintf('\n primal problem is infeasible: err = %3.2e, dist = %3.2e\n',err,dist);
    end
%%*******************************************************************
