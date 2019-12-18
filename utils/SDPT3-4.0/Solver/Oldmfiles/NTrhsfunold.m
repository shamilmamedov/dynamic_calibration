%%*******************************************************************
%% NTrhsfun: compute the right-hand side vector of the 
%%           Schur complement equation for the NT direction. 
%% 
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*******************************************************************

    function [rhs,EinvRc,hRd] = NTrhsfun(blk,At,par,X,Z,rp,Rd,sigmu,hRd,dX,dZ);

    spdensity = par.spdensity; 
    m = length(rp);   
    if (nargin > 8) 
       corrector = 1; 
    else 
       corrector = 0; 
       hRd = zeros(m,1); 
    end       
    hEinvRc = zeros(m,1); 
    EinvRc  = cell(size(blk,1),1); 
    rhsfree = []; 
%%
    for p = 1:size(blk,1)
        pblk = blk(p,:); 
        n = sum(pblk{2});  numblk = length(pblk{2});  
        if strcmp(pblk{1},'l')
           Rq = sparse(n,1); 
           if (corrector)
              if (norm(par.parbarrier{p}) <= inf)
                 Rq = dX{p}.*dZ{p}; 
              end
           else
              tmp  = par.dd{p}.*Rd{p};
              tmp2 = mexMatvec(At{p},tmp,1);
              hRd = hRd + tmp2;
           end
	   if iscell(sigmu)
              EinvRc{p} = sigmu{p}./Z{p}-X{p} -Rq./Z{p};
	   else
              EinvRc{p} = sigmu./Z{p}-X{p} -Rq./Z{p};
           end
           tmp2 = mexMatvec(At{p},EinvRc{p},1);  
           hEinvRc = hEinvRc + tmp2;
	elseif strcmp(pblk{1},'q') 
           Rq = sparse(n,1); 
   	   if (corrector)
              if (norm(par.parbarrier{p}) <= inf)
                 w = sqrt(par.gamz{p}./par.gamx{p}); 
                 hdx = qops(pblk,w,par.ff{p},5,dX{p}); 
                 hdz = qops(pblk,w,par.ff{p},6,dZ{p}); 
                 hdxdz = Arrow(pblk,hdx,hdz);
                 vv = qops(pblk,w,par.ff{p},5,X{p}); 
                 Vihdxdz = Arrow(pblk,vv,hdxdz,1); 
                 Rq = qops(pblk,w,par.ff{p},6,Vihdxdz); 
              end
           else
              tmp  = par.dd{p}.*Rd{p} + qops(pblk,qops(pblk,Rd{p},par.ee{p},1),par.ee{p},3);
              tmp2 = mexMatvec(At{p},tmp,1);
              hRd = hRd + tmp2;
           end
	   if iscell(sigmu)
              EinvRc{p} = qops(pblk,-sigmu{p}./(par.gamz{p}.*par.gamz{p}),Z{p},4) -X{p}-Rq;
	   else
              EinvRc{p} = qops(pblk,-sigmu./(par.gamz{p}.*par.gamz{p}),Z{p},4) -X{p}-Rq;
           end
           tmp2 = mexMatvec(At{p},EinvRc{p},1);         
           hEinvRc = hEinvRc + tmp2;
        elseif strcmp(pblk{1},'s') 
           n2 = pblk{2}.*(pblk{2}+1)/2; 
           Rq = sparse(n,n); 
           if (corrector)
              if (norm(par.parbarrier{p}) <= inf)
                 hdZ = Prod3(pblk,par.G{p},dZ{p},par.G{p}',1); 
                 hdX = spdiags(-par.sv{p},0,n,n)-hdZ;          
                 tmp = Prod2(pblk,hdX,hdZ,0);  
                 tmp = 0.5*(tmp+tmp');
                 if (numblk == 1) 
                    d = par.sv{p};
                    e = ones(pblk{2},1); 
                    Rq = 2*tmp./(d*e'+e*d'); 
                    if (nnz(Rq) <= spdensity*n2); Rq = sparse(Rq); end                  
                 else
                    Rq = sparse(n,n);
                    s = [0, cumsum(pblk{2})]; 
                    for i = 1:numblk
                       pos = [s(i)+1 : s(i+1)]; 
                       d = par.sv{p}(pos); e = ones(length(pos),1); 
                       Rq(pos,pos) = 2*tmp(pos,pos)./(d*e' + e*d'); 
                    end
                 end
              end
           else
              tmp = Prod3(pblk,par.W{p},Rd{p},par.W{p},1,par.nzlistAy{p});
              tmp2 = AXfun(pblk,At(p,:),par.permA(p,:),{tmp}); 
              hRd = hRd + tmp2;
           end 
	   if iscell(sigmu)
     	      ss = [0,cumsum(pblk{2})]; 
              sigmuvec = zeros(n,1); 
              for k = 1:length(pblk{2}); 
                 sigmuvec(ss(k)+1:ss(k+1)) = sigmu{p}(k)*ones(pblk{2}(k),1); 
              end
              tmp = spdiags(sigmuvec./par.sv{p} -par.sv{p},0,n,n);
           else
              tmp = spdiags(sigmu./par.sv{p} -par.sv{p},0,n,n);
           end
           EinvRc{p} = Prod3(pblk,par.G{p}',tmp-Rq,par.G{p},1);
           tmp2 = AXfun(pblk,At(p,:),par.permA(p,:),EinvRc(p)); 
           hEinvRc = hEinvRc + tmp2;  
        elseif strcmp(pblk{1},'u') 
           rhsfree = [rhsfree; Rd{p}]; 
        end 
    end
%% 
    rhs = rp + hRd - hEinvRc; 
    rhs = full([rhs; rhsfree]);  
%%*******************************************************************
