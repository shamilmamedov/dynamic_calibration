%%**************************************************************************************
%% sqlpcheckconvg: check convergence.
%%
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%**************************************************************************************

   function [param,termcode,breakyes,restart] = sqlpcheckconvg(param,runhist);

      iter        = param.iter; 
      obj         = param.obj;
      rel_gap     = param.rel_gap; 
      gap         = param.gap; 
      mu          = param.mu; 
      prim_infeas = param.prim_infeas;
      dual_infeas = param.dual_infeas;
      homRd       = param.homRd;
      homrp       = param.homrp;
      normX       = param.normX; 
      normZ       = param.normZ; 
      termcode    = param.termcode; 
      stoplevel   = param.stoplevel; 
      prim_infeas_bad = param.prim_infeas_bad; 
      prim_infeas_min = min(param.prim_infeas_min, prim_infeas); 
      normX0      = param.normX0; 
      normZ0      = param.normZ0;
      gaptol      = param.gaptol;
      inftol      = param.inftol;  
      printlevel  = param.printlevel;
      restart     = 0; 
      breakyes    = 0; 
      infeas_meas = max(prim_infeas,dual_infeas); 
%%
      if (normX > 1e15*normX0 | normZ > 1e15*normZ0)
         termcode = 3;
         breakyes = 1; 
      end
      if (homRd < max(inftol,1e-13))
         termcode = 1;
         breakyes = 1;
      end
      if (homrp < max(inftol,1e-13))
         termcode = 2;
         breakyes = 1;
      end
      if (stoplevel)
         prim_infeas_bad = prim_infeas_bad + (prim_infeas > ...
            max(1e-10,prim_infeas_min) & (prim_infeas_min < 1e-2));
         if (mu < 1e-8)
            idx = [max(1,iter-1): iter];
         elseif (mu < 1e-4);
            idx = [max(1,iter-2): iter]; 
         else
            idx = [max(1,iter-3): iter];
         end
         idx2 = [max(1,iter-4): iter]; 
         gap_ratio2 = runhist.gap(idx2+1)./runhist.gap(idx2);
         gap_slowrate = min(0.8,max(0.6,2*mean(gap_ratio2)));
         gap_ratio = runhist.gap(idx+1)./runhist.gap(idx); 
         if (infeas_meas < 1e-4 | prim_infeas_bad) & (infeas_meas < 1e-2 & rel_gap < 5e-3) ...
            & (iter > 20) 
            gap_slow = all(gap_ratio > gap_slowrate) & (rel_gap < 5e-3);
            if (rel_gap < max(0.2*prim_infeas,1e-2*dual_infeas)) & (runhist.step(iter+1) < 0.5)
               if (printlevel); fprintf('\n  sqlp stop: relative gap < infeasibility.'); end
               termcode = -1;
               breakyes = 1;           
            elseif (gap_slow) 
               if (printlevel); fprintf('\n  sqlp stop: progress is too slow.'); end
               termcode = -5; 
               breakyes = 1;
            end  
         elseif (prim_infeas_bad) & (iter >50) & all(gap_ratio > gap_slowrate)
            if (printlevel); fprintf('\n  sqlp stop: progress is bad.'); end
            termcode = -5;
            breakyes = 1; 
         elseif (infeas_meas < 1e-8) & (gap > 1.2*mean(runhist.gap(idx))) & (rel_gap < 5e-3)
            if (printlevel); fprintf('\n  sqlp stop: progress is bad.'); end
            termcode = -5;
            breakyes = 1;  
         end
         if (max(runhist.infeas) > 1e-4) & (min(runhist.infeas) < 1e-4 | prim_infeas_bad) 
            rel_gap2 = abs(diff(obj))/(1+sum(abs(obj))); 
            if (rel_gap2 < 1e-3); 
               step_short = all(runhist.step([iter:iter+1]) < 0.05) ;
            elseif (rel_gap2 < 1) 
               idx = [max(1,iter-3): iter+1];
               step_short = all(runhist.step(idx) < 0.03); 
	    else
               step_short = 0; 
            end
            if (step_short) 
               if (printlevel); fprintf('\n  sqlp stop: steps too short consecutively'); end
               termcode = -5; 
               breakyes = 1;      
            end
         end
	 if (iter > 20) & (infeas_meas > 1e-4) & all(runhist.step([iter-3:iter]) > 0.1) ...
            & (runhist.pinfeas(iter+1) > 100*prod(1.1-runhist.step([iter-3:iter]))*runhist.pinfeas(iter-3))
            %%if (printlevel); fprintf('\n  sqlp stop: lack of progress'); end
            %%termcode = -5;
            %%breakyes = 1; 
         end
         if (iter > 3 & iter < 20) & (max(runhist.step(max(1,iter-3):iter+1)) < 1e-3) ...
            & (infeas_meas > 1) & (min(homrp,homRd) > 1000*inftol) 
            if (stoplevel == 2) 
               if (printlevel)
                  fprintf('\n *** Too many tiny steps, advisable to restart sqlp'); 
                  fprintf(' with the following iterate.')
                  fprintf('\n *** Suggestion: [X0,y0,Z0] = infeaspt(blk,At,C,b,2,1e5);'); 
                  fprintf('\n  sqlp stop: steps too short consecutively'); 
               end
               termcode = -5; 
               breakyes = 1;             
            elseif (stoplevel == 3)
               if (printlevel)
                  fprintf('\n *** Too many tiny steps even')
                  fprintf(' after restarting sqlp');                
                  fprintf('\n  sqlp stop: steps too short consecutively');
               end
               termcode = -5;
               breakyes = 1;              
            else 
               if (printlevel)
                  fprintf('\n *** Too many tiny steps:')
                  fprintf('  restarting sqlp with the following iterate.')
                  fprintf('\n *** [X,y,Z] = infeaspt(blk,At,C,b,2,1e5);'); 
               end
	       prim_infeas_min = 1e20; 
   	       restart = 1; 
            end
         end
      end
      if (max(rel_gap,infeas_meas) < gaptol)
         if (printlevel)
            fprintf('\n  sqlp stop: max(relative gap, infeasibilities) < %3.2e',gaptol);
         end
         termcode = 0;
         breakyes = 1;
      end
%%
%%
      param.prim_infeas_bad = prim_infeas_bad;
      param.prim_infeas_min = prim_infeas_min;
%%**************************************************************************************
