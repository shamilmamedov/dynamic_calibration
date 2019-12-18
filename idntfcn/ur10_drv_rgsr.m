function drv = ur10_drv_rgsr(q2d_est)%(qd_fltrd)
%     drv = zeros(6,1);
% %     for i = 1:6
% %        drv(i) =  0.5*qd_fltrd(i)^2;
% %     end
%     for i = 1:6
%        drv(i) =  q2d_est(i);
%     end
    drv = zeros(6,4);
    for i = 3:6
       drv(i,i-2) = q2d_est(i); 
    end
end