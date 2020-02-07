function writeTrajectoryCoefficientsIntoFile(a, b, c_pol)
% -----------------------------------------------------------------------
% the function takes the parameters of the trajectory for identification
% that consists of trancated Fourier series and 5th order polynomial
% and writes in into three files that are used by UR in order to 
% perform trajectory.
% Input:
%       a - sine coefficients in the truncated fourier series
%       b - cosine coefficients in the truncated fourier series
%       c_pol - fifth order polynomial coefficinets
% -----------------------------------------------------------------------
prcsn = 10;
a_coefs = {};
b_coefs = {};
a_coefs{1} = 'a1 = ['; a_coefs{2} = 'a2 = ['; a_coefs{3} = 'a3 = ['; 
a_coefs{4} = 'a4 = ['; a_coefs{5} = 'a5 = ['; a_coefs{6} = 'a6 = [';
b_coefs{1} = 'b1 = ['; b_coefs{2} = 'b2 = ['; b_coefs{3} = 'b3 = ['; 
b_coefs{4} = 'b4 = ['; b_coefs{5} = 'b5 = ['; b_coefs{6} = 'b6 = [';
for i = 1:size(a,2)
    if i < size(a,2)
        for j = 1:6
            a_coefs{j} = strcat(a_coefs{j}, num2str(a(j,i), prcsn), ',');
            b_coefs{j} = strcat(b_coefs{j}, num2str(b(j,i), prcsn), ',');
        end 
    elseif i == size(a,2)
        for j = 1:6
            a_coefs{j} = strcat(a_coefs{j}, num2str(a(j,i), prcsn), ']\n');
            b_coefs{j} = strcat(b_coefs{j}, num2str(b(j,i), prcsn), ']\n');
        end
    end
end

c_coefs = {};
c_coefs{1} = 'c1 = ['; c_coefs{2} = 'c2 = ['; c_coefs{3} = 'c3 = ['; 
c_coefs{4} = 'c4 = ['; c_coefs{5} = 'c5 = ['; c_coefs{6} = 'c6 = [';
for i = 1:size(c_pol,2)
    if i < size(c_pol,2)
        for j = 1:6
            c_coefs{j} = strcat(c_coefs{j}, num2str(c_pol(j,i), prcsn+2), ',');
        end
    elseif i == size(c_pol,2)
        for j = 1:6
            c_coefs{j} = strcat(c_coefs{j}, num2str(c_pol(j,i), prcsn+2), ']\n');
        end
    end
end

fileID_a = fopen('trajectory_optmzn/coeffs4_UR/a_coeffs.script','w');
fileID_b = fopen('trajectory_optmzn/coeffs4_UR/b_coeffs.script','w');
fileID_c = fopen('trajectory_optmzn/coeffs4_UR/c_coeffs.script','w');
for i = 1:6
    fprintf(fileID_a, a_coefs{i});
    fprintf(fileID_b, b_coefs{i});
    fprintf(fileID_c, c_coefs{i});
end
fclose(fileID_a);
fclose(fileID_b);
fclose(fileID_c);

