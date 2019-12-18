%%*****************************************************************
%% SDPT3: version 4.0
%% Copyright (c) 1997 by
%% Kim-Chuan Toh, Michael J. Todd, Reha H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*****************************************************************
%% Run this script in Matlab command window 
%%*****************************************************************

   function Installmex(recompile)

   curdir = pwd;  
   fprintf(' current directory is:  %s\n',curdir);    
%%
%% generate mex files in Mexfun
%%    
   if (nargin==0); recompile = 0; end
   computer_model = computer;
   matlabversion = sscanf(version,'%f');
   matlabversion = matlabversion(1);
   tmp = version('-release'); 
   matlabrelease = str2num(tmp(1:4));
%%    
   fsp = filesep;
   if strcmp(computer_model,'PCWIN')
      str0 = ['''',matlabroot,'\extern\lib\win32\lcc\'' '];
      if (exist(eval(str0),'dir')==7)
         str1 = ['''',matlabroot,'\extern\lib\win32\lcc\libmwlapack.lib''  '];
         str2 = ['''',matlabroot,'\extern\lib\win32\lcc\libmwblas.lib''  '];
      else
         str1 = ['''',matlabroot,'\extern\lib\win32\microsoft\libmwlapack.lib''  '];
         str2 = ['''',matlabroot,'\extern\lib\win32\microsoft\libmwblas.lib''  ']; 
      end
      libstr = [str1,str2];
   elseif strcmp(computer_model,'PCWIN64')
      str_lcc = ['''',matlabroot,'\extern\lib\win64\lcc\'' '];
      str_mingw64 = ['''',matlabroot,'\extern\lib\win64\mingw64\'' '];  
      str_microsoft = ['''',matlabroot,'\extern\lib\win64\microsoft\'' '];        
      if (exist(eval(str_lcc),'dir')==7)
         str1 = ['''',matlabroot,'\extern\lib\win64\lcc\libmwlapack.lib''  '];
         str2 = ['''',matlabroot,'\extern\lib\win64\lcc\libmwblas.lib''  '];
      elseif (exist(eval(str_mingw64),'dir')==7)
         str1 = ['''',matlabroot,'\extern\lib\win64\mingw64\libmwlapack.lib''  '];
         str2 = ['''',matlabroot,'\extern\lib\win64\mingw64\libmwblas.lib''  ']; 
      elseif (exist(eval(str_microsoft),'dir')==7)
         str1 = ['''',matlabroot,'\extern\lib\win64\microsoft\libmwlapack.lib''  '];
         str2 = ['''',matlabroot,'\extern\lib\win64\microsoft\libmwblas.lib''  '];
      else
         error(' no compiler found'); 
      end 
      libstr = [str1,str2];
   else
      libstr = '  -lmwlapack -lmwblas  '; 
   end
   if strfind(computer_model,'MAC')
      mexcmd = 'mex -largeArrayDims  -output ';
   else
      mexcmd = 'mex -O -largeArrayDims  -output ';
   end       
%%
%%
%%
   src = [curdir,fsp,'Solver',fsp,'Mexfun']; 
   eval(['cd ','Solver',fsp,'Mexfun']); 
   fprintf ('\n Now compiling the mexFunctions in:\n'); 
   fprintf (' %s\n',src);    

   fname{1} = 'mexProd2'; 
   fname{2} = 'mexProd2nz';
   fname{3} = 'mexinprod';
   fname{4} = 'mexmat';
   fname{5} = 'mexsmat';
   fname{6} = 'mexsvec';
   fname{7} = 'mexschur'; 
   fname{8} = 'mexqops';
   fname{9} = 'mexexpand';
   fname{10} = 'mexskron';
   fname{11} = 'mexnnz';
   fname{12} = 'mexschurfun';
   fname{13} = 'mexMatvec';
   fname{14} = 'mextriang';
   fname{15} = 'mextriangsp';
%%
   ext = mexext; 
   for k = 1:length(fname)
      existmex = exist([fname{k},'.',ext]); 
      if (existmex ~= 3) | (recompile)
         cmd([mexcmd,fname{k},'  ',fname{k},'.c  ',libstr]);  
      end
   end 
   cd .. 
   cd ..
%%***********************************************
   function cmd(s) 
   
   fprintf(' %s\n',s); 
   eval(s); 
%%***********************************************
