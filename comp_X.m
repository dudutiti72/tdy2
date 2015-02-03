function XdFk = comp_X(Fkmod,loadP)

% comp_X This function calculates the X value of disk braked vehicles 
%        described in terms of physical parameters
%        
% INPUTS Fkmod: Total shoe-disk force multiplied by Rm/Rr ratio 
%        loadP: Load pressure
%  
% OUTPUT XdFk:  Computed X value

XdFk = Fkmod*0.35/loadP; % According to user guide

end