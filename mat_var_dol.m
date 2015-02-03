function var = mat_var_dol(lsep,nf,sep,strtofind,dol)

% MAT_VAR This function looks for the requested string in the file nf and 
%         returns the numerical info written next to it, discarding the 
%         string specified in dol
%
% INPUTS  lsep:      Number of elements of sep
%         nf:        File identifier of file to be scanned
%         sep:       GUI seperator data
%         strtofind: string to be found
%         dol:       String including the required numerical info
%
% OUTPUT  var:       Information stored next to the specified string

cline = trova_info(strtofind,nf);
ind1  = numel(strtofind)+1; 
ind2  = [findstr(cline,sep) numel(cline)+1];
ldol  = numel(dol);
n     = numel(ind2);
var   = zeros(n,1);

% Storing the names of the vehicles
for ii = 1:n
    appo    = cline(ind1:ind2(ii)-1);
    var(ii) = str2num(appo(ldol+1:end));
    ind1    = ind2(ii)+lsep;
end

end

