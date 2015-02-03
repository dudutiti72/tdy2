function var = mat_var_boo(lsep,nf,sep,strtofind)

% MAT_VAR_BOO This function looks for the requested string in the file nf 
%             and returns the boolean info written next to it
%
% INPUTS      lsep:      Number of elements of sep
%             nf:        File identifier of file to be scanned
%             sep:       GUI seperator data
%             strtofind: string to be found
%
% OUTPUT      var:       Information stored next to the specified string

cline = trova_info(strtofind,nf);
ind1  = numel(strtofind)+1; 
ind2  = [findstr(cline,sep) numel(cline)+1];
n     = numel(ind2);
var   = zeros(n,1);

for k = 1:n
    appo = cline(ind1:ind2(k)-1);
    if strcmp(appo,'true')
        var(k) = 1; 
    elseif strcmp(appo,'false') 
        var(k) = 0; 
    end
    ind1 = ind2(k)+lsep;
end

end