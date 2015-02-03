function var = mat_var(lsep,nf,sep,strtofind)

% MAT_VAR This function looks for the requested string in the file nf and 
%         returns the numerical info written next to it
%
% INPUTS  lsep:      Number of elements of sep
%         nf:        File identifier of file to be scanned
%         sep:       GUI seperator data
%         strtofind: string to be found
%
% OUTPUT  var:       Information stored next to the specified string

cline = trova_info(strtofind,nf);

if not(isempty(cline))
    ind1 = numel(strtofind)+1; 
    ind2 = [findstr(cline,sep) numel(cline)+1];
    n    = numel(ind2);
    var  = zeros(n,1);
    
    for k = 1:n
        var(k) = str2double(cline(ind1:ind2(k)-1));
        ind1   = ind2(k)+lsep;
    end
else
    var = 0;
end

end