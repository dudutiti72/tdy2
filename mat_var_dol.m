function var = mat_var_dol(lsep,nf,sep,strtofind,dol)

cline = trova_info(strtofind,nf);
ind1 = numel(strtofind)+1; ind2 = [findstr(cline,sep) numel(cline)+1];
ldol = numel(dol);
n = numel(ind2);
var = zeros(n,1);
% Storing the names of the vehicles
for ii = 1:n
    appo = cline(ind1:ind2(ii)-1);
    var(ii) = str2num(appo(ldol+1:end));
    ind1 = ind2(ii)+lsep;
end

end

