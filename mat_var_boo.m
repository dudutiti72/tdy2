function var = mat_var_boo(lsep,nf,sep,strtofind)

cline = trova_info(strtofind,nf);
ind1 = numel(strtofind)+1; ind2 = [findstr(cline,sep) numel(cline)+1];
n = numel(ind2);
var = zeros(n,1);
for k = 1:n
    appo = cline(ind1:ind2(k)-1);
    if strcmp(appo,'true'), var(k) = 1; elseif strcmp(appo,'false'), var(k) = 0; end
    ind1 = ind2(k)+lsep;
end

end