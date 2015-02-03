function var = mat_var(lsep,nf,sep,strtofind)

cline = trova_info(strtofind,nf);
if not(isempty(cline))
    ind1 = numel(strtofind)+1; ind2 = [findstr(cline,sep) numel(cline)+1];
    n = numel(ind2);
    var = zeros(n,1);
    for k = 1:n
        var(k) = str2double(cline(ind1:ind2(k)-1));
        ind1 = ind2(k)+lsep;
    end
else
    var = 0;
end

end