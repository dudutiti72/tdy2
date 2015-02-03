function writerowstruct(data,nf,field,labelstring,nveicoli,sep,SorV,stru)
if strcmp(data,'%s') || strcmp(data,'%bool')
    printformat = ['%s' sep];
    printformatend = ['%s' '\r\n'];
elseif strcmp(data,'%g')
    printformat = ['%g' sep];
    printformatend = ['%g' '\r\n'];
end
A = struct('dummy',[]);
if strcmp(SorV,'struct')
    SorV = 1;
else
    SorV = 0;
end
for ii = 1:nveicoli
    if SorV
        A(ii).dummy = getfield(stru,{ii},field);
    else
        A(ii).dummy = stru(ii);
    end
    if strcmp(data,'%bool');
        if A(ii).dummy == 0
            A(ii).dummy = 'false';
        else
            A(ii).dummy = 'true';
        end
    end
end
fprintf(nf,'%s',labelstring);
fprintf(nf,printformat,A(1:nveicoli-1).dummy);
fprintf(nf,printformatend,A(nveicoli).dummy);
end