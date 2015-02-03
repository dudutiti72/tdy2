function scrividatic(arrayoutput,fdata,fid,i1,i2,siz,temp)

%# scalar fid i1 i2 siz
%# scalar dcc k inte

% Calcolo del formato
formato = ['%3.3f\t'];
%# fastindex
for k = 1:siz-1
	formato = [formato,fdata,'\t'];
end;
formato = [formato,fdata,'\r\n'];

% Stampa su file
S = sprintf(formato,[temp,arrayoutput(:,i1:i2)]');
S = strrep(S,'.',',');
inte = 2^13; 
siz = floor(length(S)/inte);
for k = 1:siz
    fprintf(fid,'%s  ',S((k-1)*inte+1:k*inte));
end;
fprintf(fid,'%s  ',S(siz*inte+1:length(S)));
%fprintf(fid,formato,[temp,arrayoutput(:,i1:i2)]');

fclose(fid);

