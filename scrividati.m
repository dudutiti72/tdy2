function scrividati(arrayoutput,fdata,fid,i1,i2,siz,temp);

%# scalar fid i1 i2 siz
%# scalar dcc k

% Calcolo del formato
formato = ['%3.3f\t'];
%# fastindex
for k = 1:siz-1
	formato = [formato,fdata,'\t'];
end;
formato = [formato,fdata,'\r\n'];

% Stampa su file
fprintf(fid,formato,[temp,arrayoutput(:,i1:i2)]');

fclose(fid);

