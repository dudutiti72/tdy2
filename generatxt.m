function generatxt(nomefile,dati)
nf = fopen(nomefile,'w');
if size(dati,2) > size(dati,1)
    dati = dati.';
end
siz = size(dati,2); formato = '';
for ii = 1:siz
    formato = [formato,'%g\t'];
end
formato = [formato(1:end),'\r\n'];
fprintf(nf,formato,dati');
fclose(nf);
