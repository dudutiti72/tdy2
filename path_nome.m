function [nomefile,nomepath] = path_nome(nomefile,fsl)

% Questa funzione estrae il nome del file ed il nome della path dalla 
% variabile stringa nomefile

%# scalar ii Np

%#RK-modified appo = findstr(nomefile,'\');
% appo = findstr(nomefile,'/');
appo = findstr(nomefile,fsl);
if ~isempty(appo)
   % Il nome è stato scritto su file specificando completamente la path.
   % In questo caso occorre estrarre esclusivamente il nome del file
   % dalla variabile nomefile.
   Np = size(nomefile,2); appo = zeros(1,Np);
   %# fastindex  
   for ii = size(nomefile,2):-1:1
      appo(Np-ii+1) = nomefile(ii);
   end;
   % L'informazione è ora scritta in caratteri ascii ed è anche capovolta
   appo92 = find(appo == 92);
   appo92 = appo92(1);
   nomefile = appo(1:appo92-1);
   nomepath = appo(appo92:Np);
   nomefile = calcola_nome(nomefile);
   nomepath = calcola_nome(nomepath);   
else
   nomepath = zeros(1,0);
end;



function nome = calcola_nome(nome)

%# scalar ii Np

Np = size(nome,2); appo = zeros(1,Np);  
%# fastindex  
for ii = size(nome,2):-1:1
   appo(Np-ii+1) = nome(ii);
end;
nome = appo;
nome = char(nome);

