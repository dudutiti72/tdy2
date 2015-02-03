function [DtP,pFmot,Pres,TimeP] = efilepres(lwag,namedirfile,nveicoli,posinput,tFin,tIn) %,WgSwOff)

%# scalar DtP
%# scalar nveicoli
%# scalar dimP dimF dimI 

% Carico i valori di tempo e pressione letti dai file nell'array Pres
Pressioni = leggipres(namedirfile);
if isempty(Pressioni)
    Pres = []; DtP = []; pFmot = []; TimeP = [];
    return
elseif size(Pressioni,2) ~= size(posinput,2)+4
	error('The columns number of input pressure file is wrong');    
end;

% % Eliminazione colonne con tutti 0 perchè distributore disabilitato
% rmv = []; 
% for ii = 1:size(WgSwOff,2);
%     appo = find(posinput == WgSwOff(ii));
%     rmv = [rmv, appo];
% end;
% Pressioni(:,rmv+3) = [];
% posinput(rmv) = [];

% Ridefinizione dei tempi dell'analisi Stand-Alone
if tIn == -100
    tIn = 0;
end;
if tFin == -100
    tFin = Pressioni(end,1);
end;
% Added 08/11/07
tFin = min([Pressioni(end,1) abs(tFin)]);

% Definizione tempo di acquisizione balena
DtP = Pressioni(2,1)-Pressioni(1,1);
dimP = floor((tFin-tIn)./DtP)+1; %numero di righe totali da leggere
dimI = round((tIn/DtP))+1; %numero di riga da cui iniziare a leggere
dimF = round((tFin/DtP))+1; %numero di riga a cui terminare la lettura

% Memorizzo gli step temporali di acquisizione dei dati
TimeP = 0:DtP:(tFin-tIn);

% % Inizializzo la matrice delle pressioni
% Pres = zeros(dimP,nveicoli);

% Si interpolano i valori delle pressioni tenendo conto della distanza
csum = cumsum([0, lwag(1:nveicoli-1)])+lwag/2;
if nveicoli > 1
    Pres = (interplineareFr(csum(posinput)',Pressioni(dimI:dimF,4:end-1)',csum))';
else
    Pres = Pressioni(dimI:dimF,4);
end
%Pres = (interplinearedeltacost(csum(posinput)',Pressioni(dimI:dimF,4:end-1)',csum))';
%Pres = (interplinearedeltacost(csum(posinput),Pressioni(dimI:dimF,4:end-1),csum))';
% Si definisce il vettore pFmot per la percentuale di applicazione della
% forza motrice (i dati sono contenuti in (Pressioni,2))
%pFmot = 0.01*Pressioni(dimI:dimF,2);
pFmot = Pressioni(dimI:dimF,2);