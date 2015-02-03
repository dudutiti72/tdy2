function [nomefile,nomepath,A] = leggifile(nomefile);

% Questa funzione estrae l'input informativo contenuto nel file "nomefile".
% E' necessaria per eseguire il controlo sulla compatibilità dell'input 
% inserito dall'Utente e per l'acquisizione dei dati per realizzare l'analisi
% modale.

% Estrazione del nome del file d'input e della path.
[nomefile,nomepath] = path_nome(nomefile);

appo = nomefile(1:findstr(nomefile,'.')-1);
% Si memorizza la directory corrente
old_dir = pwd;
% Si cambia la directory
cd(nomepath);

% Memorizzazione del contenuto del file d'input nella matrice "A"
if exist('_MATCOM_') ~= 0
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(appo);
end;
% Si ritorna alla vecchia directory
cd(old_dir);
