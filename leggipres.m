function Pres = leggipres(namedirfile)

% Lettura del file sulle pressioni e creazione della matrice Pres

oldir = pwd;

% nomedir = [nomepath nomefilese '_FILES\'];
cd(namedirfile);
% Memorizzazione del contenuto del file d'input nella matrice "Pres"
if exist('_MATCOM_') ~= 0
   % Istruzione letta solo dal MIDEVA.
   load ('dataSA.txt','Pres');
else
   % Istruzione letta solo dal MATLAB.
   try
       load ('dataSA.txt','Pres');
   catch
       Pres = [];
       cd(oldir);
       return
       error('Il file di pressioni fornito non e'' corretto');
   end;
   Pres = eval('dataSA');
end;
% Si ritorna alla vecchia directory
cd(oldir);
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^