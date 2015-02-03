function [nomefile,nomefilese,nomepath,A] = infopreliminari(shw)

%# scalar nf
if shw
    disp(sprintf('           _________                _____         ________   '));
    disp(sprintf('           ___  ___/______________ ____(_)_______ ___  __ \\_____  __ '));
    disp(sprintf('           __  /    __  ___/_  __ `/__  / __  __ \\__  / / /__  / / / '));
    disp(sprintf('           _/ /     _/ /    / /_/ / _/ /  _/ / / /_/ /_/ / _  /_/ / '));
    disp(sprintf('           /_/      /_/     \\__,_/  /_/   /_/ /_/ /_____/  _\\__, / '));
    disp(sprintf('                                                           /____/    UIC VERSION 1.0'));

    disp(sprintf('          TRAIN DYNAMICS SIMULATOR October 2008 Release -TD2008-'));
    disp(sprintf('\r\n'));

    disp(sprintf('data acquisition...'));
    disp(sprintf('\r\n'));
end;


% Input del problema
nf = fopen('scambio.inf','r');	  	   % Apro il file di scambio in sola lettura
nomefile = fscanf(nf,'%c',256);		   % leggo i primi 256 caratteri
fclose(nf);
cr = find(nomefile == char(13));	   % Carriage return: individuo in 'nomefile' il primo 
                                       % 'a capo'(char(13)) determino nomefile come il vecchio
nomefile = nomefile(1:cr(1)-1);		   % Nomefile ma fino al suo primo 'a capo'   


% Lettura dei dati e caricamento della matrice A che legge il file di input
[nomefile,nomepath,A] = leggifile(nomefile);

% Nome del file senza estensione
nomefilese = nomefile(1:find (nomefile == char(46))-1);
