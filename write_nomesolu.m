function [namedirfile,nomesolu] = write_nomesolu(flag_autocomparison,fsl,petr,prTD)

if flag_autocomparison == 0
    % Retro-compatibility
    %#RK-modified namedirfile = [prTD '\'];
    % namedirfile = [prTD '/'];
    namedirfile = [prTD fsl];
    %nomefilese = nzug(1:end-4);

    % Building the name of the output file
    %'RK-modified f = findstr(petr,'\');
    % f = findstr(petr,'/');
    f = strfind(petr,fsl);
    s = petr(f(end)+1:end);
    nameflong = [s(15:end) '.mat'];
    nomesolu = [petr(1:f(end)) nameflong];

elseif flag_autocomparison == 1
    f2 = strfind(prTD,fsl);
    name_mat = prTD(f2(end)+1:end);
    nomesolu = [prTD fsl name_mat '.mat'];
    namedirfile = [prTD fsl];
    %nomefilese = nzug(1:end-4);
end

end