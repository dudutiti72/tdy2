function [fsl,namedirfile,nomesolu,ntrack,nzug,petr,prTD,shw,vel_0] ...
    = input_for_computation(appf,igui)

if igui == 0
    % Reading the informations about the input
    [fsl,ntrack,nzug,petr,prTD,shw,vel_0,batch_flag] = inp_TDET;
else
    nf = fopen(['GUI',appf,'.inf'],'r'); testname = fgetl(nf); fclose(nf);
    if strfind(testname,'\'), fsl = '\'; elseif strfind(testname,'/'), fsl = '/'; end

    nf = fopen(testname,'r');
    cline = trova_info('configuration=',nf);
    nzug = [cline(numel('configuration=')+1:end) '.txt'];
    cline = trova_info('track=',nf);
    ntrack = [cline(numel('track=')+1:end) '.txt'];
    petr = testname(1:strfind(testname,[fsl 'Test' fsl]));
    cline = trova_info('stspd=',nf);
    vel_0 = str2double(cline(numel('stspd=')+1:end)); vel_0 = [vel_0 1];
    prTD = pwd;
    fclose(nf);
    
    batch_flag = 0;
    shw = 1; % Variable that controls the display of the figure with the longitudinal forces
end

% Back-compatibility
if isempty(prTD)
    prTD = [petr(1:end-1) '_FILES'];
end
if batch_flag
    namedirfile = [prTD fsl];
    % to use with lancio_automatico
    name_start = strfind(prTD,fsl);
    appo = [prTD '\'];
    nomesolu = [appo prTD(name_start(end)+1:end) '.mat'];
elseif not(igui)
    % to use with the GUI
    [namedirfile,nomesolu] = write_nomesolu(0,fsl,petr,prTD);
else
    namedirfile = [prTD fsl];
    % Building the name of the output file
    f = strfind(petr,fsl);
    if exist('testname','var')
        appo = strfind(testname,fsl);
        appo = testname(appo(end)+1:end-4);
    else
        %s = petr(f(end)+1:end);
        %nameflong = [s(15:end) num2str(igui) '.mat'];
        appo = strfind(prTD,fsl);
        appo = prTD(appo(end)+1:end);
    end
    nameflong = [appo num2str(igui) '.mat'];
    nomesolu = [petr(1:f(end)) nameflong];
end


end