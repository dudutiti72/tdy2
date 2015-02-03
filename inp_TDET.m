function [fsl,ntrack,nzug,petr,prTD,shw,vel_0,batch_flag] = inp_TDET

nf = fopen('TDET.inf','r');
cline = fgetl(nf);
if strcmp(cline,'WINDOWS'), fsl = '\'; elseif strcmp(cline,'LINUX'), fsl = '/'; end
% Main folder that contains the subfolder of the requested input
petr = fgetl(nf);
% % Folder that contains the results of etrain
% pres = fgetl(nf);
% % eTrain results file name
% nres = fgetl(nf);
% Folder that will contain TrainDy results
prTD = [petr fsl fgetl(nf)]; 
% cline = str2num(fgetl(nf));
% % Starting column braking forces informations
% sibr = cline(1); 
% % Number of columns to jump to skip from a wagon to the next one
% sifl = cline(2); 
% jumpbrake = cline(3); 
shw = 1; %cline(4);
% Configuration file name
nzug = fgetl(nf);
% Name of the track file
ntrack = fgetl(nf);
% eTrain pro file name
vel_0 = str2double(fgetl(nf));
cline = fgetl(nf);
if ischar(cline) && not(isempty(str2num(cline)))
    vel_0(2) = str2num(cline);
else
    vel_0(2) = 1;
end
cline = fgetl(nf);
if ischar(cline) && not(isempty(cline))
    batch_flag = 1;
else
    batch_flag = 0;
end
fclose(nf);


end