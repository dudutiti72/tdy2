function displyascii
% tstartdisp = clock;
% This function reads the file dispris.txt and write the requested output to an ASCII
% file

timestart = clock; % Starting time of the run

nf = fopen('dispris.txt','r');
if nf >-1
    [displ,dSR,fsl,mgraph,nomesolu,nv,subp,wxls,xaxis] = ToBeDisplayed(nf);

    fclose(nf);
else
    displ.i = 'Flong';
    nv = 'all'; dSR = [];
end

ndisp = length(displ);
load(nomesolu);
%#RK-modified nfile2 = strrep(nomesolu,'.mat','_output\');
% nfile2 = strrep(nomesolu,'.mat','_output/');
nfile2 = strrep(nomesolu,'1.mat','_output');
nfile2 = strrep(nfile2,[fsl,'Projects',fsl],[fsl,'Results',fsl]);
nfile2 = [nfile2,fsl];
if ~exist(nfile2,'dir'), mkdir(nfile2); end

nveicoli = length(train);
if strcmp(nv,'all'), nv = 1:nveicoli; end;
nnv = size(nv,2);

[formato,format2nv,fstring] = TextFormat(nnv);

if not(isempty(dSR))
    rat = floor(dSR/(T(2)-T(1))); 
    if rat == 0, rat = 1; end
else
    rat = 1; 
end;

for ii = 1:ndisp
    % Initializations
    nv2 = nv; formatw = formato; Tw = T; %Pw = space;

    [info,formatw,nfile,nv2,Tw] = WhatToWrite(displ,formatw,format2nv,fstring,ii,nfile2,nv2,nveicoli,Tw,...
        Acc,BC,BP,Eb,Fbrake,Flong,Flong10,lwagcs,matTrac,MF,mfricoef,Mvt,T,TP,UBP,Y,Ypr,Yr);
    nf = fopen(nfile,'w');
    % Time and position are printed in the first two columns
    %fprintf(nf,formatw,[Tw(1:rat:end);Pw(1:rat:end);info(nv2,1:rat:end)]);
    % Time is printed in the first coloumn
    fprintf(nf,formatw,[Tw(1:rat:end);info(nv2,1:rat:end)]);

    fclose(nf);
end
if strcmp(mgraph,'Yes');
    % Plotting matlab graphs
    ind = 10; % Index of the graph
    for ii = 1:ndisp
        % Initializations
        nv2 = nv; formatw = formato; Tw = T; Pw = space;

        [info,formatw,nfile,nv2,Tw,ystring] = WhatToWrite(displ,formatw,format2nv,fstring,ii,nfile2,nv2,nveicoli,Tw,...
            Acc,BC,BP,Eb,Fbrake,Flong,Flong10,lwagcs,matTrac,MF,mfricoef,Mvt,T,TP,UBP,Y,Ypr,Yr);
        if isempty(subp)
            ind = ind+1; figure(ind); clf;
            [xstring,xw] = ChooseRightXAxis(ii,info,Pw,T,TP,Tw,xaxis,Y);
            plot(xw,info); xlabel(xstring); ylabel(ystring); grid on;
        else
            figure(ind); subplot(subp(1),subp(2),ii);
            [xstring,xw] = ChooseRightXAxis(ii,info,Pw,T,TP,Tw,xaxis,Y);
            plot(xw,info); xlabel(xstring); ylabel(ystring); grid on;
        end
    end
    if not(isempty(subp))
        % The figure is maximized
        ss = get(0,'ScreenSize');
        set(gcf,'position',ss);
    end

end
if strcmp(wxls,'Yes')
    % Writing xls files
    disp('Not supported yet');
end

% Writing of the file that stores the execution time
nfile = [nfile2 'ExecutionTime.inf'];
nf = fopen(nfile,'w');
fprintf(nf,'The execution time is %g',etime(clock,timestart));
fclose(nf);


end


function [displ,dSR,fsl,mgraph,nomesolu,nv,subp,wxls,xaxis] = ToBeDisplayed(nf)

cline = fgetl(nf);
if strcmp(cline,'WINDOWS'), fsl = '\'; elseif strcmp(cline,'LINUX'), fsl = '/'; end
cline = fgetl(nf);
nomesolu = cline;
while ~strcmp(cline,'PARAMETERS:')
    cline = fgetl(nf);
end
ii = 0;
cline = fgetl(nf);
while ~strcmp(cline,'WAGONS:')
    ii = ii+1;
    displ{ii} = cline;
    cline = fgetl(nf);
end
cline = fgetl(nf);
if strcmp(cline,'all')
    nv = 'all';
else
    nv = str2num(cline);
end
% Reading the sampling rate
cline = fgetl(nf);
cline = fgetl(nf);
dSR = str2num(cline);
% Reading the xls output
cline = fgetl(nf);
if ~ischar(cline)
    % Less inputs are provided
    wxls = 'No'; mgraph = 'No'; subp = []; xaxis = [];
else
    cline = fgetl(nf);
    if strcmp(cline,'Yes'), wxls = 'Yes'; elseif strcmp(cline,'No'), wxls = 'No'; end;
    % Reading if User wants also Matlab graphs
    cline = fgetl(nf);
    cline = fgetl(nf);
    if strcmp(cline,'Yes'), mgraph = 'Yes'; elseif strcmp(cline,'No'), mgraph = 'No'; end;
    % Reading information about sublot and x axis
    cline = fgetl(nf);
    appo = findstr(cline,' ');
    if isempty(appo)
        appo2 = [];
    else
        appo2 = str2num(cline(1:appo(1)));
    end
    if not(isempty(appo2))
        % It means that a subplot is requested
        subp(1) = appo2;
        subp(2) = str2double(cline(appo(1):appo(2)));
        cline = cline(appo(2)+1:end);
    else
        % It means that a graph for each type will be plotted
        subp = [];
    end
    if size(cline,2) == 1
        % Then it means that the same xaxis will be used for each plot
        xaxis = cline;
    else
        spaces = findstr(cline,' ');
        cline(spaces) = []; % Spaces are removed
        xaxis = cline;
    end
end

end

function [formato,format2nv,fstring] = TextFormat(nnv)

formstr = '%g'; fstring = '\t%g';
for ii = 1:nnv-1
    formstr = [formstr fstring];
end;
formato = [formstr '\t%g\r\n'];
for ii = 1:nnv
    formstr = [formstr fstring];
end;
format2nv = [formstr '\t%g\r\n'];
% % The first two columns will contain time and position
% formato = ['%g\t' formato]; format2nv = ['%g\t' format2nv];

end

function [info,formatw,nfile,nv2,Tw,ystring] = WhatToWrite(displ,formatw,format2nv,fstring,ii,nfile2,nv2,nveicoli,Tw,...
    Acc,BC,BP,Eb,Fbrake,Flong,Flong10,lwagcs,matTrac,MF,mfricoef,Mvt,T,TP,UBP,Y,Ypr,Yr)

switch displ{ii}
    case 'Position'
        % Position of each vehicle on the track, from the first vehicle
        nfile = [nfile2 'Position.txt'];
        info = (Y(:,1:size(Y,2)*0.5) - ones(size(Y,1),1)*lwagcs')'; ystring = 'Position [m]';
    case 'Speed'
        % Speed of each vehicle
        nfile = [nfile2 'Speed.txt'];
        info = 3.6*Y(:,size(Y,2)*0.5+1:end)'; ystring = 'Speed [km/h]';
    case 'Flong'
        % Longitudinal force
        nfile = [nfile2 'Flong.txt'];
        info = Flong*1e-3; ystring = 'Longitudinal force [kN]';
        if not(isempty(find(nv2 == nveicoli,1)))
            nv2(nv2 == nveicoli) = [];
            formatw = formatw(length(fstring)+1:end);
        end
    case 'Fbrake'
        % Braking force
        nfile = [nfile2 'Fbrake.txt'];
        info = 1e-3*Fbrake'; ystring = 'Braking force [kN]';
    case 'Flong10'
        % 10m Long Force
        nfile = [nfile2 'Flong10.txt'];
        info = Flong10*1e-3; ystring = '10m Longitudinal force [kN]';
        if not(isempty(find(nv2 == nveicoli,1)))
            nv2(nv2 == nveicoli) = [];
            formatw = formatw(length(fstring)+1:end);
        end
    case 'Adhesion Coefficient'
        % Istantaneous adhesion coefficient
        % FIXME: now adhesion coefficient does not depend on the track slope
        adh = 1e-3*Fbrake./(9.81*(ones(length(T),1)*Mvt));
        nfile = [nfile2 'Adh.txt']; ystring = 'Istantaneous adhesion [-]';
        info = adh';
    case 'Relative Displacement'
        nfile = [nfile2 'RelDispl.txt'];
        info = 1e3*Yr'; ystring = 'Relative distance [mm]';
        if not(isempty(find(nv2 == nveicoli,1)))
            nv2(nv2 == nveicoli) = [];
            formatw = formatw(length(fstring)+1:end);
        end
    case 'Relative Speed'
        nfile = [nfile2 'RelSpeed.txt'];
        info = Ypr'; ystring = 'Relative speed [m/s]';
        if not(isempty(find(nv2 == nveicoli,1)))
            nv2(nv2 == nveicoli) = [];
            formatw = formatw(length(fstring)+1:end);
        end
    case 'Traction Force'
        nfile = [nfile2 'Trac.txt'];
        info = 1e-3*matTrac'; ystring = 'Traction force [kN]';
    case 'Acceleration'
        nfile = [nfile2 'Acc.txt'];
        %info = [Acc;Acc(end,:)]'; ystring = 'Acceleration [m/s^2]';
        info = Acc.'; ystring = 'Acceleration [m/s^2]';
    case 'Friction Coefficient'
        nfile = [nfile2 'FC.txt'];
        info = mfricoef'; ystring = 'Friction coefficient [-]';
%         formatw = format2nv;
    case 'Pressure BP'
        nfile = [nfile2 'PBP.txt'];
        info = BP'; Tw = TP; 
        ystring = 'Pressure in BP [bar]';
    case 'Pressure BC'
        nfile = [nfile2 'PBC.txt']; ystring = 'Pressure in BC [bar]';
        info = BC'; Tw = TP;
    case 'Speed BP'
        nfile = [nfile2 'SBP.txt']; ystring = 'Speed of the air in BP [m/s]';
        info = UBP'; Tw = TP;
    case 'Mass Flow'
        nfile = [nfile2 'MF.txt'];
        info = MF'; Tw = TP; ystring = 'Mass flux [kg/s]';
        formatw = '%g'; fstring = '\t%g';
        for jj = 1:size(MF,2)-1
            formatw = [formatw fstring];
        end;
        formatw = [formatw '\t%g\r\n'];
        nv2 = 1:size(MF,2);
    case 'Braking Energy'
        nfile = [nfile2 'BE.txt']; ystring = 'Braking Energy [kJ]';
        info = 1e-3*Eb'; Tw = TP;
        


end

end

function [xstring,xw] = ChooseRightXAxis(ii,info,Pw,T,TP,Tw,xaxis,Y)

% This function provide the desired x axis
if size(xaxis,2) == 1, curx = xaxis; else curx = xaxis(ii); end
if strcmp(curx,'t')
    xstring = 'Time [s]';
    xw = Tw;
elseif strcmp(curx,'s')
    xstring = 'Distance [m]';
    if size(Pw,2) ~= length(info)
        Pw = spline(T,Y(:,1),TP);
    end
    xw = Pw;
end

end
