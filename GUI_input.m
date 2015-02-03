function [Dcoll,epsilon,indexes,kcoll,Lcoll,lwag,Mvt,nveicoli,P1,pathGUI,...
    ploco,posinput,sep,solver,SR,traccia,trainGUI] = GUI_input(appf,fsl,ntrack,nzug,petr,prTD) % prTD shall be removed from the inputs as it is not used any more 
                                                                                               % MOREOVER, it can be removed from input_for_computation outputs as it not used 
                                                                                               % anywhere else. Finallym the outputs not used in tRTUf (~) can also be removed [s!]
% GUI_INPUT After input_for_computation.m, this function is called to start
%           importing the project data from the .txt files
%
% INPUTS    appf      String appended to characterize test ('' for GUI)
%           fsl:      '\' for Windows '/' for Linux
%           ntrack:   Name of track info file (.txt)
%           nzug:     Name of configuration file (.txt)
%           petr:     Main folder where project data are located and 
%                     results are stored
%           prTD:     Current working directory, where also the main folder
%                     is located
%
% OUTPUTS   Dcoll:    Internal hose couplings diameter [m]
%           epsilon:  Roughness of Brake Pipe [m]
%           indexes:  Struct array with position of wagons equipped with
%                     disk or block brake system
%           kcoll:    Concentrated pressure loss factor of hose couplings
%           Lcoll:    Length of hose couplings [m]
%           lwag:     Vector with vehicle lengths [m]
%           Mvt:      Vector with total mass of each vehicle including
%                     rotary masses [kg]
%           nveicoli: Total number of vehicles
%           P1:       Initial absolute Brake Pipe pressure [Pa] 
%           pathGUI:  Location where results will be stored
%           ploco:    Vector with positions of locomotives
%           posinput: Vector with indices, from 1 to nveicoli 
%           sep:      GUI seperator data
%           solver:   Solver of ODE's  (default is the 'ode23t')
%           SR:       Sampling rate [s]
%           traccia:  Array with track info
%           trainGUI: Struct array with info about each vehicle

global Tamb

% Initialization of trainGUI (struct containing the GUI input)
trainGUI = struct('name','loco','type','loco');

% Read configuration file
[Dcoll,epsilon,kcoll,Lcoll,lwag,nloco,nwago,nveicoli,P1,pathGUI,ploco, ...        % lwag is calculated exactly the same way in both
    pwago,sep,solver,SR,Tamb,trainGUI] = configuration(appf,fsl,nzug,trainGUI);   % configuration and extr_var2 functions. lwag from 

% Filling "train" with information about locomotives and wagons
trainGUI = fill_train_w_l(fsl,nloco,pathGUI,ploco,sep,trainGUI,petr);
trainGUI = fill_train_w_w(fsl,nwago,pathGUI,pwago,sep,trainGUI,petr);

% Filling "train" with information about buffers and draw gearsTRAIN_CONF_NUM
trainGUI = fill_train_w_bg_dg(fsl,nveicoli,pathGUI,sep,trainGUI);

% Calculate and store brake system data for each vehicle [SBB]
[indexes,lwag,Mvt,posinput,trainGUI] = extr_var2(nveicoli,trainGUI);

% Read track file [SBB]
traccia = tracciato(fsl,ntrack,pathGUI,sep);

% Actually the GUI does not support the complex model of couplings [n]
for ii = 1:nveicoli
    trainGUI(ii).cc = 0;
end

end

function [Dcoll,epsilon,kcoll,Lcoll,lwag,nloco,nwago,nveicoli,P1,pathGUI,ploco,pwago,sep,solver,SR,Tamb,...
    trainGUI] = configuration(appf,fsl,nzug,trainGUI)

% CONFIGURATION This function extracts the configuration of the train from
%               the .txt file located in configuration folder
%
% INPUTS        appf:     String appended to characterize test
%               fsl:      '\' for Windows '/' for Linux
%               nzug:     String with name of Configuration file(.txt)
%               trainGUI: Struct array with info about each vehicle
%
% OUTPUTS       Dcoll:    Internal hose couplings diameter [m]
%               epsilon:  Roughness of Brake Pipe [m]
%               kcoll:    Concentrated pressure loss factor of hose 
%                         couplings
%               Lcoll:    Length of hose couplings [m]
%               lwag:     Vector with vehicle lengths [m]
%               nloco:    Number of locomotives
%               nwago:    Number of wagons
%               nveicoli: Total number of vehicles (nloco + nwago)
%               P1:       Initial absolute Brake Pipe pressure [Pa]   
%               pathGUI:  Location where results will be stored
%               ploco:    Vector with positions of locomotives
%               pwago:    Positions of wagons
%               sep:      GUI seperator data
%               solver:   Solver of ODE's
%               SR:       Sampling rate [s]
%               Tamb:     Atmospheric temperature [K]
%               trainGUI: Modified struct array


% Determine path containing the input from TrainDy GUI
nf       = fopen(['GUI',appf,'.inf'],'r'); 
testname = fgetl(nf); fclose(nf);
pathGUI  = testname(1:strfind(testname,[fsl 'Test' fsl])-1);
npath    = [pathGUI fsl 'Configuration' fsl];

% Instead of the above lines of code, put petr as input and define
% npath   = [petr 'Configuration' fsl];
% pathGUI = petr(1:end-1); % For compatibility with rest of the code 
% which gives exactly the same directory [s!]

%trainGUI(1).nconf = [nzug(1:end-3) 'txt'];

trainGUI(1).nconf = nzug(1:end-4);  % Name of config. without txt extension

%nzug = trainGUI(1).nconf;
nzug  = [nzug(1:end-3) 'txt'];
nfile = [npath nzug];
nf    = fopen(nfile,'r');

% Start reading in configuration data [SBB]
sep      = '_;_';                   % GUI Separator data
lsep     = numel(sep); 
cline    = trova_info('pos=1',nf);
appo     = findstr(cline,sep);

% Number of vehicles
nveicoli = str2double(cline(appo(end)+3:end));
% Reading information and storing it into the struct array trainGUI
trainGUI = train_conf_str(trainGUI,'type=',lsep,nf,nveicoli,sep,'name');
trainGUI = train_conf_str(trainGUI,'manoeuvre=',lsep,nf,nveicoli,sep,'mano');
trainGUI = train_conf_num(trainGUI,'wagLength=',lsep,1,nf,nveicoli,sep,'lwag');
trainGUI = train_conf_num(trainGUI,'brPLength=',lsep,1,nf,nveicoli,sep,'lBP');
trainGUI = train_conf_boo(trainGUI,'gpCStat=',lsep,nf,nveicoli,sep,'UnC');
trainGUI = train_conf_num(trainGUI,'load=',lsep,1,nf,nveicoli,sep,'load');
trainGUI = train_conf_num(trainGUI,'tare=',lsep,1,nf,nveicoli,sep,'tare');
trainGUI = train_conf_num(trainGUI,'fk=',lsep,1,nf,nveicoli,sep,'Fk');
trainGUI = train_conf_num(trainGUI,'bcExpTp=',lsep,1,nf,nveicoli,sep,'pBCexp');
trainGUI = train_conf_num(trainGUI,'bcNomTp=',lsep,1,nf,nveicoli,sep,'pBC');
for ii = 1:nveicoli
    % These values are imposed and used when the brake data are computed 
    % using the UIC leaflet 544-1
    trainGUI(ii).bblp = 3.8; 
    trainGUI(ii).dblp = 3.8; 
end
% trainGUI = copy_field(trainGUI,'pBCexp','bblp');
% trainGUI = copy_field(trainGUI,'pBCexp','dblp');
% trainGUI = copy_field(trainGUI,'pBC','bblp');
% trainGUI = copy_field(trainGUI,'pBC','dblp');
trainGUI = train_conf_str(trainGUI,'cv=',lsep,nf,nveicoli,sep,'CV');
trainGUI = train_conf_boo(trainGUI,'cvStat=',lsep,nf,nveicoli,sep,'CVactv');
trainGUI = train_conf_dol(trainGUI,'$BRAKE_REGIME','brReg=',lsep,nf,nveicoli,sep,[0 1],'RgBk');
trainGUI = train_conf_num(trainGUI,'ft95=',lsep,1,nf,nveicoli,sep,'t95');
trainGUI = train_conf_num(trainGUI,'ft100=',lsep,1,nf,nveicoli,sep,'tmx');
trainGUI = train_conf_num(trainGUI,'contBl=',lsep,0.01,nf,nveicoli,sep,'pbb');
trainGUI = train_conf_num(trainGUI,'contDi=',lsep,0.01,nf,nveicoli,sep,'pdb');
trainGUI = train_conf_num(trainGUI,'gap=',lsep,1,nf,nveicoli,sep,'gap');
trainGUI = train_conf_str(trainGUI,'bufGearsF=',lsep,nf,nveicoli,sep,'nbgf');
trainGUI = train_conf_str(trainGUI,'drGearsF=',lsep,nf,nveicoli,sep,'ndgf');
trainGUI = train_conf_str(trainGUI,'bufGearsR=',lsep,nf,nveicoli,sep,'nbgr');
trainGUI = train_conf_str(trainGUI,'drGearsR=',lsep,nf,nveicoli,sep,'ndgr');
trainGUI = train_conf_fric(trainGUI,'$EXT_BLOCK_FRICTION_LAWS','frictLaw=',lsep,nf,nveicoli,sep,[1 2],'frlaw');

% Generic data [SBB]
cline   = trova_info('inthcd=',nf);
Dcoll   = str2double(cline(numel('inthcd=')+1:end))*1e-3;
cline   = trova_info('lenhc=',nf);
Lcoll   = str2double(cline(numel('lenhc=')+1:end));
cline   = trova_info('cplfhc=',nf);
kcoll   = str2double(cline(numel('cplfhc=')+1:end));
cline   = trova_info('srdw=',nf);
SR      = str2double(cline(numel('srdw=')+1:end));
cline   = trova_info('envTemp=',nf);
Tamb    = str2double(cline(numel('envTemp=')+1:end));
cline   = trova_info('rouBP=',nf);
epsilon = str2double(cline(numel('rouBP=')+1:end))*1e-3;
cline   = trova_info('solver=',nf);

if isempty(cline)
    solver = 'ode23t';
else
    solver = cline(numel('solver=')+1:end); 
end

fclose(nf);

% Reading of starting pressure
nfile    = testname;
appo     = strfind(testname,fsl);
testname = testname(appo(end)+1:end-4);

trainGUI(1).test = testname;
% nfile = [npath testname '.txt'];
nf    = fopen(nfile,'r');
cline = trova_info('initpres=',nf);
P1    = (1 + str2double(cline(numel('initpres=')+1:end)))*1e5;
fclose(nf);

% Other outputs
nwago = 0; nloco = 0; lwag = zeros(1,nveicoli); ploco = lwag; pwago = lwag;

for ii = 1:nveicoli
    if isempty(trainGUI(ii).mano)
        nwago = nwago + 1;
        %pwago = [pwago ii];
        pwago(nwago) = ii;
    else
        nloco = nloco + 1;
        %ploco = [ploco ii];
        ploco(nloco) = ii;
    end
    lwag(ii) = trainGUI(ii).lwag;
end
pwago = pwago(1:nwago); 
ploco = ploco(1:nloco);

end

function trainGUI = train_conf_str(trainGUI,field,lsep,nf,nveicoli,sep,trainfield)

% TRAIN_CONF_STR This function reads in the requested information from the
%                configuration file and stores it to the specified train 
%                field for each vehicle. The retrieved information is a 
%                string
%
% INPUTS         trainGUI:   Struct array with info about each vehicle                
%                field:      Field to look for in .txt file
%                lsep:       Number of elements of sep
%                nf:         File identifier of configuration file
%                nveicoli:   Total number of vehicles
%                sep:        GUI seperator data
%                trainfield: Name of field where the info will be stored
%
% OUTPUT         trainGUI:   Updated struct array

cline = trova_info(field,nf);

if not(isempty(cline))
    ind1 = numel(field) + 1; 
    ind2 = findstr(cline,sep);
    
    % Storing the names of the vehicles
    for ii = 1:nveicoli-1
        %trainGUI(ii) = setfield(trainGUI(ii),trainfield,cline(ind1:ind2(ii)-1));
        appo     = cline(ind1:ind2(ii)-1);
        trainGUI = setfield(trainGUI, {ii,1}, trainfield, {1:numel(appo)}, appo);
        ind1     = ind2(ii)+lsep;
    end
    appo     = cline(ind1:end);
    trainGUI = setfield(trainGUI, {nveicoli,1},trainfield,{1:numel(appo)},appo);
else
    trainGUI    = setfield(trainGUI, {1,1}, trainfield, {1}, 1);
    trainGUI(1) = setfield(trainGUI(1), trainfield, []);
end

end

function trainGUI = train_conf_num(trainGUI,field,lsep,mol,nf,nveicoli,sep,trainfield)

% TRAIN_CONF_NUM This function reads in the requested information from the
%                configuration file and stores it to the specified train 
%                field for each vehicle . The retrieved information is a 
%                numeric value
%
% INPUTS         trainGUI:   Struct array with info about each vehicle
%                field:      Field to look for in .txt file
%                lsep:       Number of elements of sep
%                mol:        [?]
%                nf:         File identifier of configuration file
%                nveicoli:   Total number of vehicles
%                sep:        GUI seperator data
%                trainfield: Name of field where the info will be stored
%
% OUTPUT         trainGUI:   Updated struct array

cline = trova_info(field,nf);

if not(isempty(cline))
    ind1 = numel(field) + 1; 
    ind2 = findstr(cline,sep);
    
    % Storing the names of the vehicles
    for ii = 1:nveicoli-1
        %trainGUI(ii) = setfield(trainGUI(ii),trainfield,cline(ind1:ind2(ii)-1));
        appo = str2double(cline(ind1:ind2(ii)-1));
        if isnan(appo)
            trainGUI     = setfield(trainGUI, {ii,1}, trainfield, {1}, 1);
            trainGUI(ii) = setfield(trainGUI(ii), trainfield, []);
        else
            trainGUI     = setfield(trainGUI, {ii,1}, trainfield, {1}, mol*appo);
        end
        ind1 = ind2(ii) + lsep;
    end
    appo = str2double(cline(ind1:end));
    if isnan(appo)
        trainGUI = setfield(trainGUI, {nveicoli,1}, trainfield, {1}, 1);
        trainGUI(nveicoli) = setfield(trainGUI(nveicoli), trainfield, []);
    else
        trainGUI = setfield(trainGUI, {nveicoli,1},trainfield,{1},appo*mol);
    end
else
    trainGUI     = setfield(trainGUI, {1,1}, trainfield, {1}, 1);
    trainGUI(1)  = setfield(trainGUI(1), trainfield, []);
end

end

function trainGUI = train_conf_boo(trainGUI,field,lsep,nf,nveicoli,sep,trainfield)

% TRAIN_CONF_BOO This function reads in the requested information from the
%                configuration file and stores it to the specified train 
%                field for each vehicle . The retrieved information is a 
%                boolean variable
%
% INPUTS         trainGUI:   Struct array with info about each vehicle
%                field:      Field to look for in .txt file
%                lsep:       Number of elements of sep
%                nf:         File identifier of configuration file
%                nveicoli:   Total number of vehicles
%                sep:        GUI seperator data
%                trainfield: Name of field where the info will be stored
%
% OUTPUT         trainGUI:   Updated struct array

cline = trova_info(field,nf);

if not(isempty(cline))
    ind1 = numel(field) + 1; 
    ind2 = findstr(cline,sep);
    
    % Storing the names of the vehicles
    for ii = 1:nveicoli-1
        %trainGUI(ii) = setfield(trainGUI(ii),trainfield,cline(ind1:ind2(ii)-1));
        appo = cline(ind1:ind2(ii)-1);
        if strcmp(appo,'true'), appo = 1; elseif strcmp(appo,'false'), appo = 0; end
        trainGUI = setfield(trainGUI, {ii,1}, trainfield, {1}, appo);
        ind1     = ind2(ii)+lsep;
    end
    appo = cline(ind1:end);
    if strcmp(appo,'true'), appo = 1; elseif strcmp(appo,'false'), appo = 0; end
    trainGUI = setfield(trainGUI, {nveicoli,1},trainfield,{1},appo);
else
    trainGUI    = setfield(trainGUI, {1,1}, trainfield, {1}, 1);
    trainGUI(1) = setfield(trainGUI(1), trainfield, []);
end

end

function trainGUI = train_conf_dol(trainGUI,dol,field,lsep,nf,nveicoli,sep,tmap,trainfield)

% TRAIN_CONF_DOL This function reads in the requested information from the
%                configuration file and stores it to the specified train 
%                field for each vehicle . The retrieved information is 
%                included in a string and after its extraction it is mapped
%                to numeric values according to variable tmap
%
% INPUTS         trainGUI:   Struct array with info about each vehicle
%                dol:        String including the required info for each
%                            vehicle
%                field:      Field to look for in .txt file
%                lsep:       Number of elements of sep
%                nf:         File identifier of configuration file
%                nveicoli:   Total number of vehicles
%                sep:        GUI seperator data
%                tmap:       Mapping of extracted info to numbers
%                trainfield: Name of field where the info will be stored
%
% OUTPUT         trainGUI:   Modified struct array

cline = trova_info(field,nf);

if not(isempty(cline))
    ind1 = numel(field) + 1; 
    ind2 = findstr(cline,sep);
    ldol = numel(dol);
    
    % Storing the names of the vehicles
    for ii = 1:nveicoli-1
        %trainGUI(ii) = setfield(trainGUI(ii),trainfield,cline(ind1:ind2(ii)-1));
        appo = cline(ind1:ind2(ii)-1);
        appo = str2num(appo(ldol+1:end));
        if appo == 0
            trainGUI     = setfield(trainGUI, {ii,1}, trainfield, {1}, 1);
            trainGUI(ii) = setfield(trainGUI(ii), trainfield, []);
        else
            appo     = tmap(appo);  % Mapping according to TrainDy symbols
            trainGUI = setfield(trainGUI, {ii,1}, trainfield, {1}, appo);
        end
        ind1 = ind2(ii)+lsep;
    end
    appo = cline(ind1:end);
    appo = str2num(appo(ldol+1:end));
    if appo == 0
        trainGUI           = setfield(trainGUI, {nveicoli,1}, trainfield, {1}, 1);
        trainGUI(nveicoli) = setfield(trainGUI(nveicoli), trainfield, []);
    else
        appo     = tmap(appo);      % Mapping according to TrainDy symbols
        trainGUI = setfield(trainGUI, {nveicoli,1},trainfield,{1},appo);
    end
else
    trainGUI     = setfield(trainGUI, {1,1}, trainfield, {1}, 1);
    trainGUI(1)  = setfield(trainGUI(1), trainfield, []);
end

end

function trainGUI = train_conf_fric(trainGUI,dol,field,lsep,nf,nveicoli,sep,tmap,trainfield)

% TRAIN_CONF_FRIC This function checks, for every vehicle, whether an 
%                 external block friction law has been defined by the user.
%
% INPUTS          trainGUI:   Struct array with info about each vehicle
%                 dol:        String including the required info for each
%                             vehicle
%                 field:      Field to look for in .txt file
%                 lsep:       Number of elements of sep
%                 nf:         File identifier of configuration file
%                 nveicoli:   Total number of vehicles
%                 sep:        GUI seperator data
%                 tmap:       Not used any more                             % Obsolete input, to be removed [s!]
%                 trainfield: Name of field where the info will be stored
%
% OUTPUT          trainGUI:   Updated struct array

cline = trova_info(field,nf);

if not(isempty(cline))
    ind1 = numel(field) + 1; 
    ind2 = strfind(cline,sep);
    ldol = numel(dol);
    
    % Storing the names of the vehicles
    for ii = 1:nveicoli-1
        %trainGUI(ii) = setfield(trainGUI(ii),trainfield,cline(ind1:ind2(ii)-1));
        appo = cline(ind1:ind2(ii)-1);
        if strcmp(appo,'CONST_FRICT_COEFF') || isempty(appo)
            trainGUI     = setfield(trainGUI, {ii,1}, trainfield, {1}, 1);
            trainGUI(ii) = setfield(trainGUI(ii), trainfield, '');
        else
            dummy = str2double(appo(ldol+2:end-1));
            if not(isnan(dummy))
                trainGUI = setfield(trainGUI, {ii,1}, trainfield, {1}, dummy);
            else
                trainGUI = setfield(trainGUI, {ii,1}, trainfield, {1:numel(appo)}, appo);
            end
        end
%         if isempty(appo) || appo == 0
%             trainGUI = setfield(trainGUI, {ii,1}, trainfield, {1}, 1);
%             %trainGUI(ii) = setfield(trainGUI(ii), trainfield, []);
%             trainGUI(ii) = setfield(trainGUI(ii), trainfield, '');
%         %elseif isempty(appo)
%         %    trainGUI(ii) = setfield(trainGUI(ii), trainfield, []); %setfield(trainGUI, {ii,1}, trainfield, {1}, -1);
%         else
%             appo = tmap(appo); % Mapping according to TrainDy symbols
%         end
        ind1 = ind2(ii)+lsep;
    end

    appo = cline(ind1:end);
    if strcmp(appo,'CONST_FRICT_COEFF');
        trainGUI = setfield(trainGUI, {nveicoli,1}, trainfield, {1}, 1);
        trainGUI(nveicoli) = setfield(trainGUI(nveicoli), trainfield, []);
    else
        % appo = str2double(appo(ldol+2:end-1));
        % trainGUI = setfield(trainGUI, {nveicoli,1},trainfield,{1},appo);
        dummy = str2double(appo(ldol+2:end-1));
        if not(isnan(dummy))
            trainGUI = setfield(trainGUI, {nveicoli,1}, trainfield, {1}, dummy);
        else
            trainGUI = setfield(trainGUI, {nveicoli,1}, trainfield, {1:numel(appo)}, appo);
        end
    end
%     if isempty(appo) || appo == 0
%         trainGUI = setfield(trainGUI, {nveicoli,1}, trainfield, {1}, 1);
%         trainGUI(nveicoli) = setfield(trainGUI(nveicoli), trainfield, []);
%     else
%         appo = tmap(appo); % Mapping according to TrainDy symbols
%     end
else
    trainGUI    = setfield(trainGUI, {1,1}, trainfield, {1}, 1);
    trainGUI(1) = setfield(trainGUI(1), trainfield, []);
end

end

% This function is no longer used and therefore it can be removed or
% commented [s!]
function trainGUI = copy_field(trainGUI,sourcefield,targetfield)
% This function copies the information in the sourcefield to the target field
for ii = 1:length(trainGUI)
    appo = getfield(trainGUI(ii),sourcefield);
    trainGUI = setfield(trainGUI, {ii,1}, targetfield, {1}, appo);
end
end


function trainGUI = fill_train_w_l(fsl,nloco,pathGUI,ploco,sep,trainGUI,petr)

% FILL_TRAIN_W_L This function extracts all the information about 
%                locomotives in the train and stores it to the struct array
%                trainGUI
%
% INPUTS         fsl:      '\' for Windows '/' for Linux
%                nloco:    Number of locomotives
%                pathGUI:  String with the path of Locomotive folder
%                ploco:    Vector with positions of locomotives
%                sep:      GUI seperator data
%                trainGUI: Struct array with info about each vehicle
%                petr:     Directory where to look for user defined
%                          friction laws (via fc_inp function)
%
% OUTPUT         trainGUI: Updated struct array

npath = [pathGUI fsl 'Locomotive' fsl];
lsep  = numel(sep);

for ii = 1:nloco
    trainGUI(ploco(ii)).type = 'loco';
    nfile = [npath trainGUI(ploco(ii)).name '.txt'];
    nf    = fopen(nfile,'r');
    strtofind = 'rotaryMasses='; cline = trova_info(strtofind,nf);
    trainGUI(ploco(ii)).prot = str2double(cline(numel(strtofind)+1:end))*1e-2;
    strtofind = 'numAxes='; cline = trova_info(strtofind,nf);
    trainGUI(ploco(ii)).na = str2double(cline(numel(strtofind)+1:end));
    if isnan(trainGUI(ploco(ii)).na), trainGUI(ploco(ii)).na = []; end
    strtofind = 'cplfhc='; cline = trova_info(strtofind,nf);
    trainGUI(ploco(ii)).K = str2double(cline(numel(strtofind)+1:end));
    if trainGUI(ploco(ii)).K == 0, trainGUI(ploco(ii)).K = []; end
    %if isempty(trainGUI(ploco(ii)).nbgf)
    %    strtofind = 'buffingGearsF='; cline = trova_info(strtofind,nf);
    %    trainGUI(ploco(ii)).nbgf = cline(numel(strtofind)+1:end);
    %end
    if isempty(trainGUI(ploco(ii)).lBP)
        strtofind = 'brakePipeWagonLen='; cline = trova_info(strtofind,nf);
        trainGUI(ploco(ii)).L = trainGUI(ploco(ii)).lwag*str2double(cline(numel(strtofind)+1:end));
    else
        trainGUI(ploco(ii)).L = trainGUI(ploco(ii)).lBP;
    end
    strtofind = 'brakePipeDiam='; cline = trova_info(strtofind,nf);
    trainGUI(ploco(ii)).D = 1e-3*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'driverBrakeValve='; cline = trova_info(strtofind,nf);
    trainGUI(ploco(ii)).DBV = cline(numel(strtofind)+1:end);
    
    trainGUI(ploco(ii)).bbtype = ''; trainGUI(ploco(ii)).dbtype = '';
    strtofind = 'chkBlockBrake='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true')
        if isempty(trainGUI(ploco(ii)).pbb)
            strtofind = 'bbContribution='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).pbb = 1e-2*str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'bbBringShoesNum='; cline = trova_info(strtofind,nf);
        trainGUI(ploco(ii)).nbs = str2double(cline(numel(strtofind)+1:end));
        strtofind = 'bbShoesType='; cline = trova_info(strtofind,nf);
        appo = '$SHOES_TYPE'; typeBB = str2num(cline(numel(strtofind)+numel(appo)+1:end));
        if typeBB == 1
            trainGUI(ploco(ii)).typeSh = 'Bg';
            trainGUI(ploco(ii)).sB = 25600; %Section of the shoe expressed in mm^2 (type bg)
        elseif typeBB == 2
            trainGUI(ploco(ii)).typeSh = 'Bgu';
            trainGUI(ploco(ii)).sB = 40000; %Section of the shoe expressed in mm^2 (type bgu)
        end
        strtofind = 'bbRadioSystem1='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true')
            trainGUI(ploco(ii)).bbtype = 'BLOCK_SI';
            strtofind = 'bbRigRatio='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).bbiG = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'bbCylSection='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).bbS = 1e-2 * str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'bbRadioSystem2='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(ploco(ii)).bbtype = 'BLOCK_BW';
            strtofind = 'bbBrWeight='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).bbbw = str2double(cline(numel(strtofind)+1:end));
        end
        if isempty(trainGUI(ploco(ii)).frlaw)
            strtofind = 'bbFrictLaw='; cline = trova_info(strtofind,nf);
            appo = '$EXT_BLOCK_FRICTION_LAWS'; bfl = str2num(cline(numel(strtofind)+numel(appo)+1:end));
        else
            bfl = trainGUI(ploco(ii)).frlaw;
        end
        if bfl == 0
            trainGUI(ploco(ii)).bfl = 2; % 'Karwatzki
        elseif bfl == 1
            trainGUI(ploco(ii)).bfl = 5; % 'BZA
        elseif bfl == 2
            trainGUI(ploco(ii)).bfl = 4; % 'OSS
        else
            % The User defined a friction coefficient by textfile
            %strtofind = 'bbFrictLaw='; cline = trova_info(strtofind,nf);
            %bfl = cline(numel(strtofind)+1:end);
            trainGUI(ploco(ii)).mbfc = bfl; % The name of the model of block friction coefficient is stored
            [fcv,fcp,fcf,Pfc]        = fc_inp(fsl,petr,bfl);
            trainGUI(ploco(ii)).Pfc  = Pfc;
            trainGUI(ploco(ii)).fcv  = fcv;
            trainGUI(ploco(ii)).fcp  = fcp;
            trainGUI(ploco(ii)).fcf  = fcf;
            trainGUI(ploco(ii)).bfl  = 99;
        end
        strtofind = 'bbRigEff='; cline = trova_info(strtofind,nf);
        trainGUI(ploco(ii)).rendtim = str2double(cline(numel(strtofind)+1:end));
        strtofind = 'bbFF='; cline = trova_info(strtofind,nf);
        trainGUI(ploco(ii)).Ff = str2double(cline(numel(strtofind)+1:end));
        strtofind = 'bbFR='; cline = trova_info(strtofind,nf);
        trainGUI(ploco(ii)).Fr = str2double(cline(numel(strtofind)+1:end));
    else
        trainGUI(ploco(ii)).bblp = [];
        trainGUI(ploco(ii)).pbb  = 1;
    end
    strtofind = 'chkDiskBrake='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true');
        strtofind = 'dbRadioSystem1='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(ploco(ii)).dbtype = 'DISK_SI';
            strtofind = 'dbCylSection='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbS = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbFRigRatio='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbfr = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbSRigRatio='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbsr = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbFEffic='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbfe = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbSEffic='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbse = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbCountForce='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbcf = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbDBRadius='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbdr = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbWRadius='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbwr = str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'dbRadioSystem2='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(ploco(ii)).dbtype = 'DISK_BW';
            strtofind = 'dbBrWeight='; cline = trova_info(strtofind,nf);
            trainGUI(ploco(ii)).dbbw = str2double(cline(numel(strtofind)+1:end));
            trainGUI(ploco(ii)).dbdr = [];
        end
        trainGUI = disk_FL(ii,nf,petr,ploco,trainGUI);
    else
        trainGUI(ploco(ii)).dblp = [];
        trainGUI(ploco(ii)).dfl  = [];
        trainGUI(ploco(ii)).pdb  = 1;
    end
    
    strtofind = 'chkElettrodBrake='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true')
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'ebSpeed=','elbcv',1);
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'ebBrForce=','elbcv',2);
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'ebTime=','elbct',1);
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'ebPercMaxF=','elbct',2);

        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'etSpeed=','eltcv',1);
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'etBrForce=','eltcv',2);
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'etTime=','eltct',1);
        trainGUI = mat_train(ii,lsep,nf,ploco,trainGUI,sep,'etPercMaxF=','eltct',2);
    else
        trainGUI(ploco(ii)).elbcv = [0 0]; trainGUI(ploco(ii)).elbct = [0 0];
        trainGUI(ploco(ii)).eltcv = [0 0]; trainGUI(ploco(ii)).eltct = [0 0];
    end
    
    strtofind = 'etLocoTIG='; cline = trova_info(strtofind,nf);
    if not(isempty(cline)), trainGUI(ploco(ii)).tig = 1e3*str2double(cline(numel(strtofind)+1:end)); else trainGUI(ploco(ii)).tig = 0; end
    strtofind = 'etLocoTRG='; cline = trova_info(strtofind,nf);
    if not(isempty(cline)), trainGUI(ploco(ii)).trg = 1e3*str2double(cline(numel(strtofind)+1:end)); else trainGUI(ploco(ii)).trg = 0; end

    fclose(nf);
    
    
end

end

function trainGUI = fill_train_w_w(fsl,nwago,pathGUI,pwago,sep,trainGUI,petr)

% FILL_TRAIN_W_W This function extracts all the information about wagons
%                in the train and stores it to the struct array trainGUI
%                
% INPUTS         fsl:      '\' for Windows '/' for Linux
%                nwago:    Number of wagons
%                pathGUI:  String with the path of Wagon folder
%                pwago:    Vector with positions of wagons
%                sep:      GUI seperator data
%                trainGUI: Struct array with info about each vehicle
%                petr:     Directory where to look for user defined
%                          friction laws (via fc_inp function)
%
% OUTPUT         trainGUI: Updated struct array

npath = [pathGUI fsl 'Wagon' fsl];
lsep  = numel(sep);

for ii = 1:nwago
    trainGUI(pwago(ii)).type = 'wago';
    nfile = [npath trainGUI(pwago(ii)).name '.txt'];
    nf = fopen(nfile,'r');
    if isempty(trainGUI(pwago(ii)).tare)
        strtofind = 'tare='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).tare = str2double(cline(numel(strtofind)+1:end));
    end
    if isempty(trainGUI(pwago(ii)).lwag)
        strtofind = 'length='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).lwag = str2double(cline(numel(strtofind)+1:end));
    end
    strtofind = 'rotaryMasses='; cline = trova_info(strtofind,nf);
    trainGUI(pwago(ii)).prot = str2double(cline(numel(strtofind)+1:end))*1e-2;
    strtofind = 'axesNum='; cline = trova_info(strtofind,nf);
    trainGUI(pwago(ii)).na = str2double(cline(numel(strtofind)+1:end));
    if isnan(trainGUI(pwago(ii)).na), trainGUI(pwago(ii)).na = []; end

    if isempty(trainGUI(pwago(ii)).nbgf)
        strtofind = 'buffingGearsF='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).nbgf = cline(numel(strtofind)+1:end);
    end
    if isempty(trainGUI(pwago(ii)).ndgf)
        strtofind = 'drawGearsF='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).ndgf = cline(numel(strtofind)+1:end);
    end
    if isempty(trainGUI(pwago(ii)).nbgr)
        strtofind = 'buffingGearsR='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).nbgr = cline(numel(strtofind)+1:end);
    end
    if isempty(trainGUI(pwago(ii)).ndgr)
        strtofind = 'drawGearsR='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).ndgr = cline(numel(strtofind)+1:end);
    end
    if isempty(trainGUI(pwago(ii)).lBP)
        strtofind = 'brakePipeWagonLen='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).L = trainGUI(pwago(ii)).lwag*str2double(cline(numel(strtofind)+1:end));
    else
        trainGUI(pwago(ii)).L = trainGUI(pwago(ii)).lBP;
    end
    strtofind = 'brakePipeDiam='; cline = trova_info(strtofind,nf);
    trainGUI(pwago(ii)).D = 1e-3*str2double(cline(numel(strtofind)+1:end));
    if isempty(trainGUI(pwago(ii)).CV)
        strtofind = 'controlValve='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).CV = cline(numel(strtofind)+1:end);
    end
    trainGUI(pwago(ii)).bbtype = ''; trainGUI(pwago(ii)).dbtype = '';
    strtofind = 'chkBlockBrake='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true');
        if isempty(trainGUI(pwago(ii)).pbb)
            strtofind = 'bbContribution='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).pbb = 1e-2*str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'bbShoesNum='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).nbs = str2double(cline(numel(strtofind)+1:end));
        strtofind = 'bbShoesType='; cline = trova_info(strtofind,nf);
        appo = '$SHOES_TYPE'; typeBB = str2num(cline(numel(strtofind)+numel(appo)+1:end));
        if typeBB == 1
            trainGUI(pwago(ii)).typeSh = 'Bg';
            trainGUI(pwago(ii)).sB = 25600; %Section of the shoe expressed in mm^2 (type bg)
        elseif typeBB == 2
            trainGUI(pwago(ii)).typeSh = 'Bgu';
            trainGUI(pwago(ii)).sB = 40000; %Section of the shoe expressed in mm^2 (type bgu)
        end
        strtofind = 'bbRadioSystem1='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true')
            trainGUI(pwago(ii)).bbtype = 'BLOCK_SI';
            strtofind = 'bbRigRatio='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbiG = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'bbCylSection='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbS = 1e-2 * str2double(cline(numel(strtofind)+1:end));
            strtofind = 'bbInvMass='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbbwi = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'bbEmptyPress='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbep = str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'bbRadioSystem2='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(pwago(ii)).bbtype = 'BLOCK_BW_EL';
            strtofind = 'bbBrWeightLoad='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbbwl = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'bbChgWeight='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbbwi = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'bbBrWeightEmpty='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).bbbwe = str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'bbRadioSystem3='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(pwago(ii)).bbtype = 'BLOCK_BW_AC';
            trainGUI = mat_train(ii,lsep,nf,pwago,trainGUI,sep,'bbTotMass=','bbauto',1);
            trainGUI = mat_train(ii,lsep,nf,pwago,trainGUI,sep,'bbMassBraked=','bbauto',2);
        end
        if isempty(trainGUI(pwago(ii)).frlaw)
            strtofind = 'bbFrictLaw='; cline = trova_info(strtofind,nf);
            appo = '$EXT_BLOCK_FRICTION_LAWS'; bfl = str2num(cline(numel(strtofind)+numel(appo)+1:end));
        else
            bfl = trainGUI(pwago(ii)).frlaw;
        end
        if bfl == 0
            trainGUI(pwago(ii)).bfl = 2; % 'Karwatzki
        elseif bfl == 1
            trainGUI(pwago(ii)).bfl = 5; % 'BZA
        elseif bfl == 2
            trainGUI(pwago(ii)).bfl = 4; % 'OSS
        else
            
            % The User defined a friction coefficient by textfile
            %strtofind = 'bbFrictLaw='; cline = trova_info(strtofind,nf);
            %bfl = cline(numel(strtofind)+1:end);
            trainGUI(pwago(ii)).mbfc = bfl; % The name of the model of block friction coefficient is stored
             [fcv,fcp,fcf,Pfc] = fc_inp(fsl,petr,bfl);
             trainGUI(pwago(ii)).Pfc = Pfc;
             trainGUI(pwago(ii)).fcv = fcv;
             trainGUI(pwago(ii)).fcp = fcp;
             trainGUI(pwago(ii)).fcf = fcf;
             trainGUI(pwago(ii)).bfl = 99;
        end
        strtofind = 'bbRigEff='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).rendtim = str2double(cline(numel(strtofind)+1:end));
        strtofind = 'bbFF='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).Ff = str2double(cline(numel(strtofind)+1:end));
        strtofind = 'bbFR='; cline = trova_info(strtofind,nf);
        trainGUI(pwago(ii)).Fr = str2double(cline(numel(strtofind)+1:end));
    else
        trainGUI(pwago(ii)).bblp = [];
        trainGUI(pwago(ii)).pbb = 1;
    end

    strtofind = 'chkDiskBrake='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true');
        if isempty(trainGUI(pwago(ii)).pdb)
            strtofind = 'dbContribution='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).pdb = 1e-2*str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'dbRadioSystem1='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(pwago(ii)).dbtype = 'DISK_SI';
            strtofind = 'dbCylSection='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbS = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbInvMass='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbbwi = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbEmptyPress='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbep = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbFRigRatio='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbfr = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbSRigRatio='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbsr = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbFEffic='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbfe = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbSEffic='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbse = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbCountForce='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbcf = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbDBRadius='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbdr = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbWRadius='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbwr = str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'dbRadioSystem2='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(pwago(ii)).dbtype = 'DISK_BW_EL';
            strtofind = 'dbBrWeightLoad='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbbwl = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbChgWeight='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbbwi = str2double(cline(numel(strtofind)+1:end));
            strtofind = 'dbBrWeightEmpty='; cline = trova_info(strtofind,nf);
            trainGUI(pwago(ii)).dbbwe = str2double(cline(numel(strtofind)+1:end));
        end
        strtofind = 'dbRadioSystem3='; cline = trova_info(strtofind,nf);
        if strcmp(cline(numel(strtofind)+1:end),'true');
            trainGUI(pwago(ii)).dbtype = 'DISK_BW_AC';
            trainGUI = mat_train(ii,lsep,nf,pwago,trainGUI,sep,'dbTotMass=','dbauto',1);
            trainGUI = mat_train(ii,lsep,nf,pwago,trainGUI,sep,'dbMassBraked=','dbauto',2);
            trainGUI(pwago(ii)).dbdr = [];
        end
        trainGUI = disk_FL(ii,nf,petr,pwago,trainGUI);
    else
        trainGUI(pwago(ii)).dblp = [];
        trainGUI(pwago(ii)).dfl  = [];
        trainGUI(pwago(ii)).pdb  = 1;
    end
    
    fclose(nf);
end

end

function trainGUI = fill_train_w_bg_dg(fsl,nveicoli,pathGUI,sep,trainGUI)

% FILL_TRAIN_W_BG_DG This function extracts all the information about 
%                    buffers and draw gears of every vehicle and stores it
%                    to the struct array trainGUI
%                
% INPUTS             fsl:      '\' for Windows '/' for Linux
%                    nveicoli: Total number of vehicles
%                    pathGUI:  String with the path of BuffersDrawGears 
%                              folder
%                    sep:      GUI seperator data
%                    trainGUI: Struct array with info about each vehicle
%
% OUTPUT             trainGUI: Updated struct array

npath = [pathGUI fsl 'BuffersDrawGears' fsl];
lsep  = numel(sep);

for ii = 1:nveicoli
    nfile = [npath trainGUI(ii).nbgf '.txt'];
    nf    = fopen(nfile,'r');
    [CentrC,damp,force,stroke,vd,vpl,vpu] = extrbgdg(lsep,nf,sep);
    fclose(nf);
    trainGUI(ii).bgdf   = damp; trainGUI(ii).bgff  = force; trainGUI(ii).bgsf = stroke;
    trainGUI(ii).bgvplf = vpl; trainGUI(ii).bgvpuf = vpu; trainGUI(ii).bgccf  = CentrC; trainGUI(ii).bgvdf = vd;

    nfile = [npath trainGUI(ii).nbgr '.txt'];
    nf    = fopen(nfile,'r');
    [CentrC,damp,force,stroke,vd,vpl,vpu] = extrbgdg(lsep,nf,sep);
    fclose(nf);
    trainGUI(ii).bgdr   = damp; trainGUI(ii).bgfr  = force; trainGUI(ii).bgsr = stroke;
    trainGUI(ii).bgvplr = vpl; trainGUI(ii).bgvpur = vpu; trainGUI(ii).bgccr  = CentrC; trainGUI(ii).bgvdr = vd;


    nfile = [npath trainGUI(ii).ndgf '.txt'];
    nf = fopen(nfile,'r');
    [CentrC,damp,force,stroke,vd,vpl,vpu] = extrbgdg(lsep,nf,sep);
    fclose(nf);
    trainGUI(ii).dgdf = damp; trainGUI(ii).dgff = force; trainGUI(ii).dgsf = stroke;
    trainGUI(ii).dgvplf = vpl; trainGUI(ii).dgvpuf = vpu; trainGUI(ii).dgccf = CentrC; trainGUI(ii).dgvdf = vd;

    nfile = [npath trainGUI(ii).ndgr '.txt'];
    nf = fopen(nfile,'r');
    [CentrC,damp,force,stroke,vd,vpl,vpu] = extrbgdg(lsep,nf,sep);
    fclose(nf);
    trainGUI(ii).dgdr = damp; trainGUI(ii).dgfr = force; trainGUI(ii).dgsr = stroke;
    trainGUI(ii).dgvplr = vpl; trainGUI(ii).dgvpur = vpu; trainGUI(ii).dgccr = CentrC; trainGUI(ii).dgvdr = vd;
    
    % Order of the polynomial approximation: in Etrain is 1, in TrainDy is set to 3
    trainGUI(ii).op = 3;
end

end

function [CentrC,damp,force,stroke,vd,vpl,vpu] = extrbgdg(lsep,nf,sep)

% EXTRBGDG This function is called multiple times from fill_train_w_bg_dg 
%          to extract the data of each coupling gear 
%
% INPUTS   lsep:   Number of elements of sep
%          nf:     File identifier of coupling gear file
%          sep:    GUI seperator data
%
% OUTPUTS  CentrC: Central coupler     (0 if not given)
%          damp:   Damping coefficient (0 if not given)
%          force:  Array with force values [kN]. If damp = 0, first column
%                  for unload, second for load. If damp ~= 0, first column
%                  for load, second filled with zeros
%          stroke: Column vector with stroke values [mm] 
%          vd:     Viscous damping coefficient (0 if not given)
%          vpl:    Load   limiting velocity
%          vpu:    Unload limiting velocity

% Initializations
strtofind = 'loadLimVel=';   cline = trova_info(strtofind,nf);
vpl       = str2double(cline(numel(strtofind)+1:end));
strtofind = 'unloadLimVel='; cline = trova_info(strtofind,nf);
vpu       = str2double(cline(numel(strtofind)+1:end));
strtofind = 'chkDampCoeff='; cline = trova_info(strtofind,nf);

if strcmp(cline(numel(strtofind)+1:end),'true');
    strtofind = 'dampCoeff='; cline = trova_info(strtofind,nf);
    damp = 1e-2*str2double(cline(numel(strtofind)+1:end));
else
    damp = 0;
end
CentrC = 0;
strtofind = 'chkCentrCoupler='; cline = trova_info(strtofind,nf);
if not(isempty(cline)), CentrC = str2num(cline(numel(strtofind)+1:end)); end
vd = 0;
strtofind = 'viscDampCoeff='; cline = trova_info(strtofind,nf);
if not(isempty(cline)), vd = str2double(cline(numel(strtofind)+1:end)); end

stroke = mat_var(lsep,nf,sep,'stroke=');
n = numel(stroke);
if damp == 0
    force(1:n,2) = mat_var(lsep,nf,sep,'load=');
    force(1:n,1) = mat_var(lsep,nf,sep,'unload=');
else
    force(1:n,1) = mat_var(lsep,nf,sep,'load=');
    force(1:n,2) = 0;
end
end

function traccia = tracciato(fsl,ntrack,pathGUI,sep)

% TRACCIA This function extracts info about the track and stores it to
%         array traccia
%
% INPUTS  fsl:     '\' for Windows '/' for Linux
%         ntrack:  Name of track info file
%         pathGUI: String with the path of Track folder
%         sep:     GUI seperator data 
%
%  OUTPUT traccia: Array with track info

lsep  = numel(sep);
nfile = [pathGUI,fsl,'Track',fsl,ntrack(1:end-3) 'txt'];
nf    = fopen(nfile,'r');
% strtofind = 'secType='; cline = trova_info(strtofind,nf);
% appo = '$SEC_TYPE'; sec_type = str2num(cline(numel(strtofind)+numel(appo)+1:end));

type   = mat_var_dol(lsep,nf,sep,'secType=','$SEC_TYPE');
length = mat_var(lsep,nf,sep,'length=');
R      = mat_var(lsep,nf,sep,'curvRad=');
fi     = mat_var(lsep,nf,sep,'slope=');
teta   = mat_var(lsep,nf,sep,'elev=');
lpar   = mat_var(lsep,nf,sep,'parabLen=');
fclose(nf);

% Info in trackGUI:
% Type - Length [m]	- Curvature radius [m] - Slope [per mil] - Elevation [cm] - length of previous parabolic curve [m]

% Info in traccia:
% Length - Curvature radius - Slope - Elevation	- Vertical radius(computed) - Type

traccia = [length(1) R(1) fi(1) teta(1) 0 type(1)]; 
c       = 1;

for ii = 2:size(type,1)
    if lpar(ii) > 0
        c = c+1;
        traccia(c,:) = [lpar(ii) 0 0 0 0 3];
        c = c+1;
        traccia(c,:) = [length(ii) R(ii) fi(ii) teta(ii) 0 type(ii)];
    else
        c = c+1;
        traccia(c,:) = [length(ii) R(ii) fi(ii) teta(ii) 0 type(ii)];
    end
end

end

function trainGUI = disk_FL(ii,nf,petr,pvehi,trainGUI)

% DISK_FL  This function sets the .dfl field of the vehicle according to
%          the info written in the corresponding txt file. dfl is either a
%          number for constant friction coefficient or a matrix for a 
%          defined friction law  
%
% INPUTS   ii:       Counter for wagons or locomotives
%          nf:       File identifier of current vehicle
%          petr:     Directory where the folder with the Disk friction laws
%                    is
%          pvehi:    Matrix that contains the positions of wagons or
%                    locomotives, used to map ii to the correct element in 
%                    trainGUI struct array
%          trainGUI: Struct array with info about each vehicle
%
% OUTPUT   trainGUI: Updated struct array

% New function [f]

strtofind = 'dbRadioFC='; 
cline     = trova_info(strtofind,nf);

if strcmp(cline(numel(strtofind)+1:end),'true');
    strtofind = 'dbFrictCoeff='; cline = trova_info(strtofind,nf);
    trainGUI(pvehi(ii)).dfl = str2double(cline(numel(strtofind)+1:end));
else
    strtofind = 'dbFrictLaw='; cline = trova_info(strtofind,nf);
    dfl       = cline(numel(strtofind)+1:end);
    if isempty(dfl)
        error('Disk friction law or value is not defined');
    else
        dfl = extr_dfl(dfl,petr(1:end-1));
        trainGUI(pvehi(ii)).dfl = dfl;
    end
end

end

%SNCF MODIFICATION 
%Add of fc_inp function (from translate_input.m) in GUI_input.m
function [fcv,fcp,fcf,Pfc] = fc_inp(fsl,petr,bfl)

% FC_INP  This function reads the table in the blockfrictionlaw.txt file 
%         and builds a piecewise polynomial approximation of the friction
%         coefficient curve versus velocity, for every value of specific 
%         pressure
%
% INPUTS  fsl:  '\' for Windows '/' for Linux
%         petr: Directory where to look for the .txt file
%         bfl:  name of the .txt file
%
% OUTPTUS fcv:  Discrete velocity values defined by user [m/s]
%         fcp:  Discrete pressure values defined by user [Kg/cm^2]
%         fcf:  Matrix with friction coefficient values for every
%               combination of speed and specific pressure
%         Pfc:  Matrix with polynomial coefficients. Every column has the
%               piecewise polynomial approximation for the corresponding
%               pressure. The first element of each column indicates the 
%               number of polynomials for the used for the approximation

% Path of the file bfl.txt
nBfl  = [petr,'BlockFrictionLaws',fsl,bfl,'.txt'];

% Opening of the file bfl.txt
nfBfl = fopen(nBfl,'r');

if nfBfl == -1
    error('The requested block brake friction coefficient model has not been provided yet');
end

% Reading of the table of the bfl
j     = 1;
tline = fgetl(nfBfl);
while ischar(tline)
    [start,endLine] = strtok(tline,'=');
    lineS     = endLine(2:end);
    line(j,:) = sscanf(lineS,['%f' '_;_']);
    tline     = fgetl(nfBfl);
    j         = j + 1;
end

% Pfc has the number of rows equal to the number of specific pressures, and the
% number of columns that depend on the number of points used to describe each line

Pfc  = zeros(1,1);          % Initialization
A    = line';
sizA = size(A,1)-1;
vv   = A(1,2:end)/3.6;      % Speeds [m/s]
fcv  = vv;
fcp  = A(2:end,1);          % Specific pressures
fcf  = A(2:end,2:end);      % Values of friction coefficient
np   = sizA;                % Number of different lines

% Fitting is made using a cubic spline interpolation: the array Pfc will be filled
 for ii = 1:np
     P = comp_poly_fc(fcv,fcf(ii,:));
     Pfc(1:length(P),ii) = P;
 end

fclose(nfBfl); 
 
end
%END SNCF MODIFICATION

function dfl = extr_dfl(dfl,petr)

% EXTR_DFL This function reads the disk friction law provided in the file 
%          "dfl.txt" and computes its piecewise polynomial
% 
% INPUTS   dfl:  Name of disk friction law
%          petr: Directory where the folder with the Disk friction laws is
%    
% OUTPUTS  dfl:  Matrix with the piecewise polynomial based on the given
%                law

% New function [f]

sep = '_;_'; lsep = numel(sep);% GUI Separator data
nf = fopen([petr '\DiskFrictionLaws\' dfl '.txt'],'r');
strtofind = 'vrmrr='; cline = trova_info(strtofind,nf);
findsep = strfind(cline,sep);
v = zeros(1,numel(findsep)+1);
v(1) = str2double(cline(numel(strtofind)+1:findsep(1)-1));
for ii = 2:numel(findsep)
   v(ii) =  str2double(cline(findsep(ii-1)+lsep:findsep(ii)-1));
end
v(end) = str2double(cline(findsep(end)+lsep:end));

strtofind = 'frictCoeff='; cline = trova_info(strtofind,nf);
findsep = strfind(cline,sep);
mu = zeros(1,numel(findsep)+1);
mu(1) = str2double(cline(numel(strtofind)+1:findsep(1)-1));
for ii = 2:numel(findsep)
   mu(ii) =  str2double(cline(findsep(ii-1)+lsep:findsep(ii)-1));
end
mu(end) = str2double(cline(findsep(end)+lsep:end));

fclose(nf);

% v = A(:,1)'/3.6; mu = A(:,2)'; % Speed [m/s] and friction coefficient
delta         = 0.1;                    % Minimum distance among speeds in characteristic description
[P,newx,newy] = poly_trac(delta,v,mu);
velo          = zeros(1,length(P));
velo(1:4:end) = newx;
dfl           = [100 P;1 velo];

end