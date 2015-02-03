function [CA,CVactv,D1,DBVdata,dLC,dTF,dx,gmis,gmisCF,...
          Lcoll,Lvago,nveicoli,P1,pCF,SA,trainGUI,Tsim, ...
          Vbc] = PneumDevices(fsl,Lcoll,P1,pathGUI,sep,trainGUI)            % [f] removed several outputs
                                                                            
% PNEUMDEVICES This function reads in the input of pneumatic devices
%
% INPUTS       fsl:        '\' for Windows '/' for Linux
%              Lcoll:      Length of hose couplings [m]
%              P1:         Initial absolute Brake Pipe pressure [Pa]
%              pathGUI:    Directory containing all the folders with input
%                          data
%              sep:        GUI seperator data
%              trainGUI:   Struct array with info about each vehicle
%
% OUTPUTS      CA:         Array with Acceleration Chamber data
%              CVactv:     Vector indicating which Control Valves are
%                          active (Not with 0-1, by indexing)
%              D1:         Vector with internal diameter of Brake Pipes [m]
%              DBVdata:    Array with Driver's Brake Valve data
%              dLC:        Limiting curve data of Control Valves. Explained
%                          below
%              dTF:        Array with transfer function data of Control 
%                          Valves. Explained below
%              dx:         Spatial discretization
%              gmis:       Discretized position of vehicle centers along  
%                          the Brake Pipe, taking also the hose couplings 
%                          into account [m]
%              gmisCF:     Discretized position of Brake Cylinders along  
%                          the Brake Pipe [m]
%              Lcoll:      Modified length of hose couplings (floor) [m]
%              Lvago:      Vector with Brake Pipe lengths [m]
%              nveicoli:   Number of vehicles                              
%              P1:         Initial absolute Brake Pipe pressure             [s!] Why is P1 an output since it does not change within this function ?
%              pCF:        Vector with pressure in Brake Cylinders. All 
%                          elements set to 0 since it is assumed that the 
%                          train starts in "on run" condition
%              SA:         Matrix array with Auxiliary Reservoir data
%              trainGUI:   Updated struct array
%              Tsim:       Simulation time (set to zero, taken from 
%                          operation data[?])
%              Vbc:        Volume of Brake Cylinders for block braked
%                          vehicles, 0 otherwise [m^3]
      
global lam s  

% CONSTANTS OF PNEUMATIC MODEL

% Assignment of data for calculation of thermal flux through pipe
s   = 3e-3; % Thickness of pipe
lam = 14;   % Thermal conductivity of pipe (S.I.) [W/(m*K)]

dx  = 1; 	% spatial discretization [m]

% tvis = 1; % Video display time

nveicoli = length(trainGUI); % Number of vehicles
% VhcM = 1:nveicoli; % Vehicles where pneumatic information is available.

% From trainGUI some variables are extracted. Variables not stored before 
% are stored now.
znv = zeros(1,nveicoli); Lvago = znv; D1 = znv; pCFmx = znv; pCFex = znv;
nCV = 0; nDBV = 0;
% DPbUPbc = znv; PbUPbc = znv;

% % % % % % % % % % % % % % Limiting Curve data % % % % % % % % % % % % % %
%                                                                         %
% dLC(:,[:]) : Index of vehicle                                           %
%                                                                         %
% dLC([:],:) : 1 Pactv  Pinit - min pressure drop in BP to activate       %
%                       application stroke phase. Therefore it is the     %
%                       pressure in BP below which AS begins [bar]        %
%              2 pCFAs  Pressure in BC during Application Stroke [bar]    %
%              3 tAS    Minimum time of Application Stroke phase [s]      %
%              4 pCGAs  Pressure in BP below which AS is over [bar]       %
%              5 pCFIf  Pressure in BC at the end of Inshot Function [bar]%
%              6 tIf    Duration of Inshot Function [s]                   %
%              7:14     Coefficients of parabolic curve for braking;      %       
%                       Maximum time of parabolic curve;                  %         
%                       Coefficients of parabolic curve for releasing;    %
%                       Maximum time of parabolic curve;                  %
%                       (With the brake regime of the vehicle taken into  %
%                       acount. Moreover, since the polynomial is of 2nd  %
%                       order, the cofficients are always 3)              %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % [SBB] % % % % %

dLC = zeros(14,1); 

% dTF([:],:,:) : Number of vehicle
% dTF(:,[:],:) : series of points to build transfer function before braking 
%                and after releasing data points
% dTF(:,:,[:]) : pCG pCF  starting from maximum pressure drop

% % % % % % % % % % % % % Transfer function data  % % % % % % % % % % % % %
%                                                                         %
% dTF([:],:,:) : Index of vehicle                                         %
% dTF(:,[:],:) : Along each row, pressure values on the given transfer    %
%                function points are stuck together. First for braking    %
%                and then for releasing. Last element represents the      %
%                number of points (In dTF(:,:,1) for the braking TF and   %
%                in dTF(:,:,2) for the releasing TF)                      %
% dTF(:,:,[:]) : First dimension for the Brake Pipe pressures and second  %
%                for the Brake Cylinder pressures [bar]                   %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % [SBB] (alternative explanation) % %

dTF = zeros(1,2,2);

% DBVd is a matrix that collects DBV data:
% (1,:) : emergency brake data EB
% (2,:) : service brake data SB
% (3,:) : releasing data R
% (4,:) : EP data

% % % % % % % % % % % % Driver's Brake Valve Data % % % % % % % % % % % % %
%                                                                         %
% DBVd(3*nDBV,4) For each DBV, 3 new lines are added. Structure of the    %
%                first 3 lines:                                           %
%                                                                         %
% DBVd(1,1:2):   Emergency Brake data                                     %  
%                -Equivalent diameter of orifice for emergency brake [m]  %
%                -Flow coefficient of orifice Cq                          %
% DBVd(1,3):     Discretized position of DBV along the Brake Pipe [m]     %
% DBVd(2,1:4):   Service Brake data                                       %
%                -Equivalent diameter of orifice for service brake [m]    %
%                -Flow coefficient of orifice Cq                          %
%                -Time to achieve a drop of 1.5 [bar] in counter-pressure %
%                 shape (from 5 to 3.5 [bar]) [s]                         %
%                -Time of first decreasing in counter-pressure shape      %
%                 (from 5 to 4.5 [bar]) [s]                               % 
% DBVd(3,1:3):   Releasing data                                           %
%                -Equivalent diameter of orifice for releasing [m]        %
%                -Flow coefficient of orifice Cq                          %
%                -Time to achieve an increasing of 1.5 [bar] in counter-  %
%                 pressure shape (from 3.5 to 5 [bar]) [s]                %
% DBVd(4,:):     Electro-Pneumatic data. Not implemented yet [n]          %
%                                                                         %
%                NOTE: Actually in GUI, DBVdata is the same as DBVd [?]   % [s!] maybe no need for two variables any more
%                                                                         %
% % % % % % % % % % % % % % % % % % % % [SBB] (alternative explanation) % %

sknowndbvs = 0; DBVd = zeros(3,10);
sknowncvs  = 0; CV   = cell(2,1);
DBVname    = cell(1);
CVactv     = znv;

for ii = 1:nveicoli
    Lvago(ii) = trainGUI(ii).L;
    D1(ii)    = trainGUI(ii).D;
    pCFmx(ii) = trainGUI(ii).pBC;
    pCFex(ii) = trainGUI(ii).pBCexp;
    
    if trainGUI(ii).CVactv == 1
        nCV = nCV + 1; %tt = nCV;  
        CVactv(ii) = ii;
        [CV,dLC,dTF,sknowncvs,trainGUI] = readbyfileCV(CV,dLC,dTF,fsl,ii, ...
            P1,pathGUI,pCFex,sep,sknowncvs,trainGUI,ii); % Modified 13/09/2010
    end
    if strcmp(trainGUI(ii).type,'loco')
        nDBV = nDBV + 1;
        [DBVd,DBVname,sknowndbvs,trainGUI] = readbyfileDBV(DBVd,DBVname, ...
            fsl,ii,nDBV,pathGUI,sknowndbvs,trainGUI);
       
    end
end

CVactv(CVactv==0) = [];

[gmis,Lcoll,Lvago,LvagoC] = ridefLvago(dx,Lcoll,Lvago,nveicoli);

% TODO: CHECK CODE BEHAVIOUR WHEN THERE ARE SOME CVs DISABLED: ACTUALLY IT 
% IS ASSUMED EVERY CV WORKS.
% Progressive position of Control Valve
% Transfer function vehicle number-progressive position along train
% Lprg = [0 cumsum(LvagoC)];

posDBV = zeros(1,nDBV); c = 0;
for ii = 1:nveicoli
    if strcmp(trainGUI(ii).type,'loco');
        c = c + 1;
        posDBV(c) = ii;
    end
end

% DBVdata collects all data of DBV for the simulation starting from first:
% for each DBV you have 4 lines: 1° emergency braking data, position, delay 
%                                2° service braking data
%                                3° releasing data 
%                                4° electropneumatic data  [n]

DBVdata = zeros(3*nDBV,10);
k       = 0; % index to move inside the matrix  DBVdata
Lprg    = [0 cumsum(LvagoC)];

for ii = 1:nDBV
    % Progressive position of DBV (redefine position)
    % Transfer function vehicle number-progressive position along train
    if posDBV(ii) >= 1
        DBVd(3*(ii-1)+1,3) = Lprg(posDBV(ii))+0.5*Lcoll+Lvago(posDBV(ii))./4; 
    elseif posDBV(ii) == 1
        DBVd(3*(ii-1)+1,3) = Lvago(posDBV(ii))./4;                            % [b] it should be posDBV(ii) > 1 before so that we enter this condition for pos == 1! Results in wrong position when loco is at the beginning of train
    end
  
    % Assignement matrix DBV in simulation
    % With the GUI input there are as many DBVs as the locomotives
    DBVdata(3*k+1:3*k+3,:) = DBVd(3*(ii-1)+1:3*(ii-1)+3,:);
    k = k+1;
end

Actv    = 1:nveicoli;
CV_post = zeros(size(Actv)); % TODO: add in the GUI input this feature.      % [n] Installation of CV away from the wagon center not implemented yet

if abs(CV_post(Actv)) < Lvago(Actv)*0.5
    gmisCV = Lprg(Actv)+0.5*Lcoll+0.5*Lvago(Actv)+CV_post(Actv);
    gmisCV(Actv == 1) = 0.5*Lvago(1)+CV_post(1);
    gmisCF = gmisCV;
else
    error('CV_post too long, it must be shorter than half the length of the wagon');
end

% Storing data in matrices used for the computation
% Data Acceleration Chamber
% matrix CA: position in m , volume , diameter , P activation , P initial ,
%            P closing 

% % % % % % % % % % % % % Acceleration Chamber Data % % % % % % % % % % % % 
%                                                                         %
% CA(:,[:]):  Index of vehicle                                            %
%                                                                         %
% CA([1],:):  Discretized position of Acceleration Chamber along the      %
%             Brake Pipe [m]. LATER will be transformed to pipe section   %
% CA([2],:):  Equivalent volume of Acceleration Chamber [m^3]             %
% CA([3],:):  Equivalent nozzle diameter of Acceleration Chamber [m]      %
%             LATER will be transformed to cross section area    [m^2]    %
% CA([4],:):  Absolute activation pressure of Acceleration Chamber. When  %
%             Brake Pipe pressure is below this value, CA is active [Pa]  %
% CA([5],:):  Initial absolute pressure in Acceleration Chambers. Set to  %
%             be the atmospheric pressure assuming train starts in "on    %
%             run" condition [Pa]                                         % 
% CA([6],:):  Minimum absolute pressure closing of Acceleration Chamber   %
%             (GUI definition [?]) [Pa]                                   %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % [SBB] (alternative explanation) % %


% % % % % % % % % % % % % Auxiliary Reservoir Data  % % % % % % % % % % % % 
%                                                                         %
% SA(:,[:]):  Index of vehicle                                            %
%                                                                         %  
% SA([1],:):  Same as CA([1],:)                                           %
% SA([2],:):  Equivalent volume of Auxiliary Reservoir [m^3]              %
% SA([3],:):  Maximum equivalent nozzle diameter of Aux. Reservoir [m]    %
% SA([4],:):  Absolute dP imposed by check valve AR-BP [Pa] [?]           %
% SA([5],:):  Initial absolute pressure in Auxiliary Resrvoirs. Set to    %
%             be the "on running" pressure assuming train starts in "on   %
%             run" condition [Pa]                                         % 
% SA([6],:):  Absolute pressure on running of Auxiliary Reservoir [Pa]    %
%                                                                         %
%             NOTE that data in SA(2:6,:) are not useful during braking   %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % [SBB] (alternative explanation) % % 

Piac = 1.01325e5*ones(1,nveicoli); % Modified on 13/09/2010
% CA = [gmisCV; Vac(Actv); Dac(Actv); PactvCV(Actv); Piac; Pcac(Actv)]; 
CA   = zeros(6,nveicoli); 
% CVactv = znv; % Commented 13/09/10 
% Data Auxiliary Reservoir
%matrix SA: position in m , volume , maximum equivalent SA diameter , dP imposed by ceck valve ,  
%P initial , P running
% SA = [gmisCV; Var(Actv); Dar(Actv); dParbp(Actv); pAR; Prnar(Actv)];
SA = zeros(6,nveicoli);
% Volume of Brake Cylinder
Vbc = znv; % TODO: there should be two volumes: one for block brake and the other for disk brake.
for ii = CVactv % 1:nveicoli commented 13/09/2010 
    PactvCV  = (P1-trainGUI(ii).DPactvCV);
    CA(:,ii) = [gmisCV(ii);trainGUI(ii).Vac;trainGUI(ii).Dac;PactvCV; ...
        Piac(ii);trainGUI(ii).Pcac];
    %CVactv(ii) = trainGUI(ii).CVactv; Commented 13/09/10   
    SA(:,ii) = [gmisCV(ii);trainGUI(ii).Var;trainGUI(ii).Dar; ...
        trainGUI(ii).dParbp;trainGUI(ii).Prnar;trainGUI(ii).Prnar];
    if isfield(trainGUI(ii),'bbS') && (not(isempty(trainGUI(ii).bbS)) || ...
            any(trainGUI(ii).bbS))
        Vbc(ii) = trainGUI(ii).Stkbc * trainGUI(ii).bbS/100;
    else
        Vbc(ii) = 0; %trainGUI(ii).Stkbc * trainGUI(ii).dbS;
    end
end

if not(isempty(CVactv)), CVactv = 1; else CVactv = 0; end; % Added 13/09/10
pCF = znv; % IT ASSUMES TRAIN STARTS IN "ON RUN" CONDITIONS
% t0 = clock;
Tsim = 0; % Simulation time is taken by operation data.

function [gmis,Lcoll,Lvago,LvagoC] = ridefLvago(dx,Lcoll,Lvago,nveicoli)

% RIDEFLVAGO This function takes into account the spatial discretization of
%            the Brake Pipe to redifine the Lvago vector and build LvagoC 
%            which also considers the hose couplings together with the 
%            Brake Pipe length of each vehicle 
% 
% INPUTS     dx:       Spatial discretization
%            Lcoll:    Length of hose couplings [m]
%            Lvago:    Vector with length of Brake Pipes [m]
%            nveicoli: Total number of vehicles
%
% OUTPUTS    gmis:     Discretized position of vehicle centers along the 
%                      Brake Pipe, taking also the hose couplings into
%                      account
%            Lcoll:    Discretized value of hose couplings length (floor)
%            Lvago:    Disretized vector with Brake Pipe lengths without
%                      hose couplings [m]
%            LvagoC:   Discretized vector with Brake Pipe lengths including
%                      hose couplings [m]

% In input the length of Brake Pipe doesn't consist of hose couplings 
% length too: Redefining Lvago taking in account couplings length
LvagoC      = Lvago + Lcoll; 
LvagoC(1)   = LvagoC(1)-0.5*Lcoll; 
LvagoC(end) = LvagoC(end)-0.5*Lcoll; % Length including couplings

% Redefining Lvago rounding to the nearest integers not considering
% couplings taking in account the spatial discretization, the input length
% of hose couplings is rounded to the nearest integers towards minus
% infinity.                                                                 [?] [s!] Maybe rephrase this paragraph to make it more clear
%(This instructions are needed to calculate the effective lenght of train
% necessary to calculate the propagation velocity)

Lcoll = floor(Lcoll);
% Lvago = ridefLvago(dx,Lcoll,LvagoC,nveicoli);
Lvago = LvagoC;


Lvago2  = zeros(1,nveicoli);
errlvag = 0;

for iv = 1:nveicoli
    if (iv == 1) || (iv == nveicoli)
        lvag = Lvago(iv)-0.5*Lcoll;
    else
        lvag = Lvago(iv)-Lcoll;
    end
    Lvago2(iv) = round((errlvag+lvag)/dx);
    errlvag = (errlvag+lvag)-Lvago2(iv);
end
Lvago2(nveicoli) = Lvago2(nveicoli)+round(errlvag/dx);
Lvago = Lvago2;

% Redefining LvagoC following the "discretization" criteria applied
% previously
LvagoC      = Lvago + Lcoll; 
LvagoC(1)   = LvagoC(1)   - 0.5*Lcoll; 
LvagoC(end) = LvagoC(end) - 0.5*Lcoll; % Length comprehensive of couplings

%-------------------------------------------------------------------
% Transfer function: vehicle number - progressive position along train
Lprg = [0 cumsum(LvagoC)];
VhcM = 1:nveicoli; %sort(VhcM);
gmis = Lprg(VhcM)+0.5*Lcoll+0.5*Lvago(VhcM);

gmis(VhcM == 1) = 0.5*Lvago(1);

function [CV,dLC,dTF,sknowncvs,trainGUI] = readbyfileCV(CV,dLC,dTF,fsl, ...
    ii,P1,pathGUI,pCFex,sep,sknowncvs,trainGUI,tt)

% READFILECV This function reads in information about a specified Control
%            Valve in case it is not already known from a previous function
%            call
%
% INPUTS     CV:        Cell with Control Valve information. First row, 
%                       Model of CV, second row vehicle with this CV.
%                       Number of columns equal to distinct CV models
%            dLC:       Array with limiting curve data of Control Valves.
%                       It has as many columns as vehicles
%            dTF:       Transfer function data
%            fsl:       '\' for Windows '/' for Linux
%            ii:        Index of current vehicle
%            P1:        Initial Brake Pipe pressure [Pa]                   
%            pathGUI:   Directory where ControlValve folder is located
%            pCFex:     Vector with Brake Cylinder experimental target 
%                       pressures
%            sep:       GUI seperator data
%            sknowncvs: Number of Control Valve models that have been read
%                       from input file so far
%            trainGUI:  Struct array with info about each vehicle
%            tt:        Currently equal to ii since all Control Valves are
%                       supposed to be active [?]
%
% OUTPUTS    CV:        Updated cell 
%            dLC:       Updated limiting curve data
%            dTF:       Updated transfer function data
%            sknowncvs: Updated counter
%            trainGUI:  Updated struct array

% Information is read by file
c = 0;
while c < sknowncvs
    c = c+1;
    if strcmp(CV{1,c},trainGUI(ii).CV)
        break
    end
end
if c == 0 || not(strcmp(CV{1,c},trainGUI(ii).CV))
    % It is necessary to read information by file
    lsep            = numel(sep);
    sknowncvs       = sknowncvs + 1;
    CV{1,sknowncvs} = trainGUI(ii).CV;
    CV{2,sknowncvs} = ii;
    
    nfile = [pathGUI fsl 'ControlValve' fsl trainGUI(ii).CV '.txt'];
    nf    = fopen(nfile);
    
    strtofind = 'asPresBrCyl='; cline = trova_info(strtofind,nf);
    %trainGUI(ii).pCFAs = 1e5*str2double(cline(numel(strtofind)+1:end));
    trainGUI(ii).pCFAs = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'asTime='; cline = trova_info(strtofind,nf);
    trainGUI(ii).tAS = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'genPipe='; cline = trova_info(strtofind,nf);
    trainGUI(ii).pCGAs = 1e5*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'ifPresBrCyl='; cline = trova_info(strtofind,nf);
    trainGUI(ii).pCFIf = 1e5*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'ifTime='; cline = trova_info(strtofind,nf);
    trainGUI(ii).tIf = str2double(cline(numel(strtofind)+1:end));
    trainGUI(ii).VCi = trainGUI(ii).pBC/3.8; % scaling of empty/load device
    trainGUI(ii).VCi = 1;                    % bypassed
    strtofind = 'btp95Pm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeBkP(1) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'btpPm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeBkP(2) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'btg95Pm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeBkG(1) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'btgPm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeBkG(2) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'rtp110Pm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeRlsP(1) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'rtpPm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeRlsP(2) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'rtg110Pm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeRlsG(1) = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'rtgPm='; cline = trova_info(strtofind,nf);
    trainGUI(ii).timeRlsG(2) = str2double(cline(numel(strtofind)+1:end));
    appo(ii) = ii; 
    trainGUI = mat_train(ii,lsep,nf,appo,trainGUI,sep,'brPresGP=','tfBr',1);
    trainGUI = mat_train(ii,lsep,nf,appo,trainGUI,sep,'brPresBC=','tfBr',2);
    trainGUI = mat_train(ii,lsep,nf,appo,trainGUI,sep,'rePresGP=','tfRe',1);
    trainGUI = mat_train(ii,lsep,nf,appo,trainGUI,sep,'rePresBC=','tfRe',2);
    strtofind = 'dpBPPABrCyl='; cline = trova_info(strtofind,nf);
    trainGUI(ii).DPbUPbc = 1e5*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'strBrCyl='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Stkbc = 1e-3*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'volAcCha='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Vac = 1e-3*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'diaAcCha='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Dac = 1e-3*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'dpBPPAAcCha='; cline = trova_info(strtofind,nf);
    trainGUI(ii).DPactvCV = 1e5*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'minPrClAcCha='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Pcac = 1e5*(1 + str2double(cline(numel(strtofind)+1:end)));
    strtofind = 'volAuxRes='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Var = 1e-3*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'diaAuxRes='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Dar = 1e-3*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'dpChVal='; cline = trova_info(strtofind,nf);
    trainGUI(ii).dParbp = 1e5*str2double(cline(numel(strtofind)+1:end));
    strtofind = 'prRunAuxRes='; cline = trova_info(strtofind,nf);
    trainGUI(ii).Prnar  = 1e5*(str2double(cline(numel(strtofind)+1:end)) + 1);
    trainGUI(ii).PbUPbc = ((P1/1e5-1)-trainGUI(ii).DPbUPbc/1e5);

    fclose(nf);
else
    c = CV{2,c};
    % Information is already known
    trainGUI(ii).pCFAs    = trainGUI(c).pCFAs;
    trainGUI(ii).tAS      = trainGUI(c).tAS;
    trainGUI(ii).pCGAs    = trainGUI(c).pCGAs;
    trainGUI(ii).pCFIf    = trainGUI(c).pCFIf;
    trainGUI(ii).tIf      = trainGUI(c).tIf;
    trainGUI(ii).VCi      = trainGUI(c).VCi;
    trainGUI(ii).timeBkP  = trainGUI(c).timeBkP;
    trainGUI(ii).timeBkG  = trainGUI(c).timeBkG;
    trainGUI(ii).timeRlsP = trainGUI(c).timeRlsP;
    trainGUI(ii).timeRlsG = trainGUI(c).timeRlsG;
    trainGUI(ii).tfBr     = trainGUI(c).tfBr;
    trainGUI(ii).tfRe     = trainGUI(c).tfRe;
    trainGUI(ii).DPbUPbc  = trainGUI(c).DPbUPbc; 
    trainGUI(ii).PbUPbc   = trainGUI(c).PbUPbc;
    trainGUI(ii).Stkbc    = trainGUI(c).Stkbc;
    trainGUI(ii).Vac      = trainGUI(c).Vac;
    trainGUI(ii).Dac      = trainGUI(c).Dac;
    trainGUI(ii).DPactvCV = trainGUI(c).DPactvCV;
    trainGUI(ii).Pcac     = trainGUI(c).Pcac;
    trainGUI(ii).Var      = trainGUI(c).Var;
    trainGUI(ii).Dar      = trainGUI(c).Dar;
    trainGUI(ii).dParbp   = trainGUI(c).dParbp;
    trainGUI(ii).Prnar    = trainGUI(c).Prnar;
end

DpAdj = (P1-6e5)/1e5; % This data contains the value to take in account the 
                      % adjustment on transfer function being refered to a 
                      % nominal starting pressure of 5 bar
                
% Data Transfer Functions
appo = trainGUI(ii).tfBr; [NpBk, s2] = size(appo);
dTF(tt,1:NpBk,1:s2) = appo;
appo2 = trainGUI(ii).tfRe; [NpRls, s22] = size(appo2);
dTF(tt,1+NpBk:NpBk+NpRls,1:s22) = appo2;

% Adjustment of BP pressure in Transfer function taking in account
% the real running pressure of manoeuvre
dTF(tt,1:(NpBk+NpRls),1) = dTF(tt,1:(NpBk+NpRls),1)+DpAdj;

Np               = 1+(NpBk+NpRls);
dTF(tt,Np,:)     = [NpBk,NpRls];
dTF(tt,1,2)      = pCFex(ii); % Substitute the experimental max BC pressure
dTF(tt,NpBk+1,2) = pCFex(ii); % in case differently specified by user [SBB]

% Data Limiting Functions
dLC(1,tt)     = trainGUI(ii).PbUPbc;
dLC(2:4,tt)   = [trainGUI(ii).pCFAs trainGUI(ii).tAS (P1 - trainGUI(ii).pCGAs)*1e-5-1]';
dLC(5:6,tt)   = [trainGUI(ii).pCFIf*1e-5 trainGUI(ii).tIf]';
dLC([2 5],tt) = trainGUI(ii).VCi*dLC([2 5],tt);

% Data braking liminting curve
tinh = dLC(3,tt)+dLC(6,tt); % end of inshot function %CC correzione per attualizzare tD
p95p = 0.95*dTF(tt,1,2);
pmx = dTF(tt,1,2);

% Data releasing liminting curve
pmn = dTF(tt,NpBk+NpRls,2);
p110p = 1.1*pmn;

if trainGUI(ii).RgBk == 1 % Passenger regime
    % Braking
    %t95p = trainGUI(ii).timeBkP(1); tmx = trainGUI(ii).timeBkP(2);
    t95p   = trainGUI(ii).t95; tmx = trainGUI(ii).tmx;
    [CfPb] = polyfit([tinh t95p tmx],[dLC(5,tt) p95p pmx],2);
    % Releasing
    t110p  = trainGUI(ii).timeRlsP(1); tmn = trainGUI(ii).timeRlsP(2);
    [CfPr] = polyfit([0 t110p tmn],[pmx p110p pmn],2);
    
    dLC(7:end,tt) = [CfPb'; tmx; CfPr'; tmn]; % coefficient of limiting curves (braking releasing) assigning
else % Goods regime
    % Braking
    %t95p = trainGUI(ii).timeBkG(1); tmx = trainGUI(ii).timeBkG(2);
    t95p   = trainGUI(ii).t95; tmx = trainGUI(ii).tmx;
    [CfGb] = polyfit([tinh t95p tmx],[dLC(5,tt) p95p pmx],2);
    % Releasing
    t110p  = trainGUI(ii).timeRlsG(1); tmn = trainGUI(ii).timeRlsG(2);
    [CfGr] = polyfit([0 t110p tmn],[pmx p110p pmn],2);
    
    dLC(7:end,tt) = [CfGb'; tmx; CfGr'; tmn]; % coefficient of limiting curves (braking releasing) assigning
end

function [DBVd,DBVname,sknowndbvs,trainGUI] = readbyfileDBV(DBVd,DBVname,...
    fsl,ii,nDBV,pathGUI,sknowndbvs,trainGUI)

% READBYFILEDBV Read in information about Driver's Brake Valve from input
%               files
%
% INPUTS        DBVd:       Matrix that collects DBV data 
%               DBVname:    Cell with Driver's Brake Valve models of train
%               fsl:        '\' for Windows '/' for Linux
%               ii:         Index of current vehicle
%               nDBV:       Number of driver's brake valves
%               pathGUI:    Directory where the BrakeValve folder is
%                           located
%               sknowndbvs: Number of driver's brake valve models that have 
%                           been read from input file so far
%               trainGUI:   Struct array with info about each vehicle      

% 
% OUTPUTS       DBVd:       Updated data matrix
%               DBVname:    Updated cell
%               sknowndbvs: Updated counter
%               trainGUI:   Updated struct array

% Information read by files
c = 0;
while c < sknowndbvs
    c = c+1;
    if strcmp(DBVname{c},trainGUI(ii).DBV)
        break
    end
end
if c == 0 || not(strcmp(DBVname{c},trainGUI(ii).DBV))
    % It is necessary to read information by file
    sknowndbvs          = sknowndbvs + 1;
    DBVname{sknowndbvs} = trainGUI(ii).DBV;
    
    nfile = [pathGUI fsl 'BrakeValve' fsl trainGUI(ii).DBV '.txt'];
    nf    = fopen(nfile);
    
    strtofind = 'ebdiameo='; cline = trova_info(strtofind,nf);
    trainGUI(ii).ebd = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'sbdiameo='; cline = trova_info(strtofind,nf);
    trainGUI(ii).sbd = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'rediameo='; cline = trova_info(strtofind,nf);
    trainGUI(ii).red = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'chkebpl='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true')
        trainGUI(ii).ebfl = -1;
    else
        strtofind = 'ebflcoef='; cline = trova_info(strtofind,nf);
        trainGUI(ii).ebfl = str2double(cline(numel(strtofind)+1:end));
    end
    strtofind = 'chksbpl='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true')
        trainGUI(ii).sbfl = -1;
    else
        strtofind = 'sbflcoef='; cline = trova_info(strtofind,nf);
        trainGUI(ii).sbfl = str2double(cline(numel(strtofind)+1:end));
    end
    strtofind = 'chkrepl='; cline = trova_info(strtofind,nf);
    if strcmp(cline(numel(strtofind)+1:end),'true')
        trainGUI(ii).refl = -1;
    else
        strtofind = 'reflcoef='; cline = trova_info(strtofind,nf);
        trainGUI(ii).refl = str2double(cline(numel(strtofind)+1:end));
    end
    strtofind = 'sbtad15='; cline = trova_info(strtofind,nf);
    trainGUI(ii).sb15 = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'sbtd='; cline = trova_info(strtofind,nf);
    trainGUI(ii).sbtd = str2double(cline(numel(strtofind)+1:end));
    strtofind = 'retai15='; cline = trova_info(strtofind,nf);
    trainGUI(ii).re15 = str2double(cline(numel(strtofind)+1:end));
    fclose(nf);
else
    trainGUI(ii).ebd  = trainGUI(c).ebd;
    trainGUI(ii).sbd  = trainGUI(c).sbd;
    trainGUI(ii).red  = trainGUI(c).red;
    trainGUI(ii).ebfl = trainGUI(c).ebfl;
    trainGUI(ii).sbfl = trainGUI(c).sbfl;
    trainGUI(ii).refl = trainGUI(c).refl;
    trainGUI(ii).sb15 = trainGUI(c).sb15;
    trainGUI(ii).sbtd = trainGUI(c).sbtd;
    trainGUI(ii).re15 = trainGUI(c).re15;
end

% EMERGENCY BRAKING DATA
DBVd(3*(nDBV-1)+1,1) = trainGUI(ii).ebd/1000; % Diameter of emergency braking  [mm] % [s!] It is converted to [m] so not [mm] anymore. Same for the variables below
DBVd(3*(nDBV-1)+1,2) = trainGUI(ii).ebfl;     % Flow coefficient of orifice Cq [-]
% SERVICE BRAKING DATA
DBVd(3*(nDBV-1)+2,1) = trainGUI(ii).sbd/1000; % Diameter of equivalent orifice [mm]
DBVd(3*(nDBV-1)+2,2) = trainGUI(ii).sbfl;     % Flow coefficient of orifice Cq  [-]
DBVd(3*(nDBV-1)+2,3) = trainGUI(ii).sb15;     % Time to achieve a drop of 1.5 bar [s]
DBVd(3*(nDBV-1)+2,4) = trainGUI(ii).sbtd;     % Time of first lift increasing     [s]
% RELEASING DATA
DBVd(3*(nDBV-1)+3,1) = trainGUI(ii).red/1000; % Diameter of equivalente orifice [mm]
DBVd(3*(nDBV-1)+3,2) = trainGUI(ii).refl;     % Flow coefficient of orifice Cq  [-]
DBVd(3*(nDBV-1)+3,3) = trainGUI(ii).re15;     % Time to achieve an increasing of 1.5 bar [s]

