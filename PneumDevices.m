function [CA,CVactv,D1,DBVdata,dLC,dTF,dx,gmis,gmisCF,...
          Lcoll,Lvago,nveicoli,P1,pCF,SA,trainGUI,Tsim,Vbc] = PneumDevices(fsl,Lcoll,P1,pathGUI,sep,trainGUI)
        
global lam s  

% This function reads input of pneumatic devices

% CONSTANTS OF THE PNEUMATIC MODEL
% Assignment data for calculation of thermal flux through pipe
s = 3e-3; % tickness of pipe
lam = 14; % thermal conducibility of pipe (S.I.) [W/(m*K)]

dx = 1; % spatial discretization
% tvis = 1; % Video display time


nveicoli = length(trainGUI); % Number of vehicles
% VhcM = 1:nveicoli; % Vehicles where pneumatic information are available.

% From trainGUI are extracted some variables. Variables not stored before are stored
% now.
znv = zeros(1,nveicoli); Lvago = znv; D1 = znv; pCFmx = znv; pCFex = znv;
% DPbUPbc = znv; PbUPbc = znv;
dLC = zeros(14,1); nCV = 0; nDBV = 0;
% dTF([:],:,:) : number of vehicle
% dTF(:,[:],:) : series of points to build transfer function before braking and after releasing data points
% dTF(:,:,[:]) : pCG pCF  starting from maximum pressure drop
dTF = zeros(1,2,2);
% DBVd is a matrix that collects DBV data:
% (1,:) : emergency braking data EB
% (2,:) : service braking data SB
% (3,:) : releasing data R
% (4,:) : EP data
sknowndbvs = 0; DBVd = zeros(3,10);
sknowncvs = 0; CV = cell(2,1); DBVname = cell(1);
CVactv = znv;
for ii = 1:nveicoli
    Lvago(ii) = trainGUI(ii).L;
    D1(ii) = trainGUI(ii).D;
    pCFmx(ii) = trainGUI(ii).pBC;
    pCFex(ii) = trainGUI(ii).pBCexp;
    if trainGUI(ii).CVactv == 1
        nCV = nCV + 1; %tt = nCV;  
        CVactv(ii) = ii;
        [CV,dLC,dTF,sknowncvs,trainGUI] = readbyfileCV(CV,dLC,dTF,fsl,ii,P1,pathGUI,pCFex,sep,...
            sknowncvs,trainGUI,ii); % Modified 13/09/2010
    end
    if strcmp(trainGUI(ii).type,'loco')
        nDBV = nDBV + 1;
        [DBVd,DBVname,sknowndbvs,trainGUI] = readbyfileDBV(DBVd,DBVname,fsl,ii,nDBV,pathGUI,...
            sknowndbvs,trainGUI);
       
    end
end
CVactv(CVactv==0) = [];

[gmis,Lcoll,Lvago,LvagoC] = ridefLvago(dx,Lcoll,Lvago,nveicoli);

% TODO: CHECK CODE BEHAVIOUR WHEN THERE ARE SOME CVs DISABLED: ACTUALLY IT IS ASSUMED
% EVERY CV WORKS.
% Progressive position of control valve
% Transfer function vehicle number-progressive position along train
% Lprg = [0 cumsum(LvagoC)];

posDBV = zeros(1,nDBV); c = 0;
for ii = 1:nveicoli
    if strcmp(trainGUI(ii).type,'loco');
        c = c + 1;
        posDBV(c) = ii;
    end
end
% DBVdata collects all data of DBV for the simulation starting from first: for each DBV you
% have 4 line: 1° rapid braking data, position, delay ; 2° service braking
% data 3° releasing data 4° electropneumatic data
DBVdata = zeros(3*nDBV,10);
k = 0; %index to move inside the matrix  DBVdata
Lprg = [0 cumsum(LvagoC)];
for ii = 1:nDBV
    % Progressive position of DBV (redefine position)
    % Transfer function vehicle number-progressive position along train
    if posDBV(ii) >= 1
        DBVd(3*(ii-1)+1,3) = Lprg(posDBV(ii))+0.5*Lcoll+Lvago(posDBV(ii))./4;
    elseif posDBV(ii) == 1
        DBVd(3*(ii-1)+1,3) = Lvago(posDBV(ii))./4;
    end;
    % Assignement matrix DBV in simulation
    % With the GUI input there are as many DBVs as the locomotives
    DBVdata(3*k+1:3*k+3,:) = DBVd(3*(ii-1)+1:3*(ii-1)+3,:);
    k = k+1;
end;

Actv = 1:nveicoli;
CV_post = zeros(size(Actv)); % TODO: add in the GUI input this feature.
if abs(CV_post(Actv)) < Lvago(Actv)*0.5
    gmisCV = Lprg(Actv)+0.5*Lcoll+0.5*Lvago(Actv)+CV_post(Actv);
    gmisCV(Actv == 1) = 0.5*Lvago(1)+CV_post(1);
    gmisCF = gmisCV;
else
    error('CV_post too long, it must be shorter than half length of the wagon');
end

% Storing data in matricies used for the computation
% Data acceleration chamber
%matrix CA: position in m , volume , diameter , P activation , P initial ,
%P closing 
Piac = 1.01325e5*ones(1,nveicoli); % Modified on 13/09/2010
% CA = [gmisCV; Vac(Actv); Dac(Actv); PactvCV(Actv); Piac; Pcac(Actv)]; 
CA = zeros(6,nveicoli); 
% CVactv = znv; % Commented 13/09/10 
% Data auxiliary reservoir
%matrix SA: position in m , volume , maximum equivalent SA diameter , dP imposed by ceck valve ,  
%P initial , P running
% SA = [gmisCV; Var(Actv); Dar(Actv); dParbp(Actv); pAR; Prnar(Actv)];
SA = zeros(6,nveicoli);
% Volume of brake cylinders
Vbc = znv; % TODO: there should be two volumes: one for block brake and the other for disk brake.
for ii = CVactv % 1:nveicoli commented 13/09/2010 
    PactvCV = (P1-trainGUI(ii).DPactvCV);
    CA(:,ii) = [gmisCV(ii);trainGUI(ii).Vac;trainGUI(ii).Dac;PactvCV;Piac(ii);trainGUI(ii).Pcac];
    %CVactv(ii) = trainGUI(ii).CVactv; Commented 13/09/10   
    SA(:,ii) = [gmisCV(ii);trainGUI(ii).Var;trainGUI(ii).Dar;trainGUI(ii).dParbp;trainGUI(ii).Prnar;trainGUI(ii).Prnar];
    if isfield(trainGUI(ii),'bbS') && (not(isempty(trainGUI(ii).bbS)) || ...
            any(trainGUI(ii).bbS))
        Vbc(ii) = trainGUI(ii).Stkbc * trainGUI(ii).bbS/100;
    else
        Vbc(ii) = 0; %trainGUI(ii).Stkbc * trainGUI(ii).dbS;
    end;
end
if not(isempty(CVactv)), CVactv = 1; else CVactv = 0; end; % Added 13/09/10
pCF = znv; % IT ASSUMES TRAIN STARTS IN "ON RUN" CONDITIONS
% t0 = clock;
Tsim = 0; % Simulation time is taken by operation data.





function [gmis,Lcoll,Lvago,LvagoC] = ridefLvago(dx,Lcoll,Lvago,nveicoli)

% In input the length of brake pipe doesn't consist of hose couplings length too:
% Redefining Lvago taking in account couplings length
LvagoC = Lvago+Lcoll; LvagoC(1) = LvagoC(1)-0.5*Lcoll; LvagoC(end) = LvagoC(end)-0.5*Lcoll; %Length comprehensive of couplings


% Redefining Lvago rounding to the nearest integers not considering
% couplings taking in account the spatial discretization, the input length
% of hose couplings is rounded to the nearest integers towards minus
% infinity. 
%(This instructions are needed to calculate the effective lenght of train
% necessary to calculate the propagation velocity)
Lcoll = floor(Lcoll);
% Lvago = ridefLvago(dx,Lcoll,LvagoC,nveicoli);
Lvago = LvagoC;

Lvago2 = zeros(1,nveicoli);
errlvag = 0;

for iv = 1:nveicoli
    if (iv == 1) || (iv == nveicoli)
        lvag = Lvago(iv)-0.5*Lcoll;
    else
        lvag = Lvago(iv)-Lcoll;
    end;
    Lvago2(iv) = round((errlvag+lvag)/dx);
    errlvag = (errlvag+lvag)-Lvago2(iv);
end
Lvago2(nveicoli) = Lvago2(nveicoli)+round(errlvag/dx);
Lvago = Lvago2;

% Redefining LvagoC following the "discretization" criteria applied
% previously
LvagoC = Lvago+Lcoll; LvagoC(1) = LvagoC(1)-0.5*Lcoll; LvagoC(end) = LvagoC(end)-0.5*Lcoll; %Length comprehensive of couplings

%-------------------------------------------------------------------
% Transfer function vehicle number-progressive position along train
Lprg = [0 cumsum(LvagoC)];
VhcM = 1:nveicoli; %sort(VhcM);
gmis = Lprg(VhcM)+0.5*Lcoll+0.5*Lvago(VhcM);
gmis(VhcM == 1) = 0.5*Lvago(1);

function [CV,dLC,dTF,sknowncvs,trainGUI] = readbyfileCV(CV,dLC,dTF,fsl,ii,P1,pathGUI,pCFex,sep,...
    sknowncvs,trainGUI,tt)

% Information read by files
c = 0;
while c < sknowncvs
    c = c+1;
    if strcmp(CV{1,c},trainGUI(ii).CV)
        break
    end
end
if c == 0 || not(strcmp(CV{1,c},trainGUI(ii).CV))
    % It is necessary to read information by file
    lsep = numel(sep);
    sknowncvs = sknowncvs + 1;
    CV{1,sknowncvs} = trainGUI(ii).CV;
    CV{2,sknowncvs} = ii;
    nfile = [pathGUI fsl 'ControlValve' fsl trainGUI(ii).CV '.txt'];
    nf = fopen(nfile);
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
    trainGUI(ii).VCi = 1; % bypassed
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
    trainGUI(ii).Prnar = 1e5*(str2double(cline(numel(strtofind)+1:end)) + 1);

    trainGUI(ii).PbUPbc = ((P1/1e5-1)-trainGUI(ii).DPbUPbc/1e5);

    fclose(nf);
else
    c = CV{2,c};
    % Information are already known
    trainGUI(ii).pCFAs = trainGUI(c).pCFAs;
    trainGUI(ii).tAS = trainGUI(c).tAS;
    trainGUI(ii).pCGAs = trainGUI(c).pCGAs;
    trainGUI(ii).pCFIf = trainGUI(c).pCFIf;
    trainGUI(ii).tIf = trainGUI(c).tIf;
    trainGUI(ii).VCi = trainGUI(c).VCi;
    trainGUI(ii).timeBkP = trainGUI(c).timeBkP;
    trainGUI(ii).timeBkG = trainGUI(c).timeBkG;
    trainGUI(ii).timeRlsP = trainGUI(c).timeRlsP;
    trainGUI(ii).timeRlsG = trainGUI(c).timeRlsG;
    trainGUI(ii).tfBr = trainGUI(c).tfBr;
    trainGUI(ii).tfRe = trainGUI(c).tfRe;
    trainGUI(ii).DPbUPbc = trainGUI(c).DPbUPbc; 
    trainGUI(ii).PbUPbc = trainGUI(c).PbUPbc;
    trainGUI(ii).Stkbc = trainGUI(c).Stkbc;
    trainGUI(ii).Vac = trainGUI(c).Vac;
    trainGUI(ii).Dac = trainGUI(c).Dac;
    trainGUI(ii).DPactvCV = trainGUI(c).DPactvCV;
    trainGUI(ii).Pcac = trainGUI(c).Pcac;
    trainGUI(ii).Var = trainGUI(c).Var;
    trainGUI(ii).Dar = trainGUI(c).Dar;
    trainGUI(ii).dParbp = trainGUI(c).dParbp;
    trainGUI(ii).Prnar = trainGUI(c).Prnar;
end
DpAdj = (P1-6e5)/1e5; % this data contains the value to taking in account the 
                %adjustment on transfer function being refered to a nominal
                %starting pressure of 5 bar
% Data Transfer Functions
appo = trainGUI(ii).tfBr; [NpBk s2] = size(appo);
dTF(tt,1:NpBk,1:s2) = appo;
appo2 = trainGUI(ii).tfRe; [NpRls s22] = size(appo2);
dTF(tt,1+NpBk:NpBk+NpRls,1:s22) = appo2;
% Adjustment of BP pressure in Transfer function taking in account
% the real running pressure of manoeuvre
dTF(tt,1:(NpBk+NpRls),1) = dTF(tt,1:(NpBk+NpRls),1)+DpAdj;
Np = 1+(NpBk+NpRls);
dTF(tt,Np,:) = [NpBk,NpRls];
dTF(tt,1,2) = pCFex(ii);
dTF(tt,NpBk+1,2) = pCFex(ii);
% Data Limiting Functions
dLC(1,tt) = trainGUI(ii).PbUPbc;
dLC(2:4,tt) = [trainGUI(ii).pCFAs trainGUI(ii).tAS (P1 - trainGUI(ii).pCGAs)*1e-5-1]';
dLC(5:6,tt) = [trainGUI(ii).pCFIf*1e-5 trainGUI(ii).tIf]';
dLC([2 5],tt) = trainGUI(ii).VCi*dLC([2 5],tt);
% Data braking liminting curve
tinh = dLC(3,tt)+dLC(6,tt); % end of inshot function %CC correzione per attualizzare tD
p95p = 0.95*dTF(tt,1,2);
pmx = dTF(tt,1,2);

% Data releasing liminting curve
pmn = dTF(tt,NpBk+NpRls,2);
p110p = 1.1*pmn;

if trainGUI(ii).RgBk == 1 %Passenger regime
    % Braking
    %t95p = trainGUI(ii).timeBkP(1); tmx = trainGUI(ii).timeBkP(2);
    t95p = trainGUI(ii).t95; tmx = trainGUI(ii).tmx;
    [CfPb] = polyfit([tinh t95p tmx],[dLC(5,tt) p95p pmx],2);
    % Releasing
    t110p = trainGUI(ii).timeRlsP(1); tmn = trainGUI(ii).timeRlsP(2);
    [CfPr] = polyfit([0 t110p tmn],[pmx p110p pmn],2);
    
    dLC(7:end,tt) = [CfPb'; tmx; CfPr'; tmn]; %coefficient of limiting curves (braking releasing) assigning
else % Goods regime
    % Braking
    %t95p = trainGUI(ii).timeBkG(1); tmx = trainGUI(ii).timeBkG(2);
    t95p = trainGUI(ii).t95; tmx = trainGUI(ii).tmx;
    [CfGb] = polyfit([tinh t95p tmx],[dLC(5,tt) p95p pmx],2);
    % Releasing
    t110p = trainGUI(ii).timeRlsG(1); tmn = trainGUI(ii).timeRlsG(2);
    [CfGr] = polyfit([0 t110p tmn],[pmx p110p pmn],2);
    
    dLC(7:end,tt) = [CfGb'; tmx; CfGr'; tmn]; %coefficient of limiting curves (braking releasing) assigning
end;

function [DBVd,DBVname,sknowndbvs,trainGUI] = readbyfileDBV(DBVd,DBVname,fsl,ii,nDBV,pathGUI,...
    sknowndbvs,trainGUI)

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
    sknowndbvs = sknowndbvs + 1;
    DBVname{sknowndbvs} = trainGUI(ii).DBV;
    nfile = [pathGUI fsl 'BrakeValve' fsl trainGUI(ii).DBV '.txt'];
    nf = fopen(nfile);
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
    trainGUI(ii).ebd = trainGUI(c).ebd;
    trainGUI(ii).sbd = trainGUI(c).sbd;
    trainGUI(ii).red = trainGUI(c).red;
    trainGUI(ii).ebfl = trainGUI(c).ebfl;
    trainGUI(ii).sbfl = trainGUI(c).sbfl;
    trainGUI(ii).refl = trainGUI(c).refl;
    trainGUI(ii).sb15 = trainGUI(c).sb15;
    trainGUI(ii).sbtd = trainGUI(c).sbtd;
    trainGUI(ii).re15 = trainGUI(c).re15;
end

% EMERGENCY BRAKING DATA
DBVd(3*(nDBV-1)+1,1) = trainGUI(ii).ebd/1000; % Diameter of emergency braking [mm]
DBVd(3*(nDBV-1)+1,2) = trainGUI(ii).ebfl; % Flow coefficient of orifice Cq [-]
% SERVICE BRAKING DATA
DBVd(3*(nDBV-1)+2,1) = trainGUI(ii).sbd/1000; % Diameter of equivalente orifice [mm]
DBVd(3*(nDBV-1)+2,2) = trainGUI(ii).sbfl; % Flow coefficient of orifice Cq [-]
DBVd(3*(nDBV-1)+2,3) = trainGUI(ii).sb15; % Time to achieve a drop of 1.5 bar [s]
DBVd(3*(nDBV-1)+2,4) = trainGUI(ii).sbtd; % Time of first lift increasing [s]
% RELEASING DATA
DBVd(3*(nDBV-1)+3,1) = trainGUI(ii).red/1000; % Diameter of equivalente orifice [mm]
DBVd(3*(nDBV-1)+3,2) = trainGUI(ii).refl; % Flow coefficient of orifice Cq [-]
DBVd(3*(nDBV-1)+3,3) = trainGUI(ii).re15; % Time to achieve a increasing of 1.5 bar [s]


