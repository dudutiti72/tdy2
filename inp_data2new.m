function [CA,CVactv,D1,DBVdata,Dcoll,dLC,Dpy,dTF,dx,epsilon,EPV,EPV_active,gmis,gmisCF,iDCVa,kcoll,Lcoll,...
          Lvago,nomefilese,nomepath,nveicoli,P1,pCF,posDBV,RgBk,SA,SR,t0,t95p,tandem,Tsim,tvis,typeDBV,Vbc,VhcM,...
          VhcMa,vManovra,vmaster,VV] = inp_data2new(namedirfile,dynamic,nveicoli)

%# scalar CVactv Dpy errlvag ii iv k nveicoli P1 SR Tsim rc
  
global lam s Tamb 

% This function provide input data for computations.

%GENERIC CONSTANT
% Assignment data for calculation of thermal flux through pipe
s = 3e-3; % tickness of pipe
lam = 14; % thermal conducibility of pipe (S.I.) [W/(m*K)]

dx = 1; % spatial discretization
tvis = 1; % Tempo di visualizzazione a video


if nargin == 0
    t0 = clock; %starting time to calculate elapsed computational time

    %READING INPUT FILE TXT

    % Assignement of folder that contains the input files for simulation
    old_dir = pwd; % Memorize the main directory
    nf = fopen('scambio.inf','r');	  	   % Open the file scambio.inf where read the simulation
    nomefile = fscanf(nf,'%c',256);		   % read the initial 256 characters
    fclose(nf);                            % Close the file scambio.inf
    cr = find(nomefile == char(13));	   % Carriage return: identify the 'nomefile' individuo in 'nomefile' il primo
    % 'a capo'(char(13)) determino nomefile come il vecchio
    nomefile = nomefile(1:cr(1)-1);		   % Nomefile ma fino al suo primo 'a capo'

    fsl = '\';
    nomepath = [nomefile,fsl]; % Set the working directory

    [nomefilese] = path_nome(nomefile,fsl); % Extrapolate from file scambio.inf the name of simulation
    warning off
    cd(nomepath);
    mkdir('DataT'); % making the directory where record the BC and BP pressure
    cd(old_dir); % Return to the main directory

    dynamic = [];
else
    nomepath = namedirfile;
    nomefilese = []; t0 = [];
end


% GENERIC DATA
data = 'GenericData';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Instruction read only by MIDEVA.
   load (nomefile,'A');
else
   % Instruction read only by MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;

% Reading data
%# fastindex
rc = 1;
SR = A(rc); % Sampling rate data writing
rc = rc+1;
%# fastindex
Dpy = A(rc); % variable to control the operation of real-time BP pressure display ( 1: enable ; 0: disable)
rc = rc+1;
Tamb = A(rc); %External temperature [K] 
rc = rc+1;
epsilon = A(rc)/1000; % roughness of brake pipe (usually 0.046e-3 m)
rc = rc+1;
Dcoll = A(rc)/1000; % Diameter hose coupling
rc = rc+1;
Lcoll = A(rc); % Length of hose coupling
rc = rc+1;
kcoll = A(rc); % Concentrated pressure loss of the hose couplings (supposed all equal)
rc = rc+1;
%# fastindex
CVactv = A(rc); % If the control valves are active
rc = rc+1;
P1 = (A(rc)+1)*1e5; % initial pressure
rc = rc+1;
nManv = A(rc); % number of total manoeuvres of simulation
rc = rc+1;
nDBV = A(rc); % number of total DBV in composition
rc = rc+1; typeDBV = zeros(1,nDBV); posDBV = typeDBV;
for ii = 1:nDBV 
    typeDBV(ii) = A(rc); % type of driver's brake valve
    rc = rc+1;
end
for ii = 1:nDBV
    posDBV(ii) = A(rc); % position of driver's brake valve
    rc = rc+1;
end

if nargin > 0
    VhcM = 1:nveicoli; % Added for dynamics
else
    % OUTPUT DATA
    data = 'OutputData';
    nomefile = [nomepath,data,'.txt'];
    if exist('_MATCOM_') ~= 0
        % Instruction read only by MIDEVA.
        load (nomefile,'A');
    else
        % Instruction read only by MATLAB.
        load (nomefile,'A');
        A = eval(data);
    end;
    %Data reading
    rc = 1;
    VhcM = A(rc,:); % vehicles instrumented along the train (N.B. where the CV is active)
end

% BRAKE PIPE AND DISTRIBUTOR DATA
[CA,D1,dLC,dTF,gmis,gmisCF,iDCVa,infoEPB,Lcoll,Lvago,LvagoC,...
    Lprg,Actv,pCF,RgBk,SA,t95p,tandem,Vbc,VhcMa,vmaster,VV] = dataBpCv(CVactv,dx,Lcoll,nomepath,nveicoli,P1,VhcM);

% MANOUVRE DATA
[DBVdata,Tsim,vManovra] = dataManoeuvre(D1,dynamic,Lcoll,Lvago,LvagoC,nDBV,nManv,nomepath,P1,posDBV,typeDBV);

if length(infoEPB) > 1
    % ELECTROPNEUMATIC DATA
    [EPV,EPV_active]= dataEPB(infoEPB,Lcoll,Lvago,Lprg,Actv);
else
    EPV = []; EPV_active = [];
end
if size(CA,2) == 0, CVactv = 0; end
if size(VV,2) > 0
    if size(vManovra,1) > 1 || vManovra(1,1) ~= -1
       % error('Actually QR brake is available only for emergency brake');
    end
end




%Input data brake pipe STARTING---------------------------------------------------------
function [CA,D1,dLC,dTF,gmis,gmisCF,iDCVa,infoEPB,Lcoll,Lvago,LvagoC,...
    Lprg,Actv,pCF,RgBk,SA,t95p,tandem,Vbc,VhcMa,vmaster,VV] = dataBpCv(CVactv,dx,Lcoll,nomepath,nveicoli,P1,VhcM)

% Brake pipe data calculation
%# scalar CVactv ii iv nveicoli

% OVERALL DATA BP
data = 'OverallDataBP';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Instruction read only by MIDEVA.
   load (nomefile,'A');
else
   % Instruction read only by MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
% nveicoli = size(A,2);
VhcM = 1:nveicoli; % Added for dynamics
rc = 1;
Lvago = A(rc,:);

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
Lvago = ridefLvago(dx,Lcoll,LvagoC,nveicoli);

% Redefining LvagoC following the "discretization" criteria applied
% previously
LvagoC = Lvago+Lcoll; LvagoC(1) = LvagoC(1)-0.5*Lcoll; LvagoC(end) = LvagoC(end)-0.5*Lcoll; %Length comprehensive of couplings

%-------------------------------------------------------------------
% Transfer function vehicle number-progressive position along train
Lprg = [0 cumsum(LvagoC)];
VhcM = sort(VhcM);
gmis = Lprg(VhcM)+0.5*Lcoll+0.5*Lvago(VhcM);
gmis(VhcM == 1) = 0.5*Lvago(1);
%-------------------------------------------------------------------

rc = rc+1;
D1 =  A(rc,:)/1000;

%# fastindex
if CVactv == 1
    % Calculation input data distributor
    [Actv,CA,dLC,dTF,gmisCF,iDCVa,infoEPB,pCF,RgBk,SA,t95p,tandem,Vbc,VhcMa,vmaster,...
        VV] = dataCVs(D1,Lcoll,Lvago,LvagoC,nomepath,nveicoli,P1,VhcM);
else
    SA = []; CA = []; pCF = []; dLC = []; dTF = []; VhcMa = []; iDCVa = []; 
    gmisCF = []; Vbc = []; VV = []; vmaster = []; infoEPB = []; tandem = []; t95p = [];
end;
%Input data brake pipe END--------------------------------------------------------


%Input data distributor emulation STARTING---------------------------------------------------------
function [Actv,CA,dLC,dTF,gmisCF,iDCVa,infoEPB,pCF,RgBk,SA,t95p,tandem,Vbc,VhcMa,vmaster,...
        VV] = dataCVs(D1,Lcoll,Lvago,LvagoC,nomepath,nveicoli,P1,VhcM)

% Control valve data calculation

% OVERALL DATA CV
data = 'OverallDataCV';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Instruction read only by MIDEVA.
   load (nomefile,'A');
else
   % Instruction read only by MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;

%Data reading
rc = 1;
pCFmx = A(rc,:);
rc = rc+1;
pCFex = A(rc,:);
rc = rc+1;
DPbUPbc = A(rc,:); PbUPbc = ((P1/1e5-1)-DPbUPbc);
rc = rc+1;  
RgBk = A(rc,:); % 1 passengers 0 goods
rc = rc+1;
VCi =  A(rc,:); % variable that contain index to define type  and scaling ratio of Empty/load device
VCi(2,:) = [pCFmx./3.8]; % scaling of empty/load device
rc = rc+1;
dBC = ((A(rc,:)*25.4)/1000); % Diameter of the braking cylinder
Sbc = (pi./4.*dBC.^2); % Cross section of the braking cylinder
rc = rc+1;
Stkbc = (A(rc,:)/1000); % Stroke of the braking cylinder
Vbc = Sbc.*Stkbc;
rc = rc+1;
Vac = (A(rc,:)/1000); % Equivalent Volume of the accelerating chamber (AC)
rc = rc+1;
Dac = (A(rc,:)/1000); % Diametr of the nozzle that links the BP with the AC
rc = rc+1;
DPactvCV = A(rc,:)*1e5; PactvCV = (P1-DPactvCV); % Pressure drop that starts the control valve
rc = rc+1;  
Pcac = ((A(rc,:)+1)*1e5); % Pressure for closing the acceleration chambers: below this pressure there is no communication among BP and AC
rc = rc+1;
Var =  (A(rc,:)/1000); % Volume of the auxiliary reservoir
rc = rc+1;
Dar =  (A(rc,:)/1000); % Equivalent diameter of the nozzle that links the auxiliary reservoir to the BP
rc = rc+1;
dParbp =  (A(rc,:)*1e5); % Pressure drop for the auxiliary reservoir
rc = rc+1;
Prnar = ((A(rc,:)+1)*1e5); % running pressure on auxiliary reservoir
rc = rc+1;
if rc > size (A,1)
    CV_post = zeros(size(Lvago));
else
    CV_post = (A(rc,:)); % Position of the control valve compared to half wagon
end    
% Managing EP brake and QR brake
nf = fopen(nomefile,'r');
cline = trova_info('% QR DATA',nf);
if not(isempty(cline))
    % The train is equipped with one or more QR brake
    cline = fgetl(nf);
    VV_pos = str2num(fgetl(nf));
    cline = fgetl(nf);
    pos_master_slave = str2num(fgetl(nf));
    [vmaster,tandem] = gen_master_tandem(pos_master_slave);
    cline = fgetl(nf);
    VV_Diam = 1e-3*str2num(fgetl(nf));
    cline = fgetl(nf);
    VV_Vol = str2num(fgetl(nf)); % Volume of the limiting reservoir
    cline = fgetl(nf);
    VV_Sec = str2num(fgetl(nf)); % Cross section of the nozzle that connects the limiting reservoir with the distributor
    VV = [VV_pos;VV_Diam.^2*pi*0.25;VV_Vol;VV_Sec];
    VV = inputVV(P1,VV);
else
    vmaster = 1:nveicoli; tandem = []; VV = [];
end
cline = trova_info('% EV DATA',nf);
if not(isempty(cline))
    % The train is equipped with one or more EP brake vehicle
    cline = fgetl(nf);
    grad = str2num(fgetl(nf));
    Depb = interpolEV(nomepath,Lvago,nveicoli,D1,grad);
    cline = fgetl(nf);
    LocoRif = str2num(fgetl(nf));
    cline = fgetl(nf);
    DelayEpb = str2num(fgetl(nf));
    cline = fgetl(nf);
    Epb_post = str2num(fgetl(nf)); % Position of the EPvalve compared to a quarter of wagon
    infoEPB = [Depb;LocoRif;DelayEpb;Epb_post];
else
    infoEPB = [];
end
fclose(nf);


RvolBcAr = Vbc./Var;

% Defining active CV and Imposing maximum BC experimental data
appoD = find(pCFex == 0); nCV = nveicoli-size(appoD,2); %assigmnet number of CV active %%CC
appoV = find(pCFex > 0); apposcDv = find(pCFex < 0);
noActv = appoD; noActv = [noActv , 1000];
Actv = 1:nveicoli; Actv(appoD) = []; 
DpCFex = pCFmx(appoV)-pCFex(appoV);
stDv = std(DpCFex);
DpCFstDv = randn(1,size(apposcDv,2))*stDv;
pCFex(apposcDv) = pCFmx(apposcDv)+DpCFstDv;

%Scaling the maximum experimental BC pressure referring to 3.8 bar
pCFexTfLC = pCFex;


% Defining active CV among instrumented vehicle
NVhcI = size(VhcM,2);
kk = 0;
for ii = 1:NVhcI
    appoP = find(Actv == VhcM(ii));
    if appoP
        kk = kk+1;
        VhcMa(kk) = appoP;
        iDCVa(kk) = ii;
    end;
end;

% Transfer function data
data = 'TransferFunction';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Instruction read only by MIDEVA.
   load (nomefile,'A');
else
   % Instruction read only by MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
rc = 1;
NpBk = A(rc,1); % Number of the points for the braking transfer function
rc = rc+1;
NpRls = A(rc,1); % Number of the points for the releasing transfer function
rc = rc+1;
Np = max((NpBk+NpRls))+1;
dTF = zeros(nCV,Np,2);        
% dTF([:],:,:) : number of vehicle
% dTF(:,[:],:) : series of points to build transfer function before braking and after releasing data points
% dTF(:,:,[:]) : pCG pCF  starting from maximum pressure drop
tt = 1; kk = 1; %index to extract the transfer functions of active CV
DpAdj = (P1-6e5)/1e5; % this data contains the value to taking in account the 
                %adjustment on transfer function being refered to a nominal
                %starting pressure of 5 bar
for ii = 1:nveicoli
    if ii == noActv(kk)
        kk = kk+1;
        rc = rc+NpBk+NpRls;
    else
        tt = ii-(kk-1);
        for jj = 1:NpBk
            dTF(tt,jj,1:2) = A(rc,:);
            rc = rc+1;
        end;
        jj2 = jj+1;
        for jj = jj2:jj+NpRls
            dTF(tt,jj,1:2) = A(rc,:);
            rc = rc+1;
        end;  
        % data of transfer function Sorting (starting with data of maximum BC pressure)
        % Braking transfer function
        [rr,iS] = sort(dTF(tt,1:NpBk,1));
        dTF(tt,1:NpBk,:) = dTF(tt,iS,:);
        % Releasing transfer function
        [rr,iS] = sort(dTF(tt,NpBk+1:(NpBk+NpRls),1));
        dTF(tt,NpBk+1:(NpBk+NpRls),:) = dTF(tt,NpBk+iS,:); 
        
        % Adjustment of BP pressure in Transfer function taking in account
        % the real running pressure of manoeuvre
        dTF(tt,1:(NpBk+NpRls),1) = dTF(tt,1:(NpBk+NpRls),1)+DpAdj;
        
        dTF(tt,Np,:) = [NpBk,NpRls];
        dTF(tt,1,2) = pCFexTfLC(ii);
        dTF(tt,NpBk+1,2) = pCFexTfLC(ii);    
    end;
end;

% Limiting curve data
data = 'LimitingCurve';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Instruction read only by MIDEVA.
   load (nomefile,'A');
else
   % Instruction read only by MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;

%Data reading
dLC = zeros(14,nCV);
% dLC([:],:) : [Pactv, pCFAs, tAS, pCGAs, pCFIf, tIf, Cfb, tmx, Cfr, tmn]
% dLC(:,[:]) : munber of vehicle
dLC(1,:) = PbUPbc(Actv); t95p = zeros(1,nveicoli);
rc = 1;
tt = 1; kk = 1; %index to extract the limiting curve of active CV
for ii = 1:nveicoli 
    if ii == noActv(kk)
        kk = kk+1;
        rc = rc+6;
    else 
        tt = ii-(kk-1);
        DataAs = A(rc,:); DataAs(3) = (P1/1e5-1)-DataAs(3); 
        dLC(2:4,tt) = DataAs';
        rc = rc+1;
        DataIf = A(rc,1:2);
        dLC(5:6,tt) = DataIf';
        rc = rc+1;
        if VCi(1,ii) == 1
            dLC(2,tt) = dLC(2,tt)*VCi(2,ii);
            dLC(5,tt) = dLC(5,tt)*VCi(2,ii);
        end;
        timeBkP = A(rc,1:2);
        rc = rc+1;
        timeBkG = A(rc,1:2);
        rc = rc+1;
        timeRlsP = A(rc,1:2);
        rc = rc+1;
        timeRlsG = A(rc,1:2);   
        
        % Data braking liminting curve 
        tinh = dLC(3,tt)+dLC(6,tt); % end of inshot function %CC correzione per attualizzare tD
        p95p = 0.95*dTF(tt,1,2);
        pmx = dTF(tt,1,2);   
        
        % Data releasing liminting curve
        pmn = dTF(tt,NpBk+NpRls,2); 
        p110p = 1.1*pmn;        
        
        if RgBk(ii) == 1 %Passenger regime         
            % Braking  
            t95p(ii) = timeBkP(1); tmx = timeBkP(2);    
            [CfPb] = polyfit([tinh t95p(ii) tmx],[dLC(5,tt) p95p pmx],2);         
            % Releasing
            t110p = timeRlsP(1); tmn = timeRlsP(2);           
            [CfPr] = polyfit([0 t110p tmn],[pmx p110p pmn],2); 
            
            dLC(7:end,tt) = [CfPb'; tmx; CfPr'; tmn]; %coefficient of limiting curves (braking releasing) assigning
        else % Goods regime 
            % Braking 
            t95p(ii) = timeBkG(1); tmx = timeBkG(2);  
            [CfGb] = polyfit([tinh t95p(ii) tmx],[dLC(5,tt) p95p pmx],2);        
            % Releasing    
            t110p = timeRlsG(1); tmn = timeRlsG(2);     
            [CfGr] = polyfit([0 t110p tmn],[pmx p110p pmn],2); 
            
            dLC(7:end,tt) = [CfGb'; tmx; CfGr'; tmn]; %coefficient of limiting curves (braking releasing) assigning
        end;
        rc = rc+1;
    end;
end;

% Progressive position of control valve
% Transfer function vehicle number-progressive position along train
Lprg = [0 cumsum(LvagoC)];

if abs(CV_post(Actv)) < Lvago(Actv)*0.5
    gmisCV = Lprg(Actv)+0.5*Lcoll+0.5*Lvago(Actv)+CV_post(Actv);
    gmisCV(Actv == 1) = 0.5*Lvago(1)+CV_post(1);
    gmisCF = gmisCV;
else
    error('CV_post too long, it must be shorter than half length of the wagon');
end


% Initial condition of brake cylinder, auxiliary reservoir, acceleration
% chamber

% Initial pressure in brake cylinder, auxiliary reservoir
pCGc = dTF(:,1:NpBk,1); 
pCFc = dTF(:,1:NpBk,2); 
pCFmnv0 = 0; % I assume that in any case, before than first manouvre, the system start from running condition
Pbp0 = P1/1e5-1;
pAR = Prnar(Actv); % Pressure in AR considering running condition
RvolBcAr = RvolBcAr(Actv); % Redefining quantity only for active CV

for ii = 1:nCV  
    % Assignement pressure in brake cylinder    
    pCGa = pCGc(ii,:);
    pCFa = pCFc(ii,:);
    appo = find(pCGa > Pbp0);
    if appo
        ind = appo(1);
        if ind == 1
            pbrake0 = pCFa(1);
        else       
            m = (pCFa(ind-1)-pCFa(ind))/(pCGa(ind)-pCGa(ind-1));
            pbrake0 = m*(pCGa(ind)-Pbp0)+pCFa(ind);        
        end;                  
    else
        pbrake0 = 0;  
    end;    
    pCF(ii) = pbrake0; % initial pressure in BC
    
    % Assignement pressure in auxiliary reservoir
    if pbrake0  ~= 0
        pbrake0 = (pbrake0+1);
    end;
    
    %Initialization of pressure in AR               
    kpolSA = 1.3;
    pAR(ii) = pAR(ii)*(1-kpolSA*(RvolBcAr(ii)*((pbrake0-pCFmnv0)*1e5/pAR(ii))));
end;

Vbc = Vbc(Actv); % assignement variable for brake cylinder volume

% Data auxiliary reservoir
%matrix SA: position in m , volume , maximum equivalent SA diameter , dP imposed by ceck valve ,  
%P initial , P running
SA = [gmisCV; Var(Actv); Dar(Actv); dParbp(Actv); pAR; Prnar(Actv)];
    
% Data acceleration chamber
%matrix CA: position in m , volume , diameter , P activation , P initial ,
%P closing 
Piac = 1.01325e5*ones(1,nCV);
CA = [gmisCV; Vac(Actv); Dac(Actv); PactvCV(Actv); Piac; Pcac(Actv)]; 
standem = size(tandem,2);
if size(VV,2) > 0
    if standem > 0
        for ii = 1:standem
            % Set the diameter of the acceleration chamber of the tandem
            % vehicle to zero
            CA(3,tandem(ii)) = 0;
            CA(3,tandem(ii)+1) = 0;
        end
    end
    sca = size(CA,2);
    for ii = 1:sca
        if CA(3,ii) > 0 && VV(2,ii) > 0
            strerr = ['It is not possible to have on the vehicle ' num2str(ii) ...
                ' an UIC distributor and a QR distributor'];
            error(strerr);
        end
    end
    %iCA = find(CA(3,:) > 0);
    %CA = CA(:,iCA); %gmisCF = gmisCF(iCA); 
    %SA = SA(:,iCA);
    % The first row of the vent valve is changed. It will store the
    % position (in metres) of the vent valves
    if abs(VV(1,:)) < Lvago(Actv)*0.5
        gmisVV = Lprg(Actv)+0.5*Lcoll+Lvago(Actv)*0.5+VV(1,:);
        gmisVV(Actv == 1) = 0.5*Lvago(1)+VV(1,1);
        VV(1,:) = gmisVV;
    else
        error('VV_position too long, it must be shorter than half length of the wagon');
    end
end

%Input data distributor emulation END---------------------------------------------------------




%Redefining lenght of wagons START---------------------------------------------------------
function Lvago = ridefLvago(dx,Lcoll,Lvago,nveicoli)
%# scalar errlvag iv nveicoli 

Lvago2 = zeros(1,nveicoli);
%# fastindex
errlvag = 0;

%# fastindex
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
%Redefining lenght of wagons END---------------------------------------------------------


function [vmaster,tandem] = gen_master_tandem(pos_master_slave)

% This function computes the vectors vmaster and vslave from the input
vmaster = []; tandem = []; % Initializations
siz = size(pos_master_slave,2); ii = 1;
while ii <= siz
    if pos_master_slave(ii) == 1
        vmaster = [vmaster ii]; 
        if ii < siz && pos_master_slave(ii+1) == 0
            tandem = [tandem ii];
            ii = ii+1;
            while ii <= siz && pos_master_slave(ii) == 0
                ii = ii+1;
            end
        else
            ii = ii+1;
        end
    end
end

function [EPV,EPV_active]= dataEPB(infoEPB,Lcoll,Lvago,Lprg,Actv)

% definition of the EPV position
if abs(infoEPB(4,:)) < Lvago(Actv)*0.25
    gmisEpb = Lprg(Actv)+0.5*Lcoll+Lvago(Actv)*0.25+infoEPB(4,:); 
    gmisEpb(Actv == 1) = 0.25*Lvago(1)+infoEPB(4,1);
    gmisEV = gmisEpb;
else
    error('EpV_post too long, it must be shorter than 1/4 length of the wagon');
end
% Data Electrovalves
% matrix EPV: position in m , diameter , riferiment loco, delay 
EPV = [gmisEpb; infoEPB(1,:); infoEPB(2,:); infoEPB(3,:);gmisEV];
EPV_active = find(infoEPB(2,:)>-1);

function VV = inputVV(P1,VV)
global Tamb
% This function sets some more inputs for the vent valves
% Vent valve parameters
dBPVu = 36e-3; % Nozzle diameter among BP and the upper vent valve reservoir
dVuVd = 1.315e-3; % Nozzle diameter among the upper vent valve reservoir and down vent valve reservoir
dVuVdopen = 3.22e-3; % Nozzle diameter among the upper vent valve reservoir and down vent valve reservoir
Vu = 1e-3; % Volume of up reservoir
Vd = 1.27e-3; % Volume of down reservoir
Sru = pi*(118.25e-3)^2*0.25; % Section of up surface
% valore limite 117.46
Srd = pi*(113.17e-3)^2*0.25; % Section of down surface
Fspring = 6.38+1.21e-3*2967;% Force of the counteracting spring
k = 1.3; r = 287.5;
[Cpol1,Cpol2] = Cpoly(k,P1,r,Tamb,Vu);

VV(5,:) = pi*dBPVu^2*0.25; VV(6,:) = pi*dVuVd^2*0.25; VV(7,:) = pi*dVuVdopen^2*0.25; 
VV(8,:) = Vu; VV(9,:) = Vd; VV(10,:) = Sru; VV(11,:) = Srd; VV(12,:) = Fspring;
VV(13,:) = Cpol1; VV(14,:) = Cpol2;
[Cpol1,Cpol2] = Cpoly(k,P1,r,Tamb,Vd);
VV(15,:) = Cpol1; VV(16,:) = Cpol2; VV(17,:) = k;

function [Cpol1,Cpol2] = Cpoly(k,P0,r,T0,Vr)

ro0 = P0/(r*T0); % Starting density inside the reservoir
Cpol1 = P0/ro0^k;
Cpol2 = (P0/(Vr^k))*(r*T0/P0)^k;
