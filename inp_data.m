function [CA,CVactv,D1,DBVdata,Dcoll,dLC,dTF,dx,epsilon,gmis,gmisCF,kcoll,Lcoll,...
          Lvago,P1,pCF,SA,SR,train,Tsim,Vbc] = inp_data(namedirfile,dynamic,fsl,nveicoli,train)

%     [CA,CVactv,D1,DBVdata,Dcoll,dLC,~,dTF,dx,epsilon,~,gmis,gmisCF,~,kcoll,Lcoll,...
%         Lvago,~,~,P1,pCF,~,SA,SR,~,~,train,Tsim,~,~,Vbc] 
    
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
    cline = fgetl(nf);
    if strcmp(cline,'WINDOWS'), fsl = '\'; elseif strcmp(cline,'LINUX'), fsl = '/'; end
    nomefile = fscanf(nf,'%c',256);		   % read the initial 256 characters
    fclose(nf);                            % Close the file scambio.inf
    cr = find(nomefile == char(13));	   % Carriage return: identify the 'nomefile' individuo in 'nomefile' il primo
    % 'a capo'(char(13)) determino nomefile come il vecchio
    nomefile = nomefile(1:cr(1)-1);		   % Nomefile ma fino al suo primo 'a capo'

    %#RK-modified nomepath = [nomefile,'\']; % Set the working directory
    % nomepath = [nomefile,'/']; % Set the working directory
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
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;

%Data reading
%# fastindex
rc = 1;
SR = A(rc); % Sampling rate data writing
rc = rc+1;
%# fastindex
Dpy = A(rc); % variable to control the operation of real-time BP pressure display ( 1: enable ; 0: disable)
rc = rc+1;
Tamb = A(rc); %[K] 
rc = rc+1;
epsilon = A(rc)/1000; % roughness of brake pipe %0.046e-3 m Scabrezza acciaio commerciale
rc = rc+1;
Dcoll = A(rc)/1000; 
rc = rc+1;
Lcoll = A(rc);
rc = rc+1;
kcoll = A(rc);
rc = rc+1;
%# fastindex
CVactv = A(rc);
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
        % Istruzione letta solo dal MIDEVA.
        load (nomefile,'A');
    else
        %Istruzione letta solo dal MATLAB.
        load (nomefile,'A');
        A = eval(data);
    end;
    %Data reading
    rc = 1;
    VhcM = A(rc,:); % vehicles instrumented along the train (N.B. where the CV is active)
end

% BRAKE PIPE AND DISTRIBUTOR DATA
[CA,D1,dLC,dTF,gmis,gmisCF,iDCVa,Lcoll,Lvago,LvagoC,pCF,SA,t95p,train,Vbc,VhcMa] = dataBpCv(CVactv,dx,Lcoll,nveicoli,nomepath,P1,train,VhcM);

% MANOUVRE DATA
[DBVdata,Tsim,vManovra] = dataManouvre(dynamic,Lcoll,Lvago,LvagoC,nDBV,nManv,nomepath,P1,posDBV,typeDBV);


%Input data brake pipe STARTING---------------------------------------------------------
function [CA,D1,dLC,dTF,gmis,gmisCF,iDCVa,Lcoll,Lvago,LvagoC,pCF,SA,t95p,train,Vbc,VhcMa] = ...
    dataBpCv(CVactv,dx,Lcoll,nveicoli,nomepath,P1,train,VhcM)

% Brake pipe data calculation
%# scalar CVactv ii iv nveicoli

% OVERALL DATA BP
data = 'OverallDataBP';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
VhcM = 1:nveicoli; % Added for dynamics
rc = 1;
Lvago = A(rc,:);

% In input the length of brake pipe doesn't consist of hose couplings length too:
% Redefining Lvago taking in account couplings length
LvagoC = Lvago+Lcoll; LvagoC(1) = LvagoC(1)-0.5*Lcoll; LvagoC(end) = LvagoC(end)-0.5*Lcoll; %Length comprehensive of couplings



% Redefining Lvago rounding to the nearest integers non considering
% couplings
% taking in account the spatial discretization, the input length of hose
% couplings is rounded to the nearest integers towards minus infinity. 
%(This instructions are needed to calculate the effective lenght of train
% necessary to calculate the propagation velocity)
%Lcoll = round(Lcoll);
Lcoll = floor(Lcoll);
Lvago = ridefLvago(dx,Lcoll,LvagoC,nveicoli);

% Redefining LvagoC following the "discretization" criteria applied
% previously
LvagoC = Lvago+Lcoll; LvagoC(1) = LvagoC(1)-0.5*Lcoll; LvagoC(end) = LvagoC(end)-0.5*Lcoll; %Length comprehensive of couplings

%-------------------------------------------------------------------
% Transfer function vehicle number-progressive position along train
Lprg = [0 cumsum(LvagoC)];
VhcM = sort(VhcM);
gmis = Lprg(VhcM)+0.5*Lcoll+Lvago(VhcM)./2;
% gmis(find(VhcM == 1)) = Lvago(1)./2;
gmis(VhcM == 1) = Lvago(1)./2;
%-------------------------------------------------------------------

rc = rc+1;
D1 =  A(rc,:)/1000;

%# fastindex
if CVactv == 1
    % Calculation input data distributor
    [CA,dLC,dTF,gmisCF,iDCVa,pCF,SA,t95p,train,Vbc,VhcMa] = dataCVs(Lcoll,Lvago,LvagoC,nomepath,nveicoli,P1,train,VhcM); 
else
    SA = []; CA = []; pCF = []; dLC = []; dTF = []; VhcMa = []; iDCVa = []; gmisCF = []; Vbc = []; t95p = [];
end;
%Input data brake pipe END--------------------------------------------------------


%Input data distributor emulation STARTING---------------------------------------------------------
function [CA,dLC,dTF,gmisCF,iDCVa,pCF,SA,t95p,train,Vbc,VhcMa] = dataCVs(Lcoll,Lvago,LvagoC,nomepath,nveicoli,P1,train,VhcM)

% Control valve data calculation

% OVERALL DATA CV
data = 'OverallDataCV';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
rc = 1;
pCFmx = A(rc,:);
rc = rc+1;
pCFex = A(rc,:);
rc = rc+1;
%PbUPbc = A(rc,:);%((A(rc,:)+1)*1e5);
DPbUPbc = A(rc,:); PbUPbc = ((P1/1e5-1)-DPbUPbc);
rc = rc+1;  
RgBk = A(rc,:); % 1 viaggiatori 0 merci
rc = rc+1;
VCi =  A(rc,:); % variable that contain index to define type  and scaling ratio of Empty/load device
VCi(2,:) = [pCFmx./3.8]; % scaling of empty/load device
rc = rc+1;
dBC = ((A(rc,:)*25.4)/1000);
Sbc = (pi./4.*dBC.^2);
rc = rc+1;
Stkbc = (A(rc,:)/1000);
Vbc = Sbc.*Stkbc;
rc = rc+1;
Vac = (A(rc,:)/1000);
rc = rc+1;
Dac = (A(rc,:)/1000);
rc = rc+1;
%PactvCV = ((A(rc,:)+1)*1e5);
DPactvCV = A(rc,:)*1e5; PactvCV = (P1-DPactvCV);
rc = rc+1;  
Pcac = ((A(rc,:)+1)*1e5);
rc = rc+1;
Var =  (A(rc,:)/1000);
rc = rc+1;
Dar =  (A(rc,:)/1000);
rc = rc+1;
dParbp =  (A(rc,:)*1e5);
rc = rc+1;
Prnar = ((A(rc,:)+1)*1e5); % running pressure on auxiliary reservoir

for ii = 1:numel(pCFmx)
    train(ii).pBC = pCFmx(ii);
end

RvolBcAr = Vbc./Var;

% Defining active CV and Imposing maximum BC experimental data
% appoD = find(pCFex == 0); nCV = nveicoli-size(appoD,2); %assigmnet number
% of CV active %%CC
appoD = zeros(1,0); nCV = nveicoli; %assigmnet number of CV active %%CC
appoV = find(pCFex > 0); 
apposcDv = zeros(1,0); % Actually no auto-random pressure generation is still supported find(pCFex < 0);
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
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
rc = 1;
NpBk = A(rc,1);
rc = rc+1;
NpRls = A(rc,1);
rc = rc+1;
Np = max((NpBk+NpRls))+1;
dTF = zeros(nCV,Np,2);        
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
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
dLC = zeros(14,nCV);
% dLC([:],:) : [Pactv, pCFAs, tAS, pCGAs, pCFIf, tIf, Cfb, tmx, Cfr, tmn]
% dLC(:,[:]) : munber of vehicle
dLC(1,:) = PbUPbc(Actv);%(PbUPbc-1e5)./1e5;
dLC(1,pCFex==0) = 0; % Added to manage non active CVs properly
rc = 1;
tt = 1; kk = 1; %index to extract the limiting curve of active CV
for ii = 1:nveicoli 
    if ii == noActv(kk)
        kk = kk+1;
        rc = rc+6;
    else 
        tt = ii-(kk-1);
        DataAs = A(rc,:); DataAs(3) = (P1/1e5-1)-DataAs(3); 
        train(ii).pCFAs = A(rc,1); % Pressure application stroke
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
            % Releasing    v
            t110p = timeRlsG(1); tmn = timeRlsG(2);     
            [CfGr] = polyfit([0 t110p tmn],[pmx p110p pmn],2); 
            
            dLC(7:end,tt) = [CfGb'; tmx; CfGr'; tmn]; %coefficient of limiting curves (braking releasing) assigning
        end;
        rc = rc+1;
    end;
end;
% TODO: Check if the next row is actually essential.
dLC(2,pCFex==0) = 0; % Added to manage non active CVs properly

% Progressive position of control valve
% Transfer function vehicle number-progressive position along train
Lprg = [0 cumsum(LvagoC)];
gmisCV = Lprg(Actv)+0.5*Lcoll+Lvago(Actv)./2; 
gmisCV(find(Actv == 1)) = Lvago(1)./2;
gmisCF = gmisCV;
% Initial condition of brake cylinder, auxiliary reservoir, acceleration
% chamber

% Initial pressure in brake cylinder, auxiliary reservoir
pCGc = dTF(:,1:NpBk,1); 
pCFc = dTF(:,1:NpBk,2); 
pCFmnv0 = 0; % I assume that in any case, before than first manouvre, the system start from running condition
Pbp0 = P1/1e5-1;
pAR = Prnar(Actv); % Pressure in AR considering running condition
%pAR = 5.9e5*ones(1,nCV);
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
    %pbrake0 = 2.5;    
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
%Input data distributor emulation END---------------------------------------------------------


%Input data manouvre and driver's brake valve STARTING---------------------------------------------------------
function [DBVdata,Tsim,vManovra] = dataManouvre(dynamic,Lcoll,Lvago,LvagoC,nDBV,nManv,nomepath,P1,posDBV,typeDBV)

%# scalar Tsim

% DRIVER BRAKE VALVE DATA
data = 'DBVData';
nomefile = [nomepath,data,'.txt'];
if exist('_MATCOM_') ~= 0
   % Istruzione letta solo dal MIDEVA.
   load (nomefile,'A');
else
   %Istruzione letta solo dal MATLAB.
   load (nomefile,'A');
   A = eval(data);
end;
%Data reading
rc = 1;
NmbrDBV =  A(rc,:); % number of DBV
rc = rc+1;

% DBVd is a matrix that collects DBV data:
% (1,:) : emergency braking data EB
% (2,:) : service braking data SB
% (3,:) : releasing data R
DBVd = zeros(3*NmbrDBV,10); 
for ii = 1:NmbrDBV    
    TypeDBV(ii) = A(rc,:); % type of DBV
    rc = rc+1;    
    % EMERGENCY BRAKING DATA
    DBVd(3*(ii-1)+1,1) = A(rc)/1000; % Diameter of emergency braking [mm] 
    rc = rc+1;
    DBVd(3*(ii-1)+1,2) = A(rc); % Flow coefficient of orifice Cq [-] 
    rc = rc+1;
    % SERVICE BRAKING DATA
    DBVd(3*(ii-1)+2,1) = A(rc)/1000; % Diameter of equivalente orifice [mm]
    rc = rc+1;
    DBVd(3*(ii-1)+2,2) = A(rc); % Flow coefficient of orifice Cq [-] 
    rc = rc+1;
    DBVd(3*(ii-1)+2,3) = A(rc); % Time to achieve a drop of 1.5 bar [s]
    rc = rc+1;
    DBVd(3*(ii-1)+2,4) = A(rc); % Time of first lift increasing [s]
    rc = rc+1;
    % RELEASING DATA
    DBVd(3*(ii-1)+3,1) = A(rc)/1000; % Diameter of equivalente orifice [mm]
    rc = rc+1;
    DBVd(3*(ii-1)+3,2) = A(rc); % Flow coefficient of orifice Cq [-] 
    rc = rc+1;
    DBVd(3*(ii-1)+3,3) = A(rc); % Time to achieve a increasing of 1.5 bar [s]
    rc = rc+1;
end;

% DBVdata collects all data of DBV for the simulation starting from first: for each DBV you
% have 3 line: 1� rapid braking data, position, delay ; 2� service braking
% data 3� releasing data
DBVdata = zeros(3*nDBV,10);
%# fastindex
k = 0; %index to move inside the matrix  DBVdata
Lprg = [0 cumsum(LvagoC)];
for ii = 1:nDBV
%     rc = rc+1;    
%     typeDBV = A(rc,1); posDBV = A(rc,2); 
    % Progressive position of DBV (redefine position) 
    % Transfer function vehicle number-progressive position along train
    if posDBV(ii) >= 1
        %posDBV(ii) = Lprg(posDBV(ii))+LvagoC(posDBV(ii))./4; 
        posDBV(ii) = Lprg(posDBV(ii))+0.5*Lcoll+Lvago(posDBV(ii))./4;
    elseif posDBV(ii) == 1
         posDBV(ii) = Lvago(posDBV(ii))./4;
    end;  
    % Assignement matrix DBV in simulation
    DBVd(3*(typeDBV(ii)-1)+1,3) = posDBV(ii);    
    DBVdata(3*k+1:3*k+3,:) = DBVd(3*(typeDBV(ii)-1)+1:3*(typeDBV(ii)-1)+3,:);
    k = k+1;    
end;

if not(isempty(dynamic))
    vManovra = zeros(nManv,3);
else
    % MANOUVRE DATA
    data = 'ManoeuvreData';
    nomefile = [nomepath,data,'.txt'];
    if exist('_MATCOM_') ~= 0
        % Istruzione letta solo dal MIDEVA.
        load (nomefile,'A');
    else
        %Istruzione letta solo dal MATLAB.
        load (nomefile,'A');
        A = eval(data);
    end;
    % vManovra is a matrix that stores information about manoeuvre; for each manoeuvre,
    % it stores the type, target pressure, the time (duration) of the
    % manoeuvre. A zero is used in pressure fields to avoid pressure
    % initialization (you will find zeros on the second colums from the second row
    % untill the last).
    % type = -1 : repid braking type = -2 : service braking
    % type = 1 : repid braking type = -2 : service braking

    %Data reading
    rc = 1;
    j = 4; %index to to move inside coloumns  of the matrix  DBVdata
    for ii = 1:nManv
        if ii == 1
            Pst = P1;
        else
            Pst = vManovra(ii-1,2);
        end;
        vManovra(ii,2) = (A(rc,1)+1)*1e5; % target pressure of the  manoeuvre
        Pfin = vManovra(ii,2);
        %rc = rc+1;
        vManovra(ii,3) = A(rc,2); % duration of the manoeuvre

        % type manouvre assignement
        dPMnv = Pst - Pfin;
        if dPMnv < 0 % Releasing manouvre
            if Pst == 1e5;
                vManovra(ii,1) = 1;
            else
                vManovra(ii,1) = 2;
            end;
        elseif  dPMnv > 0  % Braking manouvre
            if Pfin == 1e5;
                vManovra(ii,1) = -1;
            else
                vManovra(ii,1) = -2;
            end;
        else % DBV in running or acceleration
            vManovra(ii,1) = 0;
        end;

        % Defining delay of each  DBV for each manouvre
        k = 0; %index to move inside rows of the matrix  DBVdata
        for pp = 1:nDBV
            DBVdata(3*k+1,j) = A(rc,2+pp);
            k = k+1;
        end;
        j = j+1;
        rc = rc+1;
    end;
end

Tsim = sum(vManovra(:,3)); % Tempo da simulare in secondi
%Input data manouvre and driver's brake valve END---------------------------------------------------------


%Redefining lenght of wagons START---------------------------------------------------------
function Lvago = ridefLvago(dx,Lcoll,Lvago,nveicoli)
%# scalar errlvag iv nveicoli 

Lvago2 = zeros(1,nveicoli);
%# fastindex
errlvag = 0;

%# fastindex
iv = 0;

%# fastindex
for iv = 1:nveicoli
    if (iv == 1) | (iv == nveicoli)
        lvag = Lvago(iv)-0.5*Lcoll;
    else
        lvag = Lvago(iv)-Lcoll;
    end;
    Lvago2(iv) = round((errlvag+lvag)/dx);
    errlvag = (errlvag+lvag)-Lvago2(iv);
    %errlvag = lvag-Lvago2(iv);
end
Lvago2(nveicoli) = Lvago2(nveicoli)+round(errlvag/dx);
Lvago = Lvago2;
%Redefining lenght of wagons END---------------------------------------------------------