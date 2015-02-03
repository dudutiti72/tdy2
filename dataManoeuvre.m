%Input data manouvre and driver's brake valve STARTING---------------------------------------------------------
function [DBVdata,Tsim,vManovra] = dataManoeuvre(D1,dynamic,Lcoll,Lvago,LvagoC,nDBV,nManv,nomepath,P1,posDBV,typeDBV)

%# scalar Tsim

% DRIVER BRAKE VALVE DATA
data = ('DBVData');
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
NmbrDBV =  A(rc,:); % number of DBV
rc = rc+1;

% DBVd is a matrix that collects DBV data:
% (1,:) : emergency braking data EB
% (2,:) : service braking data SB
% (3,:) : releasing data R
% (4,:) : EP data
DBVd = zeros(4*NmbrDBV,10);
for ii = 1:NmbrDBV
    TypeDBV(ii) = A(rc,:); % type of DBV
    rc = rc+1;
    % EMERGENCY BRAKING DATA
    DBVd(4*(ii-1)+1,1) = A(rc)/1000; % Diameter of emergency braking [mm]
    rc = rc+1;
    DBVd(4*(ii-1)+1,2) = A(rc); % Flow coefficient of orifice Cq [-]
    rc = rc+1;
    % SERVICE BRAKING DATA
    DBVd(4*(ii-1)+2,1) = A(rc)/1000; % Diameter of equivalente orifice [mm]
    rc = rc+1;
    DBVd(4*(ii-1)+2,2) = A(rc); % Flow coefficient of orifice Cq [-]
    rc = rc+1;
    DBVd(4*(ii-1)+2,3) = A(rc); % Time to achieve a drop of 1.5 bar [s]
    rc = rc+1;
    DBVd(4*(ii-1)+2,4) = A(rc); % Time of first lift increasing [s]
    rc = rc+1;
    % RELEASING DATA
    DBVd(4*(ii-1)+3,1) = A(rc)/1000; % Diameter of equivalente orifice [mm]
    rc = rc+1;
    DBVd(4*(ii-1)+3,2) = A(rc); % Flow coefficient of orifice Cq [-]
    rc = rc+1;
    DBVd(4*(ii-1)+3,3) = A(rc); % Time to achieve a increasing of 1.5 bar [s]
    rc = rc+1;
    % ELECTROPNEUMATIC BRAKING DATA
    nf = fopen(nomefile,'r');
    cline = trova_info('% ELECTROPNEUMATIC BRAKING',nf);
    if not(isempty(cline))
        % The train is equipped with one or more EP brake
        cline = fgetl(nf);
        DBVd(4*(ii-1)+4,1) = interpolEV(nomepath,Lvago(posDBV(ii)),1,D1(posDBV(ii)),(str2num(fgetl(nf)))); % Diameter of equivalente orifice [mm]
        cline = fgetl(nf);
        DBVd(4*(ii-1)+4,2) = str2num(fgetl(nf)); % Flow coefficient of orifice Cq [-]
        cline = fgetl(nf);
        DBVd(4*(ii-1)+4,3) = str2num(fgetl(nf)); % Time to achieve a drop of 1.5 bar [s]
        cline = fgetl(nf);
        DBVd(4*(ii-1)+4,4) = str2num(fgetl(nf)); % Time of first lift increasing [s]
    else
        DBVd(4*(ii-1)+4,1) = 0; % Diameter of equivalente orifice [mm]
        DBVd(4*(ii-1)+4,2) = 0; % Flow coefficient of orifice Cq [-]
        DBVd(4*(ii-1)+4,3) = 0; % Time to achieve a drop of 1.5 bar [s]
        DBVd(4*(ii-1)+4,4) = 0; % Time of first lift increasing [s]
    end
    fclose(nf);
end;

% DBVdata collects all data of DBV for the simulation starting from first: for each DBV you
% have 4 line: 1° rapid braking data, position, delay ; 2° service braking
% data 3° releasing data 4° electropneumatic data
DBVdata = zeros(4*nDBV,10);
k = 0; %index to move inside the matrix  DBVdata
Lprg = [0 cumsum(LvagoC)];
for ii = 1:nDBV
    % Progressive position of DBV (redefine position)
    % Transfer function vehicle number-progressive position along train
    if posDBV(ii) >= 1
        posDBV(ii) = Lprg(posDBV(ii))+0.5*Lcoll+Lvago(posDBV(ii))./4;
    elseif posDBV(ii) == 1
        posDBV(ii) = Lvago(posDBV(ii))./4;
    end;
    % Assignement matrix DBV in simulation
    DBVd(4*(typeDBV(ii)-1)+1,3) = posDBV(ii);
    DBVdata(4*k+1:4*k+4,:) = DBVd(4*(typeDBV(ii)-1)+1:4*(typeDBV(ii)-1)+4,:);
    k = k+1;
end;
vManovra = zeros(nManv,3);
if isempty(dynamic)
    % MANOUVRE DATA
    data = 'ManoeuvreData';
    nomefile = [nomepath,data,'.txt'];
    if exist('_MATCOM_') ~= 0
        % Instruction read only by MIDEVA.
        load (nomefile,'A');
    else
        % Instruction read only by MATLAB.
        load (nomefile,'A');
        A = eval(data);
        sizA = size(A,2);
        if sizA >= 4
            index = 1;
        else
            index = 0;
        end
    end;
    % vManovra is a matrix that stores information about manoeuvre; for each manoeuvre,
    % it stores the type, target pressure, the time (duration) of the
    % manoeuvre. A zero is used in pressure fields to avoid pressure
    % initialization (you will find zeros on the second colums from the second row
    % untill the last).
    % type = -1 : repid braking type = -2 : service braking type = -3 :
    % electropneumatic brake type = -4 : emergency braking with
    % electropneumatic brake
    % type = 1 : repid braking type = 2 : service braking
    
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
        vManovra(ii,3) = A(rc,2+index); % duration of the manoeuvre
        
        % type manouvre assignement
        dPMnv = Pst - Pfin;
        if dPMnv < 0 % Releasing manouvre
            if Pst == 1e5;
                vManovra(ii,1) = 1; % Releasing from emergency brake
            else
                vManovra(ii,1) = 2; % Releasing from service or electropneumatic brake
            end;
        elseif  dPMnv > 0  % Braking manouvre
            if Pfin == 1e5 && (sizA >4 && A(rc,2) == 0)
                vManovra(ii,1) = -1; % Emergency brake
            elseif Pfin == 1e5 && (sizA >4 && A(rc,2) == 1)
                vManovra(ii,1) = -4; % Emergency brake and electropneumatic
            elseif sizA >4 && A(rc,2) == 1
                vManovra(ii,1) = -3; % Electropneumatic brake
            else
                vManovra(ii,1) = -2; % Service brake
            end;
        else % DBV in running or acceleration
            vManovra(ii,1) = 0;
        end;
        
        % Defining delay of each  DBV for each manouvre
        k = 0; %index to move inside rows of the matrix  DBVdata
        for pp = 1:nDBV
            DBVdata(4*k+1,j) = A(rc,2+index+pp);
            k = k+1;
        end;
        j = j+1;
        rc = rc+1;
    end;
end

Tsim = sum(vManovra(:,3)); % Tempo da simulare in secondi
%Input data manouvre and driver's brake valve END---------------------------------------------------------
