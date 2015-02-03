function integraode(appf,bgdg,indexes,Mvt,Pbrake,Ppres,shw,strack,swcf,Tbrake,traccia,train,vel_0,...
    CA,CVactv,D,DBVdata,dLC,DT,dt,dTF,dQdto,d2Qdt2o,drodto,d2rodt2o,dudto,d2udt2o,dx,EOT,FOT,...
    iCV,iV,j0,j1,jj,K,Lmedia,loco,n,nCV,nDBV,P1,pCF,rugrel,segnoCA,segnoSA,...
    SA,solver,SR,typeDBV,Vbc,vmisCF,vUnC)

% INTEGRAODE This function performs the numerical integration of the 
%            longitudinal dynamics problem. It consists the core of the
%            TrainDy computations                                           [f] integraode1 no longer used
%
% INPUTS     appf:     String appended to characterize test
%            bgdg:     Struct array with coupling data (Buffers,Draw gears)
%            indexes:  Struct array with position of wagons equipped
%                      with disk or block brake system
%            Mvt:      Vector with total mass of every vehicle including
%                      rotary masses [kg]
%            Pbrake:   Not used
%            Ppres:    Not used
%            shw:      Show or not the LF graph. (Also activates the output
%                      fcn of the solver that calculates the pneumatics
%                      in every timestep. So do not set to 0)               
%            strack:   Struct array with track info
%            swcf:     Not used
%            Tbrake:   Not used
%            traccia:  Array with track info
%            train:    Struct array with info about each vehicle
%            vel_0:    Initial speed. 2nd column not supported by 
%                      GUI [km\h]                                           [n]
%            CA:       Matrix with Acceleration Chamber data
%            CVactv:   Scalar with status of Control Valves. Set to 1 when
%                      there is at least one active Control Valve in the
%                      train
%            D:        Array with diameters of all pipe sections [m]
%            DBVdata:  Array with Driver's Brake Valve data
%            dLC:      Array with limiting curve data of Control Valves. It
%                      has as many columns as vehicles. See PneumDevices.m 
%                      for details
%            DT:       Maximum time step at the beginning of integration 
%                      (actually 10E-6) [s]
%            dt:       Actual time step (both DT and dt are changing during
%                      the simulation) [s]
%            dTF:      Array with transfer function data of Control Valves.
%                      It has as many rows as vehicles. See PneumDevices.m
%            dQdto:    Initial condition for partial derivative of specific 
%                      energy wrt time
%            d2Qdt2o:  Initial condition for second partial derivative of
%                      specific energy wrt time
%            drodto:   Initial condition for partial deriavtive of density
%                      wrt time
%            d2rodt2o: Initial condition for second partial derivative of   
%                      density wrt time
%            dudto:    Initial condition for partial deriavtive of flow 
%                      speed wrt time
%            d2udt2o:  Initial condition for second partial deriavtive of 
%                      flow speed wrt time
%            dx:       Discretization of Brake Pipe [m]
%            EOT:      End of train (changes the venting device to the end 
%                      of vehicle's Brake Pipe). Not supported in GUI       [n]
%            FOT:      Front of train
%            iCV:      Index with section ids where Control Valves are 
%                      connected to the Brake Pipe
%            iV:       Index with section ids where Driver's Brake Valves 
%                      are connected to the Brake Pipe. Array with 3 rows,
%                      1st: section id, right boundary of flow modification
%                      2nd: section id-1,left boundary of flow modification
%                      3rd: Only for EOT or FOT
%            j0:       Manages section ids in pneumatic solver
%            j1:       Manages section ids in pneumatic solver
%            jj:       Manages section ids in pneumatic solver
%            K:        Vector with concentrated pressure losses
%            Lmedia:   Two times dx, used for the derivative approximations
%                      in comp_pressure_tpSR                                
%            loco:     Struct array with locomotive data
%            n:        Number of Brake Pipe sections
%            nCV:      Number of Control Valves
%            nDBV:     Number of Driver's Brake Valves
%            P1:       Initial absolute Brake Pipe pressure [Pa]
%            pCF:      Array with relative Brake Cylinder pressure at
%                      current time step. Initialized to zeros as an input
%                      to this function since train is assumed to be in "on
%                      run" condition [bar]
%            rugrel:   Relative roughness of Brake Pipe
%            segnoCA:  Sign to indicate direction of air flow between Brake
%                      Pipe and Accelaration Chamber
%            segnoSA:  Sign to indicate direction of air flow between 
%                      Auxiliary Reservoir and Brake Pipe
%            SA:       Structure to manage Auxiliary Reservoir data (only 
%                      for releasing)                                       
%            solver:   Type of matlab differential equation solver
%            SR:       Sampling rate [s]
%            typeDBV:  Vector with Driver's Brake Valve types, 
%                       1 = Lateral DBV
%                       0 = DBV at front of pipe
%                      -1 = DBV at end   of pipe
%            Vbc:      Vector with volume of Brake Cylinders (releasing)
%                      [m^3]
%            vmisCF:   Vector with section ids where Brake Cylinders are 
%                      connected to the Brake Pipe
%            vUnC:     Vector with section ids where Brake Pipe is not
%                      coupled
%
% OUTPUT     None

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global mCA prCA                         % used in comp_CA & initialManv
global mSA prSA                         % used in comp_SA
global cv lam r s Tamb viscosita        % generic constant
global cRch1 cRch2                      % used in recharge
global ASph eftPt inpt

% global pCF_0 %used in CalcCF & initialManv

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

appftxt = [appf,'.txt'];
appfmat = [appf,'.mat'];

% Initialization of variables used during the numerical integration
[CCtrac,Fel,Feld,Fels,Ffren,fitrac,frico,ilow,invM,ipos,iup,lwagcs,N,...
    onv,options,ptb,pesoz,posfc,ssp,train,vel_0,xmaxct,y0,znv] = ... 
    IniNumInt(bgdg,Mvt,solver,train,vel_0);

% Updating of the struct options used by the numerical solver
if shw == 1
    h100    = figure(100);
    options = odeset(options,'outputfcn',@odeplotL);
    tic
end

% If the pneumatics must be computed along wth the dynamics, the events 
% function is provided
if isempty(swcf), options = odeset(options,'Events',@events); end

% PNEUMATIC PART
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty(swcf)
    
    % Initialization of parameters
    u1 = 0; 
    u  = u1*ones(1,n);
    P  = P1*ones(1,n); % Initial absolute Brake Pipe pressure 
    Pn = P;
    T  = Tamb*ones(1,n);
    ro = P./(r*T);
    S  = 0.25*D.^2*pi;
    Q  = cv*T+0.5*u.^2;
    
    % Initialization of parameters for Brake Cylinder pressure calculation 
    
    % Acceleration Chambers
    prCA   = CA(5,:);                    
    mCA    = (CA(2,:).*prCA) / Tamb / r; % Air mass inside chambers [SBB]
    
    % Auxiliary Reservoirs
    prSA   = SA(5,:);
    mSA    = (SA(2,:).*prSA) / Tamb / r;
    P0     = prSA; 
    T0     = Tamb;
    kpolSA = 1.3; 
    cstPol = T0*P0.^((1-kpolSA) / kpolSA); 
    
    % Assignement to consider pressure in BC refered to nominal 3.8 bar
    pCFm   = pCF;
    
    % Variables for the pressure calculation in Auxiliary Reservoirs during
    % braking [SBB] (translation)                                          
    % Termini per calcolo pressioni in SA durante frenatura
    pCFa       = pCF;
    appo       = find(pCFa ~= 0);
    pCFa(appo) = pCFa(appo)+1;
    pCFmnv0    = pCFa;
    
    % Initialization of BP pressure close to Control Valve
    pCdG_0 = (0.25*(Pn(vmisCF(1:end)+1)+Pn(vmisCF(1:end)-1) + 2*Pn(vmisCF(1:end))))./1e5-1;
    
    % Variable to control the flow through Driver's Brake Valve
    prpi  = P1*ones(1,nDBV); % Starting value of external back pressure to calculate mass flow through DBV
    Pmed  = P1*ones(1,nDBV); % Starting value of BP pressure to calculate mass flow rate through DBV (BP pressure close to DBV)
    Pmedo = P1*ones(1,nDBV); % Pressure at time t-dt to calculate gradient of BP raising during releasing
    dmf   = zeros(1,nDBV);   % Mass flow rate through DBV
    
    jm    = 0;               % Index to manage the different manoeuvres
    
    pBCt  = [-SR,znv']; pBCtpSR  = [0,znv']; 
    PBP   = (1e-5*P1-1)*onv; UBP = zeros(size(PBP));
    
    % PneumaticData is a txt file that stores relevant pneumatic info:
    % Pressure in BCs, Pressure in BP, Air Speed in BP, Mass flow throught 
    % the DBVs
    
    %nfP = fopen('PneumaticData.txt','w');
    nfP = fopen(['PneumaticData',appftxt],'w');
    formP = '%5.2f';
    for iP = 1:3*N-1 + nDBV              
        formP = [formP '\t%6.3f'];
    end
    formP = [formP '\t%6.3f\r\n'];
else
    jm = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NEW MANOEUVRE INPUT
nloco = length(loco);
znl   = zeros(nloco,1);

% Variables used to manage more than one locomotive
% isservice: Variable that indicates which type of pneumatic manoeuvre is
%            performed 
% ploco:     Vector with positions of locomotives
% tlocos:    Local time of the loco, to manage the behaviour of the loco 
%            that is time dependent [?]

isservice = znl; ploco = znl; tlocos = znl';

for iloc = 1:nloco
    ploco(iloc) = loco(iloc).ploco;
end
if isempty(swcf)
    vjm    = znl; % Counter of the manoeuvres performed by each loco
    jmloc  = vjm;
    vcond  = vjm; % Exit conditions for each loco
    % condup  is used to manage locos that have finished their manoeuvre
    condup = znl;
    nManv  = 0; % Maximum number of manoeuvres to be performed
    
    for iloc = 1:nloco
        nManv = nManv + loco(iloc).nm;
    end
    
    % Matrix with minimum and maximum time for each manoeuvre
    tmm = zeros(nloco,2);
    % Struct that will contain information on the Manoeuvres: mainly used in visres
    Mano(1:nManv+1) = struct('pn',0);
else
    nManv = 1;
end

while jm <= nManv
    % Some initializations
    Ffren = znv'; pn = znl; trac = pn'; eldyn = pn'; elpn = pn';
    if jm == 0, Feldyn = znl; Ftrac = znl; end
    if isempty(swcf)
        pfin = znl';
        
        % Updating of the indices that represent the manoeuvre
        for iloc = 1:nloco
            if vjm(iloc) < loco(iloc).nm
                jmloc(iloc) = vjm(iloc)+1;
            else
                jmloc(iloc) = vjm(iloc);
            end
            if jmloc(iloc) > 1 && (loco(iloc).man(jmloc(iloc),10) ~= loco(iloc).man(jmloc(iloc)-1,10))
                loco(iloc).tg = sol.x(end); % Updating the time for the gradient in loco operation
            end
            pn(iloc)    = loco(iloc).pn(jmloc(iloc));           % .pn is status of pneumatic brake in every submanoeuvre of the loco (nm elements) [SBB]
            elpn(iloc)  = loco(iloc).elettropn(jmloc(iloc));    % Same for electro-pneumatic brake
            eldyn(iloc) = loco(iloc).elettrodyn(jmloc(iloc));   %          electro-dynamic brake
            trac(iloc)  = loco(iloc).trac(jmloc(iloc));         %          traction
            
            % Updating the delays of the sub manoeuvre
            loco(iloc).eltts = loco(iloc).man(jmloc(iloc),9);
            loco(iloc).elbts = loco(iloc).man(jmloc(iloc),9);
            
            % Updating the exit conditions
            if loco(iloc).man(jmloc(iloc),3) > -1
                vcond(iloc) = 1; % the manoeuvre is controlled by the time
                if jm == 0, t0 = 0; else t0 = sol.x(end); end
                if (condup(iloc) > 0) || jm == 0
                    if vjm(iloc) < loco(iloc).nm
                        % Updating the initial time of the loco
                        % TODO: Check if it is possible to delete the
                        % variable tlocos
                        if jmloc(iloc) == 1 || jmloc(iloc) > 1 && ...
                                ( (loco(iloc).trac(jmloc(iloc)-1) ~= 1 && loco(iloc).trac(jmloc(iloc)) == 1) ...
                                || (loco(iloc).elettrodyn(jmloc(iloc)-1) ~= 1 && loco(iloc).elettrodyn(jmloc(iloc)) == 1) )
                            loco(iloc).ts = t0; tlocos(iloc) = t0;
                        end
                        tmm(iloc,:) = t0+[0 loco(iloc).man(jmloc(iloc),3)];
                    else
                        % The time 3e2 for a sub-manoeuvre can be put in input as
                        % "advanced input"
                        tmm(iloc,:) = t0+[0 3e2];
                        loco(iloc).man(end,3) = tmm(iloc,2);
                    end
                end
            else                                                            
                if loco(iloc).man(jmloc(iloc),4) > -1
                    vcond(iloc) = 2; % the manoeuvre is controlled by the distance
                    % Updating the initial distance of the loco: position of the loco
                    % at the beginning of the sub-manoeuvre
                    loco(iloc).ds = y0(1);
                elseif loco(iloc).man(jmloc(iloc),5) > -1
                    % the manoeuvre is controlled by the speed
                    if y0(N+1) > loco(iloc).man(jmloc(iloc),5), vcond(iloc) = -3; else vcond(iloc) = 3; end;
                elseif loco(iloc).man(jmloc(iloc),6) > -1
                    % the manoeuvre is controlled by the pressure in BP
                    if PBP(loco(iloc).man(jmloc(iloc),8)) > loco(iloc).man(jmloc(iloc),6), vcond(iloc) = -4; else vcond(iloc) = 4; end;
                elseif loco(iloc).man(jmloc(iloc),7) > -1
                    % the manoeuvre is controlled by the pressure in BC
                    if pBCt(1+loco(iloc).man(jmloc(iloc),8)) > loco(iloc).man(jmloc(iloc),7), vcond(iloc) = -5; else vcond(iloc) = 5; end;
                elseif sum(loco(iloc).man(jmloc(iloc),2:8)) == -7
                    if loco(iloc).man(jmloc(iloc),1) >= 1000
                        % A traction must be performed until a zero traction force
                        vcond(iloc) = -6;
                    elseif loco(iloc).man(jmloc(iloc),1) >= 100
                        % An electro dynamic braking must be performed until a zero
                        % force is applied
                        vcond(iloc) = -7;
                    end
                end
                % In this cases tspan is set conventionally
                if jm == 0, t0 = 0; else t0 = sol.x(end); end
                tmm(iloc,:) = [t0 t0+3e2];
            end
            % Initialization
            loco(iloc).Feldyn = Feldyn(iloc); 
            loco(iloc).Ftrac  = Ftrac(iloc);
            % Updating of the indices that collect the locos that brake pneumatically
            if loco(iloc).man(jmloc(iloc),2) > -1
                % This means that loco iloc in its manouevre jmloc(iloc) is braking with
                % pneumatics
                pfin(iloc) = (1+loco(iloc).man(jmloc(iloc),2))*1e5;
            end
            vDBV = find(pfin);
        end
        trac  = find(trac); eldyn = find(eldyn);
        tspan = max(tmm(:,1)):SR:min(tmm(:,2));
    else
        tspan  = Tbrake; pn = 1;
        % Initialization
        Feldyn = znl; Ftrac = znl; trac = []; eldyn = [];
    end
    
    if isempty(swcf) && sum(pfin) > 0
        % Redefining parameter of DBV and CV following the actual manouvre
        cRch1 = 0; cRch2 = 0;
        
        if tspan(1) > 0 && any(isservice)
            % This means that a sub-manoeuvre has previously run, so it is 
            % necessary to read the pneumatic data
            load(['Pneu_Dyn',appfmat],'dmf','d2Qdt2o','dQdto','d2rodt2o',...
                'drodto','dt','d2udt2o','dudto','DT','grapi','mSA',...
                'P','pBCtpSR','pCdG_0','pCF','pCFm','pCFmnv0','Pmed',...
                'Pmedo','prpi','prSA','ptarget','Q','ro','T','tBkon','u');
            Pn = P;
        end
        
        % Updating parameters following new manoeuvre
        for iloc = find(pfin)
            if jm == 0,
                Pst = P1;
            else
                if typeDBV(iloc) ~= 1 % in case of front or rear device
                    Pst = P(iV(3,iloc));
                else % in case of lateral device
                    Pst = mean([P(iV(1,iloc)) P(iV(2,iloc))]);
                end
            end      
            
            % % % % % % % % % % % % % %  isservice flag % % % % % % % % % % 
            %                                                             %
            %    0: Added for dynamics [?]                                %
            %  1,2: Releasing                                             %
            %   -1: Emergency brake (Pneumatic)                           %
            %   -5: Emergency brake (Electropneumatic) [n]                %
            %   -2: Service brake   (Pneumatic)                           %
            %   -3: Service brake   (Electropneumatic) [n]                %
            %                                                             %
            % % % % % % % % % % % % % % % % % % % % % % % % [SBB] % % % % %
            
            % type manoeuvre assignement
            dPMnv = Pst - pfin(iloc);
            if dPMnv < -0.1e5 %0 % Releasing manouvre
                if Pst == 1e5, isservice(iloc) = 1; else isservice(iloc) = 2;end;
            elseif  dPMnv > 0  % Braking manouvre
                if pfin(iloc) == 1e5 && elpn(iloc) == 0
                    isservice(iloc) = -1;
                elseif pfin(iloc) == 1e5 && elpn(iloc) == 1
                    isservice(iloc) = -4;
                elseif elpn(iloc) > 0
                    isservice(iloc) = -3;
                else
                    isservice(iloc) = -2;
                end
                % if pfin(iloc) == 1e5, isservice(iloc) = -1; else isservice(iloc) = -2; end;
                
            end
        end
        [Cfq,grapi,prpi,ptarget,segnof,Sut,t1] = initialManvBP(DBVdata,isservice,pfin,...
            prpi,typeDBV);
        if not(exist('tBkon','var'))
            tBkon = zeros(1,nCV);
            eftPt = zeros(1,nCV);
            ASph  = zeros(1,nCV);
            inpt  = ones(1,nCV);
        end
        [mCA,prCA,tBkon] = initialManvBC(CA,CVactv,isservice,nCV,pCF, ...
            Pn,r,Tamb,tBkon);
    end
    
    strode = [solver,'(@longdyn,tspan,y0,options);'];
    if jm == 0
        sol = eval(strode);
    else
        sol2 = eval(strode);
        % Updating struct array
        sol.x = [sol.x sol2.x(2:end)];
        sol.y = [sol.y sol2.y(:,2:end)];
    end
    % Manouevre-indices updating
    UpManInd
    
end
if isempty(swcf)
    while fclose(nfP); end
end

    function dy = longdyn(t,y)
        
    % LONGDYN The function to be integrated by the solver. First half of y
    %         vector contains the displacement of each wagon from its
    %         initial position (0) and the second half the velocities.
    %         Therefore, dy contains the velocities and accelerations
        
        xr    = -diff(y(1:N));      % Relative approach of the vehicles
        vr    = -diff(y(N+1:2*N));  % Relative speed of the vehicles
        Ffren = znv;
        
        if any(pn)
            % Pneumatic braking is active.
            if not(isempty(swcf))
                % The usage of a pre compiled file for pressure in Brake Cylinders is
                % not supported by the GUI
                front = 0;
                while t > Tbrake(ptb+1,1)
                    ptb = ptb+1; front = 1;
                end
                while front == 0 && t < Tbrake(ptb,1)
                    ptb = ptb-1;
                end
                tb = t - Tbrake(ptb,1);
                if swcf
                    pbrake_0 = Ppres(2+(ptb-1)*4,:)*tb^3+Ppres(3+(ptb-1)*4,:)*tb^2+...
                        Ppres(4+(ptb-1)*4,:)*tb+Ppres(5+(ptb-1)*4,:);
                    [Ffren,frico,ilow,iup,posfc] = brakeforce(Ffren,frico,ilow,iup,indexes,N,pbrake_0,posfc,train,y,vel_0);
                else
                    Ffren = Pbrake(2+(ptb-1)*4,:)*(tb^3)+Pbrake(3+(ptb-1)*4,:)*(tb^2)+...
                        Pbrake(4+(ptb-1)*4,:)*tb+Pbrake(5+(ptb-1)*4,:);
                end
            else
                % Pneumatics and dynamics are computed together
                pbrake_0 = pBCt(2:end) + (pBCtpSR(2:end)-pBCt(2:end))/(pBCtpSR(1)-pBCt(1))*(t-pBCt(1));
                % pbrake_0 stores the pressure in Brake Cylinder at the actual
                % integration time
                [Ffren,frico,ilow,iup,posfc,train] = brakeforce(Ffren,frico,ilow,iup,indexes,N,pbrake_0,posfc,train,y,vel_0);
            end
        end
        
        if any(elpn)
            % Electropneumatic brake not yet supported [n]
            Felpn = znv';
            Ffren = Ffren + Felpn;
        end
        
        % Initializations of loco forces
        % brake_eldyn and trac_eldyn do need Feldyn and Ftrac as inputs since the
        % calculations are not based on previous force values
        Feldyn = znl;
        Ftrac  = znl;
        if any(eldyn)
            % Electrodynamic braking is active
            [Feldyn,loco] = brake_eldyn(eldyn,Feldyn,loco,N,ploco,t,jmloc,y);
        end
        
        if any(trac)
            % Traction is active
            [Ftrac,loco] = trac_eldyn(trac,Ftrac,loco,N,ploco,t,jmloc,y);
        end
        
        % Coupling forces
        [bgdg,CCtrac,Fel,Feld,Fels,fitrac,ipos] = fbgdg(bgdg,CCtrac,Fel,Feld,Fels,...
            fitrac,ipos,lwagcs,N,strack,traccia,vr,y,xr);
        
        % Smoothing function for low speeds (these parameters can be put as 
        % advanced inputs, TO BE DISCUSSED)
        % When the speed is smtspeed (in m/s) the smooting value is smtval. 
        % This smoothing is necessary to manage very low speeds
        smtval = 0.99; smtspeed = 0.1;
        smt    = 2*atan(tan(smtval*pi*0.5)/smtspeed*y(N+1:2*N))/pi;
        
        % Running resistance forces
        fres = fresistenza(CCtrac,fitrac,N,pesoz,smt,y);
        
        % Force vector
        F = [-Fel(1);-Fel(2:end)+Fel(1:end-1);Fel(end)];
        F = F - smt.*Ffren - ssp*fres;
        F(ploco) = F(ploco) + ssp*(Ftrac - Feldyn);
        
        dy(1:N,1)   = y(N+1:2*N);
        dy(N+1:2*N) = invM.*F; % Acceleration
        % dy(2*N+1:3*N) = 0;
    end

    function UpManInd
        
    % UPMANIND This sub function updates the index of sub manoeuvre (jm) 
    %          and the vector that stores the sub-manoeuvres for each 
    %          locomotive (vjm). Moreover, the struct Mano (used to write 
    %          the results on hd) is updated
    
        if isempty(swcf)
            y0 = sol.y(:,end);
            
            % Updating of vjm and jm
            condup = znl;
            vjmo = vjm; % Variable is recorded to understand if a disruption is occurred
            for iloc = 1:nloco
                if vjm(iloc) < loco(iloc).nm
                    jmloc(iloc) = vjm(iloc)+1;
                else
                    jmloc(iloc) = vjm(iloc);
                end
                if vcond(iloc) == 1 && sol.x(end)+SR >= tmm(iloc,2) - 1e-9
                    condup(iloc) = 1;
                end
                if vcond(iloc) == 2 && sol.y(1,end) >= (loco(iloc).ds + loco(iloc).man(jmloc(iloc),4)) - 1e-9
                    condup(iloc) = 1;
                end
                if abs(vcond(iloc)) == 3 && abs(sol.y(N+1,end) - (loco(iloc).man(jmloc(iloc),5))) < 1e-3
                    condup(iloc) = 1;
                end
                if abs(vcond(iloc)) == 4 && abs(PBP(loco(iloc).man(jmloc(iloc),8)) - (loco(iloc).man(jmloc(iloc),6)))...
                        /((loco(iloc).man(jmloc(iloc),6))+1) < 1e-1
                    % The exit condition is reached when the percentage relative
                    % difference respect to the target is less than 1%.
                    % TODO: This value can be also put in input.
                    condup(iloc) = 1;
                end
                if abs(vcond(iloc)) == 5 && abs(pBCt(1+loco(iloc).man(jmloc(iloc),8)) - (loco(iloc).man(jmloc(iloc),7)))...
                        /((loco(iloc).man(jmloc(iloc),7))+1) < 1e-1
                    % The exit condition is reached when the percentage relative
                    % difference respect to the target is less than 1%.
                    % TODO: This value can be also put in input.
                    condup(iloc) = 1;
                end
                if vcond(iloc) == -6 && Ftrac(iloc) < 1, condup(iloc) = 1; end % Fictitious traction ended
                if vcond(iloc) == -7 && Feldyn(iloc) < 1, condup(iloc) = 1; end % Fictitious electro dynamic braking ended
                if condup(iloc) == 1 && vjm(iloc) < loco(iloc).nm
                    vjm(iloc) = vjm(iloc)+1;
                end
            end
            jm = sum(vjm);
            if jm == 0,
                jm = 1;
                warning('The simulation has been interrupted due to excessive longitudinal forces');
            end
            Mano(jm).pn = pn; Mano(jm).elpn = elpn; Mano(jm).eldyn = eldyn; Mano(jm).trac = trac;
            Mano(jm).tspan = sol.x; Mano(jm).jmloc = jmloc; Mano(jm).ts = tlocos;
            % Managing a delay in the activation of the sub-manoeuvre
            for iloc = 1:nloco
                if size(trac,2) == iloc && trac(iloc) == 1
                    Mano(jm).ts(iloc) = Mano(jm).ts(iloc) + loco(iloc).eltts;
                elseif size(eldyn,2) == iloc && eldyn(iloc) == 1
                    Mano(jm).ts(iloc) = Mano(jm).ts(iloc) + loco(iloc).elbts;
                end
                Mano(jm).Ftrac(iloc) = loco(iloc).Ftrac; Mano(jm).Feldyn(iloc) = loco(iloc).Feldyn;
                Mano(jm).tg(iloc) = t0;
            end
            
            if jm == nManv
                % The entire simulation has reached the end
                jm = nManv+1;
            end
            if nnz(vjm-vjmo) == 0 && sol.x(end) < tspan(end) % It means simulation stopped
                jm = nManv+1;
            end
        else
            Mano.pn = pn; Mano.elpn = elpn; Mano.eldyn = eldyn; Mano.trac = trac;
            Mano.tspan = tspan; Mano.jmloc = 1; Mano.ts = tlocos;
            jm = nManv+1;
        end
    end

    function [value,isterminal,direction] = events(t,y)
        
    % EVENTS The events function of the ODE solver stops the integration
    %        in specified conditions. Namely, when a submanoeuvre of a
    %        vehicle is completed.
        
        vvalue = znl; vdir = znl;
        for iloc_ev = 1:nloco
            % Possible updating of the manoeuvre indices
            if vjm(iloc_ev) < loco(iloc_ev).nm
                jmloc(iloc_ev) = vjm(iloc_ev)+1;
            else
                jmloc(iloc_ev) = vjm(iloc_ev);
            end
            if vcond(iloc_ev) == 1
                % Manoeuvre is controlled by time
                vvalue(iloc_ev) = t - (tspan(1) + loco(iloc_ev).man(jmloc(iloc_ev),3));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = 1;    % Negative direction only
            elseif vcond(iloc_ev) == 2
                % Manoeuvre is controlled by position
                vvalue(iloc_ev) = y(1) - (loco(iloc_ev).ds + loco(iloc_ev).man(jmloc(iloc_ev),4));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = 1;    % Negative direction only
            elseif vcond(iloc_ev) == 3
                % Manoeuvre is controlled by speed: it is assumed that 
                % starting speed is lower than target speed (traction)
                vvalue(iloc_ev) = y(N+1) - (loco(iloc_ev).man(jmloc(iloc_ev),5));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = 1;    % Negative direction only
            elseif vcond(iloc_ev) == -3
                % Manoeuvre is controlled by speed: it is assumed that 
                % starting speed is greter than target speed (braking)
                vvalue(iloc_ev) = y(N+1) - (loco(iloc_ev).man(jmloc(iloc_ev),5));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = -1;   % Positive direction only
            elseif vcond(iloc_ev) == 4
                % Manoeuvre is controlled by pressure in BP and it is assumed 
                % that starting pressure is lower than target pressure
                % (releasing, not supported actually) [n]
                vvalue(iloc_ev) = PBP(loco(iloc_ev).man(jmloc(iloc_ev),8)) - (loco(iloc_ev).man(jmloc(iloc_ev),6));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = +1;   % Negative direction only
            elseif vcond(iloc_ev) == -4
                % Manoeuvre is controlled by pressure in BP and it is assumed that
                % starting pressure is bigger than target pressure (braking)
                vvalue(iloc_ev) = PBP(loco(iloc_ev).man(jmloc(iloc_ev),8)) - (loco(iloc_ev).man(jmloc(iloc_ev),6));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = -1;   % Positive direction only
            elseif vcond(iloc_ev) == 5
                % Manoeuvre is controlled by pressure in BC and it is assumed that
                % starting pressure is lower than target pressure (braking)
                vvalue(iloc_ev) = pBCt(1+loco(iloc_ev).man(jmloc(iloc_ev),8)) - (loco(iloc_ev).man(jmloc(iloc_ev),7));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = +1;   % Negative direction only
            elseif vcond(iloc_ev) == -5
                % Manoeuvre is controlled by pressure in BC and it is assumed that
                % starting pressure is bigger than target pressure (releasing, not supported actually)
                vvalue(iloc_ev) = pBCt(1+loco(iloc_ev).man(jmloc(iloc_ev),8)) - (loco(iloc_ev).man(jmloc(iloc_ev),7));
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = -1;   % Positive direction only
            elseif vcond(iloc_ev) == -6
                % Fictitious traction manoueuvre added when the User asks for an
                % electro dynamic braking just after a traction
                vvalue(iloc_ev) = Ftrac(iloc_ev);
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = -1;   % Positive direction only
            elseif vcond(iloc_ev) == -7
                % Fictitious electrodynamic manoueuvre added when the User asks for an
                % traction just after an electro dynamic braking
                vvalue(iloc_ev) = Feldyn(iloc_ev);
                isterminal      = 1;    % Stop the integration
                vdir(iloc_ev)   = -1;   % Positive direction only
            end
        end
        % The manoeuvre interrupts if just one of the previous conditions is reached,
        % i.e. value crosses zero.
        [value,pos] = min(abs(vvalue));
        value       = value*sign(vvalue(pos));
        direction   = vdir(pos);
    end

    function status = odeplotL(t,y,flag,varargin)
        
    % ODEPLOTL This is the output fcn of the solver. It is called after
    %          every step of the integration. It is important for the 
    %          computations since the new pressure values are computed 
    %          for the next time step. Rest of the routine is copied from
    %          Matlab's default odeplotL function
        
        persistent TARGET_FIGURE TARGET_AXIS
        
        status = 0;                             % Assume stop button wasn't pushed.
        chunk = 128;                            % Memory is allocated in chunks.
        if not(strcmp(flag,'done'))
            
            splo = size(y,2);
            Flong = zeros(N-1,splo);
            Ffren = znv;
            for iplo = 1:splo
                xr = -diff(y(1:N,iplo));
                vr = -diff(y(1+N:2*N,iplo));
                
                [bgdg,CCtrac,Fel,Feld,Fels,fitrac,ipos] = fbgdg(bgdg,CCtrac,Fel,Feld,Fels,...
                    fitrac,ipos,lwagcs,N,strack,traccia,vr,y,xr);
                Flong(:,iplo) = Fel;
                
                % It is checked if the relative displacemets are allowed. In case
                % this is not true, the simulation stops. In future it can be
                % possible to manage what happens if the train disrupts.
                mij = max(xmaxct(:,1)-xr);
                if  mij > 0, status = 1; end;
                
                mij = min(xmaxct(:,2)-xr);
                if  mij < 0, status = 1; end;
            end
            y = Flong;
            if isempty(swcf) && t(splo)+SR - pBCtpSR(1) > 1e-9
                pBCt = pBCtpSR;
                % The pressure in Brake Pipe and in Brake Cylinder is written on the
                % pneumatic file
                fprintf(nfP,formP,pBCt,PBP,UBP,dmf);
                % Compute new pressure in BCs at time t(splo)+SR: it supposes that
                % during the initialization splo=1
                if any(isservice)
                    % Pressure is computed only if we are in a situation that differs
                    % by coasting.
                    [dmf,d2Qdt2o,dQdto,d2rodt2o,drodto,dt,d2udt2o,dudto,DT,grapi,mSA,...
                        P,pBCtpSR,PBP,pCdG_0,pCF,pCFm,pCFmnv0,Pmed,Pmedo,prpi,prSA,...
                        ptarget,Q,ro,T,tBkon,u,UBP] = comp_pressure_tpSR(CA,Cfq,cstPol,cv,CVactv,...
                        D,DT,dLC,dmf,dTF,d2Qdt2o,dQdto,d2rodt2o,drodto,dt,d2udt2o,dudto,dx,...
                        EOT,FOT,grapi,iCV,isservice,iV,j0,j1,jj,K,kpolSA,lam,Lmedia,mSA,...
                        n,nCV,nDBV,P,pBCt,pCdG_0,pCFm,pCFmnv0,pfin,Pmed,Pmedo,prpi,prSA,ptarget,...
                        Q,r,ro,rugrel,S,SA,s,segnoCA,segnoSA,segnof,splo,SR,Sut,...
                        t,T,Tamb,tBkon,t1,typeDBV,u,viscosita,vmisCF,Vbc,vDBV,vUnC);
                else
                    tfin    = t(splo)+SR;
                    pBCtpSR = [tfin pCF];
                end
                
            end
        end
        
        if nargin < 3 || isempty(flag)
            
            if (isempty(TARGET_FIGURE) || isempty(TARGET_AXIS))
                
                error('MATLAB:odeplot:NotCalledWithInit', ...
                    'ODEPLOT has not been initialized. Use syntax ODEPLOT(tspan,y0,''init'').');
                
            elseif (ishandle(TARGET_FIGURE) && ishandle(TARGET_AXIS))  % figure still open
                
                try
                    ud = get(TARGET_FIGURE,'UserData');
                    % Append t and y to ud.t and ud.y, allocating if necessary.
                    nt = length(t);
                    chunk = max(chunk,nt);
                    [rows,cols] = size(ud.y);
                    oldi = ud.i;
                    newi = oldi + nt;
                    if newi > rows
                        ud.t = [ud.t; zeros(chunk,1)];
                        ud.y = [ud.y; zeros(chunk,cols)];
                    end
                    ud.t(oldi+1:newi) = t;
                    ud.y(oldi+1:newi,:) = y.';
                    ud.i = newi;
                    set(TARGET_FIGURE,'UserData',ud);
                    
                    if ud.stop == 1                       % Has stop button been pushed?
                        status = 1;
                    else
                        % Rather than redraw all of the data every timestep, we will simply move
                        % the line segments for the new data, not erasing.  But if the data has
                        % moved out of the axis range, we redraw everything.
                        ylim = get(TARGET_AXIS,'ylim');
                        
                        % Replot everything if out of axis range or if just initialized.
                        if (oldi == 1) || (min(y(:)) < ylim(1)) || (ylim(2) < max(y(:)))
                            for j = 1:cols
                                set(ud.lines(j),'Xdata',ud.t(1:newi),'Ydata',ud.y(1:newi,j));
                            end
                            
                        else
                            % Plot only the new data.
                            for j = 1:cols
                                set(ud.line(j),'Xdata',ud.t(oldi:newi),'Ydata',ud.y(oldi:newi,j));
                            end
                        end
                    end
                    
                catch
                    err = lasterror();
                    error('MATLAB:odeplot:ErrorUpdatingWindow',...
                        'Error updating the ODEPLOT window. solution data may have been corrupted. %s',...
                        err.message);
                end
            end
            
        else
            
            switch(flag)
                case 'init'                           % odeplot(tspan,y0,'init')
                    ud = [];
                    cols = length(y);
                    ud.t = zeros(chunk,1);
                    ud.y = zeros(chunk,cols);
                    ud.i = 1;
                    ud.t(1) = t(1);
                    ud.y(1,:) = y.';
                    
                    % Rather than redraw all data at every timestep, we will simply move
                    % the last line segment along, not erasing it.
                    f = figure(gcf);
                    
                    TARGET_FIGURE = f;
                    TARGET_AXIS = gca;
                    
                    sgn = '-o';
                    
                    if ~ishold
                        ud.lines = plot(ud.t(1),ud.y(1,:),sgn);
                        hold on
                        ud.line = plot(ud.t(1),ud.y(1,:),sgn,'EraseMode','none');
                        hold off
                        set(TARGET_AXIS,'XLim',[min(t) max(t)]);
                    else
                        ud.lines = plot(ud.t(1),ud.y(1,:),sgn,'EraseMode','none');
                        ud.line = plot(ud.t(1),ud.y(1,:),sgn,'EraseMode','none');
                    end
                    
                    % The STOP button.
                    h = findobj(f,'Tag','stop');
                    if isempty(h)
                        ud.stop = 0;
                        pos = get(0,'DefaultUicontrolPosition');
                        pos(1) = pos(1) - 15;
                        pos(2) = pos(2) - 15;
                        uicontrol( ...
                            'Style','pushbutton', ...
                            'String','Stop', ...
                            'Position',pos, ...
                            'Callback',@StopButtonCallback, ...
                            'Tag','stop');
                    else
                        set(h,'Visible','on');            % make sure it's visible
                        if ishold
                            oud = get(f,'UserData');
                            ud.stop = oud.stop;             % don't change old ud.stop status
                        else
                            ud.stop = 0;
                        end
                    end
                    set(f,'UserData',ud);
                    grid on; xlabel('Time [s]'); ylabel('Longitudinal forces [N]');
                    
                case 'done'                           % odeplot([],[],'done')
                    
                    f = TARGET_FIGURE;
                    TARGET_FIGURE = [];
                    ta = TARGET_AXIS;
                    TARGET_AXIS = [];
                    
                    if ishandle(f)
                        ud = get(f,'UserData');
                        ud.t = ud.t(1:ud.i);
                        ud.y = ud.y(1:ud.i,:);
                        set(f,'UserData',ud);
                        cols = size(ud.y,2);
                        for j = 1:cols
                            set(ud.lines(j),'Xdata',ud.t,'Ydata',ud.y(:,j));
                        end
                        if ~ishold
                            set(findobj(f,'Tag','stop'),'Visible','off');
                            
                            if ishandle(ta)
                                set(TARGET_AXIS,'XLimMode','auto');
                            end
                            
                            refresh;                          % redraw figure to remove marker frags
                        end
                    end
                    
                    if isempty(swcf)
                        if jm == nManv
                            % This path is followed only if something goes wrong in
                            % the simulation.
                            input('It seems some information is missing. Please fix me!');
                            pBCt = pBCtpSR;
                            fprintf(nfP,formP,pBCt);
                            if exist(['Pneu_Dyn',appfmat],'file')
                                delete(['Pneu_Dyn',appfmat]);
                            end
                        elseif any(isservice)
                            save(['Pneu_Dyn',appfmat],'dmf','d2Qdt2o','dQdto','d2rodt2o',...
                                'drodto','dt','d2udt2o','dudto','DT','grapi','mSA',...
                                'P','pBCtpSR','pCdG_0','pCF','pCFm','pCFmnv0','Pmed',...
                                'Pmedo','prpi','prSA','ptarget','Q','ro','T','tBkon','u');
                        end
                    end
                    
            end
        end
        
        drawnow;
        
    end  % odeplot

% -------------------------------------------------------------------------
% Sub-function
%

    function StopButtonCallback(src,eventdata)
        ud = get(gcbf,'UserData');
        ud.stop = 1;
        set(gcbf,'UserData',ud);
    end  % StopButtonCallback


% -------------------------------------------------------------------------

if isempty(appf)
    disp(['Simulation time ',num2str(toc),' s']);
    disp(['Running distance ',num2str(sol.y(1,end)),' [m]']);
    % If the simulation is launched by the GUI, the window is closed
    if isempty(appf), close(h100); end
end

% Saving the temporal mat file, which has to be read by visres
save(['lastrun',appfmat])
end

function [CCtrac,Fel,Feld,Fels,Ffren,fitrac,frico,ilow,invM,ipos,iup,lwagcs,N,...
    onv,options,ptb,pesoz,posfc,ssp,train,vel_0,xmaxct,y0,znv] = IniNumInt(bgdg,Mvt,solver,train,vel_0)

% ININUMINT Initialization of variables used in numerical integration
%
% INPUTS    See integraode description
%
% OUTPTUS   CCtrac:  Track curvature
%           Fel:     Elastic force among the vehicles (full force) [N] 
%           Feld:    Right (destra)  elastic force among vehicles, positive
%           Fels:    Left (sinistra) elastic force among vehicles, positive
%           Ffren:   Braking force 
%           fitrac:  Slope of the track at the position of each vehicle    
%           frico:   Friction coefficient
%           ilow:    Indx of friction curves depending on specific pressure
%           invM:    Inverse of the mass of each vehicle
%           ipos:    Track section id where each vehicle is
%           iup:     Indx of friction curves depending on specific pressure 
%                    (to speed up access in look up table)
%           lwagcs:  Cumulative distance between centers of vehicles [m]    
%           N:       Number vehicles
%           onv:     Vector with ones 
%           options: Structure for ode solver
%           ptb:     Not supported any more (management of pressure)
%           pesoz:   Weight of each vehicle [N]
%           posfc:   Section id of the look up table for friction
%           ssp:     Sign of starting speed
%           train:   Struct array with info about each vehicle
%           vel_0:   Initial speed. Positive in this function [km/h]
%           xmaxct:  Array with maximum allowed relative displacements (to 
%                    avoid accessing points that do not belong to the 
%                    buffer or draw gear characteristics)
%           y0:      Initial condition for solver
%           znv:     Vector with zeros 

% Comments on ININUMINIT function:
%
% First half of initial condition y0 is initialized to zero and the second
% half to initial speed in [m/sec]. Rest of the variables also initialized
% to zero or one. Finally, lwagcs and xmaxct arrays are computed from train
% data and the structure Options for the ODE solver is filled with some
% custom settings. [SBB]

% Numerical integration of longitudinal dynamics using odes
N    = size(Mvt,2); 
Mvt  = Mvt';
invM = 1./Mvt; % The mass matrix is here inverted (it is diagonal and constant) % [s!] How can it be diagonal since it is a vector?

% TODO: put pesoz in input: now it wrongly depends also upon the rotating
% inertias.
pesoz = 9.81*Mvt;
ptb   = 1; % Index to manage the precomputed pressure in BCs
ipos  = ones(N,1);
znv   = zeros(N,1); znvm1 = zeros(N-1,1); onv = ones(1,N);
Ffren = znv'; Fel = znvm1; Feld = znvm1; Fels = znvm1; frico = [1;1]*Ffren;
ilow  = 2*onv; iup = onv; posfc = onv;

% Cumulative sum of the lengths of the vehicles
lwagcs = znv; fitrac = znv; CCtrac = znv;

% Set initial conditions for the integrator
y0  = zeros(2*N,1);
ssp = sign(vel_0(1)+1e-6);          % Sign of the starting speed

y0(N+1:2*N) = (vel_0(1)+1e-6)/3.6;
vel_0(1)    = vel_0(1)*ssp;         % The speed is considered positive
    
% The position of the indices controlled by speed are updated: these 
% indices control the friction coefficients; moreover, a matrix with 
% maximum allowed relative displacements is formed
xmaxct = zeros(N-1,2);
for ii = 1:N
    if not(isempty(train(ii).dfl)) && train(ii).dfl(1) == 100
        c = train(ii).dfl(2,1);
        while c < train(ii).dfl(1,2) && vel_0(1)/3.6 > train(ii).dfl(2,2+(c-1)*4)
            c = c+1;
        end
        train(ii).dfl(2,1) = c-1;
    end
    if not(isempty(train(ii).bbtype)) && train(ii).bfl == 99
        c = 1;
        while c < length(train(ii).fcv)-1 && vel_0(1)/3.6 > train(ii).fcv(c)
            c = c+1;
        end
        posfc(ii) =  c-1;
    end
    if ii < N
        xmaxct(ii,:) = [-bgdg(1,ii).xl(end-1) bgdg(2,ii).xl(end-1)];
    end
    if ii == 1
        lwagcs(ii) = 0;
    else
        lwagcs(ii) = 0.5*(train(ii-1).lwag + train(ii).lwag)+lwagcs(ii-1);
    end
    
end
% Jacobian Pattern
JJ = diag(0.5*ones(1,2*N))+diag([ones(1,N-1) 0 ones(1,N-1)],1)+...
    diag(ones(1,N),N)+diag(ones(1,N-1),N+1)+diag([0 ones(1,N)],N-1);
JJ = JJ+JJ';
options = odeset('RelTol',1e-6,'BDF','off','maxorder',2,...
    'AbsTol',1e-6,'Jpattern',sparse(JJ));
if strcmp(solver,'ode23t'), options = odeset(options,'RelTol',1e-5); end

end
