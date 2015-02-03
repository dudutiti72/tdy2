function [Ffren,frico,ilow,iup,posfc,train] = brakeforce(Ffren,frico,ilow,iup,indexes,...
    nveicoli,pbrake_0,posfc,train,y,vel_0)

% BRAKEFORCE    This function computes the brake forces considering the
%               braking system, the pressure in Brake Cylinders and the  
%               running conditions
%
% INPUTS        Ffren:    Previous brake force                              [s!] Since it's always initialized to zero before calling this function, why do we need it as an input?
%               frico:    Array with friction coefficients. First line for
%                         disk braked vehicles and second for block braked.
%                         It has as many columns as vehicles
%               ilow:     Index of friction curves depending on specific 
%                         pressure, to speed up access in look up table
%               iup:      Index of friction curves depending on specific 
%                         pressure
%               indexes:  Struct array with position of wagons equipped
%                         with disk or block brake system
%               nveicoli: Number of vehicles
%               pbrake_0: Pressure in Brake Cylinders at time t, using 
%                         linear interpolation between pBCt and pBCtpSR
%               posfc:    Section id of the look up table for friction
%               train:    Struct array with info about each vehicle
%               y:        The first half of this vector includes the 
%                         position of each vehicle and the second half
%                         the velocities. i.e: y(i+1)-y(i) is the relative
%                         displacement between vehicles i and i+1
%               vel_0:    Initial velocity [km/h]. Second element not
%                         supported in GUI
%
% OUTPUTS       Ffren:    Current calculated brake force
%               frico:    Updated array with friction coefficients 
%               ilow:     Updated index
%               iup:      Updated index
%               posfc:    Updated index
%               train:    Updated struct array


% Fk [KN]:  Through these values, the sum of the maximum contact forces 
%           between shoes (or disk) and wheel can be defined for each 
%           vehicle. The braking  force  is  computed  scaling  the maximum 
%           contact force according to the pressure in the Brake Cylinder. 
%
% pbrake_0: Interpolated pressure of Brake Cylinders
%
% pCFA:     Control Valve option. Brake Cylinder pressure during 
%           application stroke [SBB]


% In this function the speed is positive only
y(nveicoli+1:2*nveicoli) = abs(y(nveicoli+1:2*nveicoli));

% DISK BRAKE SYSTEMS
%--------------------
% indexes.diskbrake contains the positions of vehicles with a disk brake
% system

for ii = indexes.diskbrake
    if train(ii).dfl(1) < 100 % Convention when setting dfl field [SBB]
        
        % A fixed disk brake friction coefficient is used
        % df contains the constant friction coefficient               
        df = train(ii).dfl;
    else
        % The disk friction law of the vehicle ii is described by a piecewise
        % polynomial
        
        % ratio  = Disk radius / Wheel radius                        

        if isempty(train(ii).dbdr) || isempty(train(ii).dbwr)
            % If a load system DISK_BW_EL or an  auto continuous system
            % DISK_BW_AC is set, there is not information about the ratio     
            % between disk braking radius and wheel radius. This means that
            % the value train(ii).X is used, and the middle value below is 
            % considered
            ratio = 247/470;
        else
            ratio = train(ii).dbdr/train(ii).dbwr;
        end
        c = train(ii).dfl(2,1); % index for piecewise polynomials
        [df,c] = interpbgdg(train(ii).dfl(1,2:end),c,train(ii).dfl(2,2:4:end),y(nveicoli+ii)*ratio);
        train(ii).dfl(2,1) = c; % Updating index
    end
    % DISK_SI means that the disk brake system is defined by the physical
    % characteristics, for example S: section of the Brake Cylinder and i:
    % rigging ratio
    if strcmp(train(ii).dbtype,'DISK_SI') && isempty(train(ii).Fk)
        % .X1 and .X2 are two costants computed by the disk brake system
        % equations
        Ffren(ii) = train(ii).pdb*(pbrake_0(ii)*train(ii).X1 - train(ii).X2)*df * 1000; % [N]
        if Ffren(ii) < 0, Ffren(ii) = 0; end
    elseif train(ii).Fk > 0 
        if pbrake_0(ii) > train(ii).pCFAs
            Ffren(ii) = train(ii).pdb * df * 1e3 * train(ii).Fk * ...
                (pbrake_0(ii)-train(ii).pCFAs)/(train(ii).pBC-train(ii).pCFAs);
        end
    else
        % In this case the disk brake system is defined by the brake
        % weight, the system could be an empty load system (defined by braked
        % weight)DISK_BW_EL or an  auto continuous system DISK_BW_AC 
        Ffren(ii) = train(ii).pdb*train(ii).X*df/0.35*pbrake_0(ii)*1000; %[N]
    end
    frico(1,ii) = df; % Disk friction coefficient is stored
end

% BLOCK BRAKE SYSTEMS
%--------------------
% indexes.blockbrake contains the positions of vehicles with a block brake
% system
for ii = indexes.blockbrake
    if not(isempty(train(ii).Fk)) && (train(ii).Fk > 0 && pbrake_0(ii) > train(ii).pCFAs)
        
        SFdyn = 1e3 * train(ii).Fk * (pbrake_0(ii)-train(ii).pCFAs)/(train(ii).pBC-train(ii).pCFAs); % [N]
        Psp   = 10.194 * SFdyn/(train(ii).nbs*train(ii).sB); % Specific pressure
        [coefat,ilow,iup,posfc] = comp_coefat(ii,ilow,iup,nveicoli,posfc,Psp,...
            SFdyn,train,y,vel_0); % Computation of friction coefficient
        Ffren(ii)   = train(ii).pbb * coefat * SFdyn;
        frico(2,ii) = coefat; % Block friction coefficient is stored
    elseif isempty(train(ii).Fk) && train(ii).rendtim == 0.83
        % Ft is the force applied by the Brake Cylinder [kN]
        Ft = pbrake_0(ii)*train(ii).bbS-train(ii).Ff;
        % SFdyn is the sum of the forces applied by the shoes on the wheels [N]
        SFdyn = 1e3*(Ft*train(ii).bbiG-train(ii).na*2*train(ii).Fr)*train(ii).rendtim;
        if SFdyn > 0
            %Psp is the specific pressure on the brake block [kgf/cm^2]
            Psp = 10.194 * SFdyn/(train(ii).nbs*train(ii).sB);
            [coefat,ilow,iup,posfc] = comp_coefat(ii,ilow,iup,nveicoli,posfc,Psp,...
                SFdyn,train,y,vel_0);
            % .pbb is a constant that you can choose in input as percentage of
            % application
            % Computation of the brake force
            Ffren(ii) = Ffren(ii) + train(ii).pbb*SFdyn*coefat; %[N]
            % The block friction coefficient is computed (and then stored) only when
            % there is a contact among the wheel and the brake shoe.
            frico(2,ii) = coefat;
        end
    elseif pbrake_0(ii) > train(ii).pCFAs
        % This is a special situation not supported by the GUI, useful for
        % computation of stopping distance by computing the desired SFdyn
        SFdyn = train(ii).Fdyn;
        veloA = y(nveicoli+ii)*3.6; % train speed Km/h
        g     = 9.81; % Acceleration of gravity
        % Karwatzki block friction law equations
        FdynkN = (SFdyn/train(ii).nbs2)/1000;
        kKarw = 0.6*((((16/g)*FdynkN)+100)./(((80/g)*FdynkN)+100));
        coefat = kKarw*((veloA+100)/(5*veloA+100));
        Ffren(ii) = train(ii).pbb * coefat * 1e3 * SFdyn * ...
            (pbrake_0(ii)-train(ii).pCFAs)/(train(ii).pBC-train(ii).pCFAs);
        frico(2,ii) = coefat;
    end
end

end

function [coefat,ilow,iup,posfc] = comp_coefat(ii,ilow,iup,nveicoli,posfc,Psp,...
    SFdyn,train,y,vel_0)

% COMP_COEFAT This function calculates the friction coefficient of a block
%             brake system taking into account the specified friction law
%
% INPUTS      ii:       Index of vehicle in question
%             ilow:     Index of friction curves depending on specific 
%                       pressure
%             iup:      Index of friction curves depending on specific 
%                       pressure (to speed up access in look up table)
%             nveicoli: Number of vehicles
%             posfc:    Section id of the look up table for friction
%             Psp:      Specific pressure  between shoe and wheel
%             SFdyn:    Total normal force from shoe to wheel [N]
%             train:    Struct array with info about each vehicle
%             vel_0:    Initial speed [km/h]
%
% OUTPUTS     coefat:   Friction coefficient given the current conditions
%             ilow:     Updated index
%             iup:      Updated index
%             posfc:    Updated index


%99 means that the block friction law cames from a look-up table (file)
if train(ii).bfl == 99
    % The function fricoef_p_v computes polynomial equations and some interpolations 
    % in order to extract from the look-up table the friction coefficient coefat    
    [coefat,ilow,iup,posfc] = fricoef_p_v(ii,ilow,iup,nveicoli,posfc,Psp,...
        train,y);
elseif train(ii).bfl == 2
    % Karwatzki block friction law
    veloA  = y(nveicoli+ii)*3.6; % train speed converted to [km/h]
    g      = 9.81;
    % Karwatzki block friction law equations
    FdynkN = (SFdyn./train(ii).nbs)/1000;
    kKarw  = 0.6*((((16/g)*FdynkN)+100)./(((80/g)*FdynkN)+100));
    coefat = kKarw*((veloA+100)/(5*veloA+100));
    % coefat: is the friction coefficient
elseif train(ii).bfl == 4
    % OSS block friction law
    % train speed Km/h
    veloA   = y(nveicoli+ii)*3.6;
    g       = 9.81;
    % OSS block friction law equations
    factorP = ((875/g)*Psp+100)/((2860/g)*Psp+100);
    factorV = ((10/3.6)*veloA+100)/((35/3.6)*veloA+100);
    coefat  = 0.49*factorP*factorV;
elseif train(ii).bfl == 5
    % BZA block friction law
    starting_speed=vel_0(1) ;
    current_speed=y(nveicoli+ii)*3.6; %[km/h]
    if starting_speed<=30
        indice_velo=1;
    elseif starting_speed<=60.1 && starting_speed>30
        if current_speed <=20
            indice_velo=2;
        elseif current_speed <=60.1 && current_speed > 20
            indice_velo=3;
        end
    end
    if isempty(train(ii).bbbwl) || strcmp(train(ii).typeSh,'Bgu')==1
        disp('BZA can not be used with Bgu and autovariable, stop the program');
        pause
    end
    if not(isempty(train(ii).bbbwl))
        if  train(ii).tare+train(ii).load<=train(ii).bbbwi
            indice_carico=2;
            % empty conditions
        else
            indice_carico=1;
            % load conditions
        end
        % the BZA friction coefficient is computed by an external function called
        % BZA_friction, it takes as inputs the current speed, the load condition
        % (load or empty) and the starting speed
        coefat = BZA_friction(current_speed, indice_carico, indice_velo);
    end
end
end

function [muBZA] = BZA_friction(velo, indice_carico, indice_velo)

% BZA_friction This function computes the BZA friction coefficient, the
%              matrix DAT contains all the constants that are used in the 
%              BZA equations in function of the current speed, starting 
%              speed and load condition
% 
% INPUTS       velo:          Current speed
%              indice_carico: 1 for load conditions, 2 for empty conditions
%              indice_velo:   From 1 to 3 depending on the interval of
%                             initial speed
% 
% OUTPUT       muBZA:         Calculated friction coefficient

% velo is writed in [Km/h]
DAT=[-6.67 6 -1.93 0.46 0;...
    -7.5 5.75 -1.65 0.4 0;...
    -36.67 15.33 -2.18 0.385 0;...
    -43.33 16.67 -2.23 0.315 0;...
    0 0 0 0.21 0.012;...
    0 0 -0.1 0.21 0];
indice=2*indice_velo+(1-indice_carico);
a=DAT(indice,1);
b=DAT(indice,2);
c=DAT(indice,3);
d=DAT(indice,4);
e=DAT(indice,5);
muBZA=a*(velo/100)^3+b*(velo/100)^2+c*(velo/100)+d+e*(100/velo);
end

function [coefat,ilow,iup,posfc] = fricoef_p_v(ii,ilow,iup,nveicoli,posfc,Psp,...
    train,y)

% FRICOEF_P_V In the case of a user defined block friction law, this 
%             function evaluates the polynomial approximation in order to
%             extract the friction coefficient for the specified speed and
%             specific pressure
%
% INPUTS      ii:       number of vehicle in question
%             ilow:     Index of friction curves depending on specific 
%                       pressure
%             iup:      Index of friction curves depending on specific 
%                       pressure, to speed up access in look up table
%             nveicoli: Number of vehicles
%             posfc:    Section id of the look up table for friction
%             Psp:      Specific pressure from shoe to wheel
%             train:    Struct array with info about each vehicle
%             y:
%
% OUTPUTS     coefat:   Calculated coefficient
%             ilow:     Updated index
%             iup:      Updated index
%             posfc:    Updated index

front = 0; % It is used to catch the moving direction
Pfc   = train(ii).Pfc;

% iup is an index in fcp that refers to a specific pressure greater than Psp
while train(ii).fcp(iup(ii)) > Psp && iup(ii) > 1
    iup(ii) = iup(ii) - 1; front = 1;
end
while front == 0 && train(ii).fcp(iup(ii)+1) < Psp
    iup(ii) = iup(ii) + 1;
end
ilow(ii) = iup(ii)+1; % Index in fcp that refers to a specific pressure lower than Psp

[f,posfc(ii)] = interpbgdg(Pfc(:,iup(ii)),posfc(ii),train(ii).fcv,y(nveicoli+ii));
fcfup  = f; % Value of the friction coefficient if the specific pressure were fcp(iup(ii))
x      = y(nveicoli+ii)-train(ii).fcv(posfc(ii)); % Speed local to the piece of polynomy
% fcflow is the value of the friction coefficient if the specific pressure were fcp(ilow(ii))
fcflow = Pfc(2+(posfc(ii)-1)*4,ilow(ii))*x^3+Pfc(3+(posfc(ii)-1)*4,ilow(ii))*x^2+...
    Pfc(4+(posfc(ii)-1)*4,ilow(ii))*x+Pfc(5+(posfc(ii)-1)*4,ilow(ii));

% Linear interpolation to compute the desired friction coefficient
coefat = fcfup + (fcflow-fcfup)/(train(ii).fcp(ilow(ii))-train(ii).fcp(iup(ii)))*(Psp-train(ii).fcp(iup(ii)));

end