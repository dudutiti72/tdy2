function [Ffren,frico,ilow,iup,posfc,train] = brakeforce(Ffren,frico,ilow,iup,indexes,...
    nveicoli,pbrake_0,posfc,train,y,vel_0)
% This function computes the brake force as function of the brake system (disk or block),
% of the pressure in the brake cylinders and of the speed

% In this function the speed is positive only
y(nveicoli+1:2*nveicoli) = abs(y(nveicoli+1:2*nveicoli));

% DISK BRAKE SYSTEMS
%--------------------
% indexes.diskbrake contains the positions of vehicles with a disk brake
% system
for ii = indexes.diskbrake
    if train(ii).dfl(1) < 100
        % A fixed disk brake friction coefficient is used
        % df contains the costant friction coefficient
        df = train(ii).dfl;
    else
        % The disk friction law of the vehicle ii is described by a piecewise
        % polynomial
        % ratio is the ratio among the disk radious and the rolling radious
        if isempty(train(ii).dbdr) || isempty(train(ii).dbwr)
            % If a load system DISK_BW_EL or an  auto continuous system
            % DISK_BW_AC is set, there are not information about the ratio
            % between disk braking radius ad wheel radius. Is means that
            % the value train(ii).X is used, and the middle value below is taken
            ratio = 247/470;
        else
            ratio = train(ii).dbdr/train(ii).dbwr;
        end
        c = train(ii).dfl(2,1); % is a counter to moving on the piecewises polynomials
        [df,c] = interpbgdg(train(ii).dfl(1,2:end),c,train(ii).dfl(2,2:4:end),y(nveicoli+ii)*ratio);
        train(ii).dfl(2,1) = c; % Updating of the counter
    end
    % DISK_SI means that the disk brake sistem is defined by the physical
    % characteristics, for example S: section of the brake cylinder and i:
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
        Psp = 10.194 * SFdyn/(train(ii).nbs*train(ii).sB); % Specific pressure
        [coefat,ilow,iup,posfc] = comp_coefat(ii,ilow,iup,nveicoli,posfc,Psp,...
            SFdyn,train,y,vel_0); % Computation of friction coefficient
        Ffren(ii) = train(ii).pbb * coefat * SFdyn;
        frico(2,ii) = coefat; % Block friction coefficient is stored
    elseif isempty(train(ii).Fk) && train(ii).rendtim == 0.83
        % Ft is the force applied by the brake cylinder [kN]
        Ft = pbrake_0(ii)*train(ii).bbS-train(ii).Ff; %[kN]
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
        g = 9.81; % Acceleration of gravity
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
%99 means that the block friction law cames from a look-up table (file)
if train(ii).bfl == 99
    % The function fricoef_p_v computes polinomial equations and some interpolations in way
    % to extract from the look-up table the friction coefficinet
    % coefat
    [coefat,ilow,iup,posfc] = fricoef_p_v(ii,ilow,iup,nveicoli,posfc,Psp,...
        train,y);
elseif train(ii).bfl == 2
    % Karwatzki block friction law
    veloA = y(nveicoli+ii)*3.6; % train speed Km/h
    g = 9.81;
    % Karwatzki block friction law equations
    FdynkN = (SFdyn./train(ii).nbs)/1000;
    kKarw = 0.6*((((16/g)*FdynkN)+100)./(((80/g)*FdynkN)+100));
    coefat = kKarw*((veloA+100)/(5*veloA+100));
    % coefat: is the friction coefficient
elseif train(ii).bfl == 4
    % OSS block friction law
    % train speed Km/h
    veloA = y(nveicoli+ii)*3.6;
    g = 9.81;
    % OSS block friction law equations
    factorP=((875/g)*Psp+100)/((2860/g)*Psp+100);
    factorV=((10/3.6)*veloA+100)/((35/3.6)*veloA+100);
    coefat = 0.49*factorP*factorV;
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
% This function computes the BZA friction coefficient, the matrix
% DAT contains all the constant that are used in the BZA equations
% in function of the current speed, starting speed and load
% condition
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
 % This function computes polinomial equations and interpolations in way
 % to extract from the look-up table the friction coefficinet coefat
front = 0; % It is used to catch the moving direction
Pfc = train(ii).Pfc;
% iup is an index in fcp that refers to a specific pressure greater than Psp
while train(ii).fcp(iup(ii)) > Psp && iup(ii) > 1
    iup(ii) = iup(ii) - 1; front = 1;
end;
while front == 0 && train(ii).fcp(iup(ii)+1) < Psp
    iup(ii) = iup(ii) + 1;
end;
ilow(ii) = iup(ii)+1; % Index in fcp that refers to a specific pressure lower than Psp

[f,posfc(ii)] = interpbgdg(Pfc(:,iup(ii)),posfc(ii),train(ii).fcv,y(nveicoli+ii));
fcfup = f; % Value of the friction coefficient if the specific pressure were fcp(iup(ii))
x = y(nveicoli+ii)-train(ii).fcv(posfc(ii)); % Speed local to the piece of polynomy
% fcflow is the value of the friction coefficient if the specific pressure were fcp(ilow(ii))
fcflow = Pfc(2+(posfc(ii)-1)*4,ilow(ii))*x^3+Pfc(3+(posfc(ii)-1)*4,ilow(ii))*x^2+...
    Pfc(4+(posfc(ii)-1)*4,ilow(ii))*x+Pfc(5+(posfc(ii)-1)*4,ilow(ii));

% Linear interpolation to compute the desired friction coefficient
coefat = fcfup + (fcflow-fcfup)/(train(ii).fcp(ilow(ii))-train(ii).fcp(iup(ii)))*(Psp-train(ii).fcp(iup(ii)));

end