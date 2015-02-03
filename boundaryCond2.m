function [dmf,grapi,P,Pmed,Pmedo,prpi,ptarget,Q,ro,T,u,velc] = boundaryCond2(Cfq,...
    dmf,dt,grapi,isservice,iV,nDBV,P,pfin,Pmed,Pmedo,prpi,ptarget,Q,ro,S,...
    segnof,Sut,t,T,t1,typeDBV,u,vDBV)

% BOUNDARYCOND2 This function calculates the boundary conditions for the 
%               pneumatic problem at each step of the integration taking
%               into account the status of each Driver's Brake Valve and
%               their current operation mode. For this purpose, the mass 
%               flow rate is also calculated
%
% INPUTS        Cfq:       Flow coefficient). -1 if not defined by user
%               dmf:       Mass flow rate through Driver's Brake Valves
%                          [kg/s]
%               dt:        Current time step [s]
%               grapi:     Pressure gradient in pilot chamber [Pa/s]
%               isservice: Variable that indicates the type of manoeuvre 
%                          for each locomotive (EB,SB,R)
%               iV:        Index with section ids where Driver's Brake  
%                          Valves are connected to the Brake Pipe 
%               nDBV:      Number of Driver's Brake Valves
%               P:         Pressure in each Brake Pipe section [Pa]
%               pfin:      Final pressure of manoeuvre [Pa]
%               Pmed:      Mean pressure of Brake Pipe around Driver's
%                          Brake Valves [Pa]
%               Pmedo:     Mean pressure on previous time step [Pa]
%               prpi:      Pressure in pilot chamber [Pa]
%               ptarget:   Absolute target pressure of pneumatic manoeuvre
%                          [Pa]
%               Q:         Specific energy of each Brake Pipe section
%               ro:        Density of each Brake Pipe section [Kg/m^3]
%               S:         Cross-section of each Brake Pipe section
%               segnof:    Direction of flow (charge/discharge of nozzle)
%               Sut:       Cross section of nozzle [m^2]
%               t:         Current time [s]
%               T:         Temperature of each Brake Pipe section [K]
%               t1:        Used for recharging.Represents the time needed
%                          to reach the target pressure in Brake Pipe
%                          during releasing [s]
%               typeDBV:   Vector with type of each Driver's Brake Valve
%               u:         Flow speed in each Brake Pipe section
%               vDBV:      Vector indicating active Driver's Brake Valves
%
% OUTPUTS       dmf:       Updated Mass flow rate [kg/s]
%               grapi:     Updated gradient in case of recharge
%               P:         Updated pressure in case of front-rear devices
%               Pmed:      Updated mean pressure depending on manoeuvre
%               Pmedo:     Current mean pressure, to be used as previous
%                          pressure in next iteration
%               prpi:      Modified gradient from discharge or recharge
%                          function
%               ptarget:   Updated targer pressure in case of recharge
%               Q:         THESE QUANTITIES ONLY GET UPDATED
%               ro:        IN CASE OF FRONT-REAR DEVICES
%               T:         WHICH ARE NOT CURRENTLY 
%               u:         SUPPORTED IN GUI                                 
%               velc:      Lateral velocity (u_l in governing equations)


global cv Tamb

% DEFINING NEW DATA FOR BRAKE PIPE PRESSURE DEVELOPING
% FOLLOWING INPUT DBV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velc = zeros(1,nDBV);
for kk = vDBV
    tkk = t;
    if tkk >= 0
        switch isservice(kk)
            case -2 % Service discharging
                Pmed(kk) = 0.5*(P(iV(1,kk))+P(iV(2,kk)));
                %# fastindex
                Tmed     = 0.5*(T(iV(1,kk))+T(iV(2,kk)));
                %# fastindex
                romed    = 0.5*(ro(iV(1,kk))+ro(iV(2,kk)));
                % Counterpressure computation
                prpi(kk) = discharge(dt,grapi(kk,:),prpi(kk),ptarget(kk));
                %# fastindex
                %Pd = prpi(kk);     Td = Tamb
                % If the service braking does not start from 5 bar, the
                % pressure in Brake Pipe increases and then decreases. The
                % following line fixes this behaviour
                Pd = min([prpi(kk) Pmed(kk)]);     
                Td = Tamb; % Modified on 05/07/2012
                %# fastindex
                Pu = Pmed(kk);
                Tu = Tmed;

            case {1,2} % Recharging
                Pmed(kk)  = 0.5*(P(iV(1,kk))+P(iV(2,kk)));
                %# fastindex
                Tmed      = 0.5*(T(iV(1,kk))+T(iV(2,kk)));
                %# fastindex
                romed     = 0.5*(ro(iV(1,kk))+ro(iV(2,kk)));
                %# fastindex
                gracg     = (Pmed(kk)-Pmedo(kk))/dt; % Pressure gradient in Brake Pipe during releasing [SBB]
                Pmedo(kk) = Pmed(kk);                % Update for next function call [SBB]

                % Counterpressure computation
                [grapi(kk,1),prpi(kk),ptarget(kk)] = recharge(dt,gracg,...
                    grapi(kk,1),pfin,Pmed(kk),prpi(kk),ptarget(kk),t,t1(kk));
                %# fastindex
                Pd = Pmed(kk);       Td = Tmed;
                %# fastindex
                Pu = prpi(kk);       Tu = Tamb;

            case -1 % Emergency braking
                Pmed(kk) = 0.5*(P(iV(1,kk))+P(iV(2,kk)));
                %# fastindex
                Tmed     = 0.5*(T(iV(1,kk))+T(iV(2,kk)));
                %# fastindex
                romed    = 0.5*(ro(iV(1,kk))+ro(iV(2,kk)));
                
                %# fastindex
                Pd = 1.01325e5;      Td = Tamb;
                %# fastindex
                Pu = Pmed(kk);       Tu = Tmed;
        end
        
        % Calculation mass flow rate through orifice
        %coeqv = 0.72; %per EOT in coda al treno da 1200m %coeqv = 0.76; per dispositivo ostruzioni  
        dmf(kk) = fxNzleVcq(Cfq(kk),Pd,Pu,segnof(kk),Sut(kk),Td,Tu);
        
        % Setting the B.Cs.
        if typeDBV(kk) ~= 1 % in case of frontal or rear device
            velc(kk)     = dmf(kk)/(ro(iV(3,kk))*S(iV(3,kk)));

            % Setting the boundary conditions
            u(iV(3,kk))  = velc(kk);
            P(iV(3,kk))  = P(iV(1,kk)); 
            ro(iV(3,kk)) = ro(iV(1,kk)); 
            T(iV(3,kk))  = T(iV(1,kk)); 
            Q(iV(3,kk))  = cv*T(iV(3,kk)) + 0.5*u(iV(3,kk))^2;

        else % in case of lateral device
            velc(kk)     = dmf(kk)/(romed*Sut(kk));
        end
    end    
end
