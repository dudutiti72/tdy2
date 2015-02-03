function [dmf,grapi,P,Pmed,Pmedo,prpi,ptarget,Q,ro,T,u,velc] = boundaryCond2(Cfq,...
    dmf,dt,grapi,isservice,iV,nDBV,P,pfin,Pmed,Pmedo,prpi,ptarget,Q,ro,S,...
    segnof,Sut,t,T,t1,typeDBV,u,vDBV)

global cv Tamb

% DEFINING NEW DATA FOR BRAKE PIPE PRESSURE DEVELOPING
% FOLLOWING INPUT DBV
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velc = zeros(1,nDBV);
for kk = vDBV
    tkk = t;
    if tkk >= 0
        switch isservice(kk)
            case -2 % Service discharging
                Pmed(kk) = 0.5*(P(iV(1,kk))+P(iV(2,kk)));
                %# fastindex
                Tmed = 0.5*(T(iV(1,kk))+T(iV(2,kk)));
                %# fastindex
                romed = 0.5*(ro(iV(1,kk))+ro(iV(2,kk)));

                % Counterpressure computation
                prpi(kk) = discharge(dt,grapi(kk,:),prpi(kk),ptarget(kk));
                %# fastindex
                %Pd = prpi(kk);     Td = Tamb
                % If the service braking does not start from 5 bar, the
                % pressure in brake pipe increases and then decreases. The
                % following line fixes this behaviour
                Pd = min([prpi(kk) Pmed(kk)]);     Td = Tamb; % Modified on 05/07/2012
                %# fastindex
                Pu = Pmed(kk);     Tu = Tmed;

            case {1,2} % Recharging
                Pmed(kk) = 0.5*(P(iV(1,kk))+P(iV(2,kk)));
                %# fastindex
                Tmed = 0.5*(T(iV(1,kk))+T(iV(2,kk)));
                %# fastindex
                romed = 0.5*(ro(iV(1,kk))+ro(iV(2,kk)));
                %# fastindex
                gracg = (Pmed(kk)-Pmedo(kk))/dt;
                Pmedo(kk) = Pmed(kk);

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
                Tmed = 0.5*(T(iV(1,kk))+T(iV(2,kk)));
                %# fastindex
                romed = 0.5*(ro(iV(1,kk))+ro(iV(2,kk)));
                
                %# fastindex
                Pd = 1.01325e5;      Td = Tamb;
                %# fastindex
                Pu = Pmed(kk);       Tu = Tmed;
        end;
        
        % Calculation mass flow rate through orifice
        %coeqv = 0.72; %per EOT in coda al treno da 1200m %coeqv = 0.76; per dispositivo ostruzioni  
        dmf(kk) = fxNzleVcq(Cfq(kk),Pd,Pu,segnof(kk),Sut(kk),Td,Tu);
        
        % Setting the B.Cs.
        if typeDBV(kk) ~= 1 % in case of frontal or rear device
            velc(kk) = dmf(kk)/(ro(iV(3,kk))*S(iV(3,kk)));

            % Setting the boundary conditions
            u(iV(3,kk)) = velc(kk);
            P(iV(3,kk)) = P(iV(1,kk)); ro(iV(3,kk)) = ro(iV(1,kk)); T(iV(3,kk)) = T(iV(1,kk)); Q(iV(3,kk)) = cv*T(iV(3,kk)) + 0.5*u(iV(3,kk))^2;

        else % in case of lateral device
            velc(kk) = dmf(kk)/(romed*Sut(kk));
        end;
    end;    
end;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%