function [pCF,pCFm] = pCF_CG(dLC,dt,dTF,eftPt,ii,Manovra,pCdG,pCdG_0,pCF,pCFm,tBk)   

% PCF_CG  This function calculates the new pressure in a given, active 
%         Brake Cylinder
%
% INPUTS  dLC:     Array with limiting curve data of selected Control Valve
%         dt:      Time between the beginning and the end of the
%                  integration in comp_pressure_tpSR (tfin - told)
%         dTF:     Array with transfer function data of selected Control 
%                  Valve. Note that since from CalcCF only the data of one 
%                  vehicle are passed, dTF loses one dimension in the
%                  scope of this function. The same with dLC
%         eftPt:   eftPt(ii)                                                [?]
%         ii:      Counter of Control Valves
%         Manovra: Value of isservice variable 
%         pCdG:    Relative mean pressure in Brake Pipe around the Control
%                  Valve of selected vehicle at tfin [bar]
%         pCdG_0:  Previous relative pressure, at the beginning of
%                  integration in comp_pressure_tpSR
%         pCF:     Pressure in Brake Cylinders at time told (when 
%                  comp_pressure_tpSR is called)
%         pCFm:    Used to distinguish if Brake Cylinder is filled 
%                  following the limiting curve or the Brake Pipe pressure
%         tBk:     Time elapsed since beginning of Brake Cylinder filling
%
% OUTPUTS pCF:     Updated pressure in Brake Cylinder at time tfin
%         pCFm:    Updated pCFm at tfin

global tFpt ASph   

% ASph: Stands for Application Stroke phase (true,false)
% tFpt: Variable to measure time during Application stroke. It varies from
%       0 to t_end_of_AS and then its last value is used to calculate the
%       pressure in BC during the inshot function phase. [SBB]
 
% Assignment of temporary variables to make notation more intuitive 
pCFAs = dLC(2); tAs   = dLC(3); pCGAs = dLC(4); pCFIf = dLC(5);  
tIf   = dLC(6); cfLmt = dLC(7:end); 
%Pactv = dLC(1); dTFa = zeros(6,2);

dTFa(1:6,1:2) = dTF; 
                                
NpBk    = dTFa(end,1); 
NpRls   = dTFa(end,2);
pCGtf   = dTFa(1:NpBk,1); 
pCFtf   = dTFa(1:NpBk,2); 
pCGtfRl = dTFa(NpBk+1:NpBk+NpRls,1); 
pCFtfRl = dTFa(NpBk+1:NpBk+NpRls,2); 

%--------------------------------------------------------------------------
% Actual value of application stroke and inshot function BC pressure
% following the type of empty/load device
if eftPt == 1 && (pCdG >= pCGAs || tBk <= tAs+dt/2) && ASph(ii) == 1       
    %==============================
    % Simulation application stroke
    pCF  = pCFAs;
    pCFm = pCF;
    tFpt(ii) = tBk; 
elseif eftPt == 1 && (tBk > tAs+dt/2) && tBk-tFpt(ii) <= tIf+dt/2
    %======================================================================
    %Simulation Inshot function
    pCF  = pCFAs+(pCFIf-pCFAs)/tIf*(tBk-tFpt(ii)); % Linear
    pCFm = pCF;
    ASph(ii) = 0;
else
    
    %----------------------------------------------------------------------
    % BC PRESSURE CONTROLLED BY BP
    if (pCdG - pCdG_0)/dt < 0.05
        %==================================================================
        % BC pressure controlled by BP during braking
        appo = find(pCGtf > pCdG);
        if appo
            ind = appo(1);
            if ind == 1
                pCFcdg = pCFtf(1);
            else       
                m = (pCFtf(ind-1)-pCFtf(ind))/(pCGtf(ind)-pCGtf(ind-1));
                pCFcdg = m*(pCGtf(ind)-pCdG)+pCFtf(ind);        
            end                  
        else
            pCFcdg = pCFm;
        end
    else
        %==================================================================
        % BC pressure controlled by BP during releasing     
        appo = find(pCGtfRl > pCdG);
        if appo
            ind = appo(1);
            if ind == 1
                pCFcdg = pCFtfRl(1);
            else       
                m = (pCFtfRl(ind-1)-pCFtfRl(ind))/(pCGtfRl(ind)-pCGtfRl(ind-1));
                pCFcdg = m*(pCGtfRl(ind)-pCdG)+pCFtfRl(ind);        
            end                  
        else
            pCFcdg = 0;  
        end
    end
        
    SgnBK    = sign(pCFcdg-pCFm); % Variable to define if we are in charging or discharging the BC    
    pCFdTcdg = (pCFcdg-pCFm);     % BC pressure gradient controlled by BP
    
    %----------------------------------------------------------------------
    % BC PRESSURE CONTROLLED BY BRAKE REGIME (G or P)
    %======================================================================
    % BC pressure controlled by brake regime during charging of BC    
    if (1+SgnBK) 
        % Extrapolation of value on limiting curve 
        a = cfLmt(1); b = cfLmt(2); c = cfLmt(3); tmx = cfLmt(4); %support variable     
        sol = (-b+[1 -1].*sqrt(b^2-4*a*(c-pCFm)))./(2*a);
        sol(find(imag(sol))) = [];
        rad     = find(sol >= 0 & sol <= tmx);
        tBkdn   = sol(rad);
        pCFdTdn = ((2*a)*tBkdn+b)*dt;   % BC pressure gradient controlled by brake regime
        
        % Define coefficient histeresys during CV action
        Cfhis = 1;
        if (pCdG > pCGtfRl(1)) && max(Manovra) > 0 && ( pCdG >= pCdG_0 || abs(pCdG-pCdG_0)/dt < 0.05 )
            Cfhis = 0;
        end
        % Calculation of actual value of BC pressure
        [pCFdT] = min([pCFdTcdg,pCFdTdn]);        
        pCFm    = pCFm+Cfhis*pCFdT;
        pCF     = pCF+Cfhis*pCFdT;
    end     
    %======================================================================
    % BC pressure controlled by brake regime during discharging of BC  
    if SgnBK == -1
        % Extrapolation of value on limiting curve 
        a = cfLmt(5); b = cfLmt(6); c = cfLmt(7); tmn = cfLmt(8);%support variable        
        sol = (-b+[1 -1].*sqrt(b^2-4*a*(c-pCFm)))./(2*a);
        sol(find(imag(sol))) = [];
        rad = find(sol >= 0 & sol <= tmn);
        tBkdn = sol(rad);
        pCFdTdn = ((2*a)*tBkdn+b)*dt;  % BC pressure gradient controlled by brake regime

        % Define coefficient histeresys during CV action
        Cfhis = 1;
        if eftPt == 1 && ((pCdG-pCdG_0)/dt > 0.08) %sensitivity threshold
            Cfhis = 1; %0.01
        %elseif (pCdG > pCGtf(1)) && Manovra < 0 && ( (pCdG-pCdG_0) <= 0 || abs(pCdG-pCdG_0)/dt < 0.05 )
        elseif (pCdG > pCGtf(1)) && min(Manovra) < 0 && ( pCdG <= pCdG_0 || abs(pCdG-pCdG_0)/dt < 0.05 )
            Cfhis = 0;
            %elseif Manovra == 1 & (pCdG-pCdG_0) <= 0
        end
        % Calculation of actual value of BC pressure
        [pCFdT] = max([pCFdTcdg,pCFdTdn]);
        pCFm    = pCFm+Cfhis*pCFdT;
        pCF     = pCF+Cfhis*pCFdT;  
    end  
end