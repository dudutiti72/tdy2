function [Cfq,grapi,prpi,ptarget,segnof,Sut,t1] = initialManvBP(CA,CVactv,DBVdata,...
    dt,isservice,nCV,pCF,pfin,Pn,prpi,typeDBV)                              % [s!] Too many inputs are not used. Consider removing them

% INITIALMANVBP This function initializes the manoeuvre indices for the 
%               Brake Pipe according to the current manoeuvre
%
% INPUTS        CA:        Array with Acceleration Chamber data
%               CVactv:    Scalar with status of Control Valves. see
%                          integraode for details
%               DBVdata:   Array with Driver brake valve data
%               dt:        Actual time step [s]
%               isservice: Variable that indicates the type of manoeuvre
%                          for each locomotive (EB,SB,R)
%               nCV:       Number of Control Valves
%               pCF:       Brake Cylinder pressures [bar]
%               pfin:      Final absolute pressure of pneumatic manoeuvre  
%                          for each locomotive [Pa]
%               Pn:        Absolute Brake Pipe pressure in each pipe 
%                          section [Pa]
%               prpi:      Pressure in pilot chambers [Pa]
%               typeDBV:   Vector with Driver's Brake Valve types
%
% OUTPUTS       Cfq:       Flow coefficients of Driver's Brake Valves 
%                          taking into account their operation mode
%               grapi:     Pressure gradient in pilot chambers. For service
%                          brake, first column has the gradient from 5 to
%                          4.5 [bar] and the second the gradient from 4.5
%                          to 3.5 [bar]. [Pa/sec]
%               prpi:      Updated pressure in pilot chambers [Pa]
%               ptarget:   [?]
%               segnof:    Sign to specify air flow from Driver's Brake 
%                          Valves (+1 for braking, -1 for releasing, the 
%                          opposite for end of train devices)
%               Sut:       Cross section of Driver's Brake Valves taking
%                          into account their operation mode [m^2]
%               t1:        Used for recharging. Represents the time needed
%                          to reach the target pressure in Brake Pipe 
%                          during releasing [s]

global Tamb r  % [s!] Global variables not used in this function. Can be removed

Cfq = []; grapi = []; segnof = []; Sut = []; t1 = []; k = 0;

ptarget = pfin;

%# fastindex
%for kk = 1:nDBV
for kk = find(pfin)
    %%% Define delay of manouvre
    %dly(kk) = DBVdata(3*k+1,3+jm);
    if isservice(kk) > 0
        DBVdRch =  DBVdata(3*k+3,:);      % Releasing DBV data [SBB]    
        % Recharge
        segnof(kk) = -1; 
        if typeDBV(kk) == -1              % For end of train device [n] [SBB]
            segnof(kk) = +1;
        end       
        %# fastindex
        Tbk = DBVdRch(3);                 % Time to achieve an increasing in pressure from 3.5 to 5 [bar] [SBB]
        
        if pfin(kk) < 5.5e5
            ptarget(kk) = pfin(kk);
        else
            ptarget(kk) = pfin(kk)+0.4e5; % Target pressure defined in function of manouvre inside servrec [?]            
        end
        grapi(kk,1) = ((5-3.5)*1e5)/Tbk; 
        t1(kk)      = ((pfin(kk)-0.2e5)-prpi(kk))/grapi(kk,1); 
        %# fastindex
        diaRch  = DBVdRch(1); 
        Cfq(kk) = DBVdRch(2);
        Sut(kk) = diaRch^2*pi*0.25;       % Cross section of the nozzle with fixed diameter 
        
    elseif isservice(kk) == -2
        DBVdDsc = DBVdata(3*k+2,:);
        % Service discharge
        segnof(kk) = +1;
        if typeDBV(kk) == -1
            segnof(kk) = -1;
        end    
        %# fastindex
        Tbk = DBVdDsc(3);                 % Time to achieve a decreasing in pressure from 5 to 3.5 [bar] [SBB]
        %# fastindex
        td1 = DBVdDsc(4);                 % Time to achieve a decreasing in pressure from 5 to 4.5 [bar] [SBB]
        
        ptarget(kk) = pfin(kk);           % Target pressure equal to final service brake pressure         
        grapi(kk,1) = (5.5e5-6e5)/td1;
        grapi(kk,2) = -1.0e5/(Tbk-td1);
        %# fastindex
        diaDsc  = DBVdDsc(1); 
        Cfq(kk) = DBVdDsc(2);  
        Sut(kk) = diaDsc^2*pi*0.25; % Cross section of the nozzle with fixed diameter  
        %prpi(kk) = 6e5;
    elseif isservice(kk) == -1
        DBVdRp     =  DBVdata(3*k+1,:); 
        segnof(kk) = +1;
        if typeDBV(kk) == -1
            segnof(kk) = -1;
        end    
        prpi(kk) = 1e5;
        %# fastindex
        diaRp   = DBVdRp(1); 
        Cfq(kk) = DBVdRp(2);          
        Sut(kk) = diaRp^2*pi*0.25; % Cross section of the nozzle with fixed diameter            
    elseif isservice(kk) == 0 % Added for dynamics
        Cfq = []; dly = []; grapi = []; ptarget = []; segnof = []; Sut = []; t1 = []; t = []; tBkon = []; 
    end  
    k = k+1;
end 


end