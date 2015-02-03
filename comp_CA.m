function [dQdt,drodt,dudt] = comp_CA(CA,dQdt,drodt,dt,dudt,dx,...
    iCA,P,Q,ro,S,segnoCA,T,u)

% COMP_CA This function updates the time partial derivatives of Q, ro and u
%         around the Brake Pipe sections where open Acceleration Chambers
%         are located
%
% INPUTS  CA:      Matrix with Acceleration Chamber data (rows)
%         dQdt:    Partial derivative of specific energy wrt time
%         drodt:   Partial derivative of density wrt time  
%         dt:      Current time step
%         dudt:    Partial derivative of flow speed wrt time
%         dx:      Discretization of Brake Pipe
%         iCA:     Index with sections where Acceleration Chambers are 
%                  connected to the Brake Pipe.
%         P:       Absolute Brake Pipe pressure in each pipe section [Pa]
%         Q:       Specific energy of each Brake Pipe section [J/Kg]
%         ro:      Density of each Brake Pipe section [Kg/m^3] 
%         S:       Cross-section of each Brake Pipe section [m^2]
%         segnoCA: Sign to indicate direction of air flow between Brake 
%                  Pipe and Accelaration Chamber
%         T:       Temperature of each Brake Pipe section [K]
%         u:       Flow speed (axial velocity) of each pipe section [m/s]
%
% OUTPUTS dQdt:    Updated derivative including mass flow rate terms
%         drodt:   Updated derivative
%         dudt:    Updated derivative

global Cms cv eftPt gam kpol mCA prac prCA r Tamb

%# scalar nsca 

%scatt = find(P(CA(1,:)) < CA(4,:));

% Maching to control the activation and the closing of Acceleration
% Chambers
appo1 = find(P(CA(1,:)) < CA(4,:));
appo2 = find(P(CA(1,appo1)) >= CA(6,appo1));
scatt = appo1(appo2);

% To consider the action of Acceleration Chamber only during the initial
% stage of braking
% appo = find(eftPt(scatt));
% scatt = scatt(appo);

if not(isempty(scatt))
    iCA   = iCA(scatt);          % Indexes of opened Acceleration Chambers
    %# fastindex
    nsca  = size(iCA,2);         % Number of opened Acceleration Chambers
    praCA = prCA(scatt)./P(iCA); % Pressure ratio
    
    % If pressure in Brake Pipe is lower than pressure in Acceleration 
    % Chamber, the air flows from the Acceleration Chamber to the pipe.
    %# fastindex
    appo        = find(praCA > 1);
    appo2       = scatt(appo);
    praCA(appo) = 1./praCA(appo);
    coeqCA      = 0.8414-0.1002*praCA+0.8415*praCA.^2-3.9*praCA.^3+4.6001*praCA.^4-1.6827*praCA.^5;
    effl        = find(praCA > prac);
    Cm          = Cms*ones(1,nsca);
    Cm(effl)    = sqrt(2*gam/(r*(gam-1)))*sqrt(praCA(effl).^(2/gam)-praCA(effl).^((gam+1)/gam));
    dmCA        = coeqCA.*CA(3,scatt).*Cm.*P(iCA)./sqrt(T(iCA)); % Lateral mass flow
    
    % Updating lateral mass flow for the Acceleration Chambers that have a  
    % pressure greater than the pressure of the Brake Pipe.
    dmCA(appo)     = coeqCA(1,appo).*CA(3,appo2).*Cm(1,appo).*prCA(1,appo2)/sqrt(Tamb); % Lateral mass flow
    segnoCA(appo2) = -1;
    dmCA           = segnoCA(scatt).*dmCA;
    dPdt           = kpol * prCA(scatt).*(dmCA./mCA(scatt));
    prCA(scatt)    = prCA(scatt)+dPdt*dt;
    roCA           = prCA(scatt)/(r*Tamb);
    mCA(scatt)     = roCA.*CA(2,scatt);
    
    % TODO: check the temperature and the velocity when the air flows from 
    % the Acceleration Chamber to the Brake Pipe.
    tvc  = Tamb*ones(1,nsca); 
    velc = dmCA./(roCA.*CA(3,scatt)); 

    % Derivatives update
%     drodt(iCA) = drodt(iCA) - 0.5*dmCA./(dx*S(iCA));
%     drodt(iCA+1) = drodt(iCA+1) - 0.5*dmCA./(dx*S(iCA+1));
%     dudt(iCA) = dudt(iCA) + 0.5*dmCA.*u(iCA)./(dx*S(iCA).*ro(iCA));
%     dudt(iCA+1) = dudt(iCA+1) + 0.5*dmCA.*u(iCA+1)./(dx*S(iCA+1).*ro(iCA+1));
%     dQdt(iCA) = dQdt(iCA) - 0.5*dmCA.*(-Q(iCA)+(cv+r)*tvc+0.5*velc.^2)./...
%         (dx*S(iCA).*ro(iCA));
%     dQdt(iCA+1) = dQdt(iCA+1) - 0.5*dmCA.*(-Q(iCA+1)+(cv+r)*tvc+0.5*velc.^2)./...
%         (dx*S(iCA+1).*ro(iCA+1));
    
    drodt(iCA-1) = drodt(iCA-1) - 0.5*dmCA./(dx*S(iCA));
    drodt(iCA)   = drodt(iCA) - 0.5*dmCA./(dx*S(iCA+1));
    dudt(iCA-1)  = dudt(iCA-1) + 0.5*dmCA.*u(iCA)./(dx*S(iCA).*ro(iCA));
    dudt(iCA)    = dudt(iCA) + 0.5*dmCA.*u(iCA+1)./(dx*S(iCA+1).*ro(iCA+1));
    dQdt(iCA-1)  = dQdt(iCA-1) - 0.5*dmCA.*(-Q(iCA)+(cv+r)*tvc+0.5*velc.^2)./(dx*S(iCA).*ro(iCA));
    dQdt(iCA)    = dQdt(iCA) - 0.5*dmCA.*(-Q(iCA+1)+(cv+r)*tvc+0.5*velc.^2)./(dx*S(iCA+1).*ro(iCA+1));    
end
