function [dQdt,drodt,dudt] = comp_SA(SA,dQdt,drodt,dt,dudt,dx,...
    iSA,P,pfin,Q,ro,S,segnoSA,T,u)

global Cms cv gam kpol mSA prac prSA r Tamb  

%# scalar nsca 

dpCGSA = (P(iSA)-(prSA+SA(4,:)))/1e5;
scatt = find(dpCGSA > 0);

if not(isempty(scatt))
    iSA = iSA(scatt); % Indexes of opened auxiliary reservoirs
    %# fastindex
    nsca = size(iSA,2); % Number of opened auxiliary reservoirs
    praSA = prSA(scatt)./P(iSA); % Pressure ratio
    % If pressure in main brake pipe is lower than pressure in auxiliary reservoir,
    % the air flows from auxiliary reservoir to the main brake pipe.
    appo = find(praSA > 1);
    appo2 = scatt(appo);
    praSA(appo) = 1./praSA(appo);
    %coeqSA = 0.8414-0.1002*praSA+0.8415*praSA.^2-3.9*praSA.^3+4.6001*praSA.^4-1.6827*praSA.^5;
    coeqSA = 0.72;
    effl = find(praSA > prac);
    Cm = Cms*ones(1,nsca);
    Cm(effl) = sqrt(2*gam/(r*(gam-1)))*sqrt(praSA(effl).^(2/gam)-praSA(effl).^((gam+1)/gam));
    %SSA = (dpCGSA(scatt)/(pfin/1e5)).*(12/1000).^2*pi*0.25;
    % FIXME: IT ASSUMES THAT THE FINAL PRESSURE IS THE SAME FOR ALL THE LOCO
    SSA = (dpCGSA(scatt)/((pfin(1)-1e5)/1e5)).*(SA(3,scatt)).^2*pi*0.25;
    %SSA = (dpCGSA(scatt)/0.7).*(4.2/1000).^2*pi*0.25;
    dmSA = coeqSA.*SSA.*Cm.*P(iSA)./sqrt(T(iSA)); % Lateral mass flow rate    
    % Updating lateral mass flow for the reservoirs that have a pressure greater than
    % the pressure of the main brake pipe.
    dmSA(appo) = coeqSA(1,appo).*SSA(1,appo2).*Cm(1,appo).*prSA(1,appo2)/sqrt(Tamb); % Lateral mass flow rate
    segnoSA(appo2) = -1;
    dmSA = segnoSA(scatt).*dmSA;

    dPdt = kpol * prSA(scatt).*(dmSA./mSA(scatt));
    prSA(scatt) = prSA(scatt)+dPdt*dt;
    roSA = prSA(scatt)/(r*Tamb);
    mSA(scatt) = roSA.*SA(2,scatt);
    tvc = Tamb*ones(1,nsca); velc = dmSA./(roSA.*SSA); 

    % Derivatives update
%     drodt(iSA) = drodt(iSA) - 0.5*dmSA./(dx*S(iSA));
%     drodt(iSA+1) = drodt(iSA+1) - 0.5*dmSA./(dx*S(iSA+1));
%     dudt(iSA) = dudt(iSA) + 0.5*dmSA.*u(iSA)./(dx*S(iSA).*ro(iSA));
%     dudt(iSA+1) = dudt(iSA+1) + 0.5*dmSA.*u(iSA+1)./(dx*S(iSA+1).*ro(iSA+1));
%     dQdt(iSA) = dQdt(iSA) - 0.5*dmSA.*(-Q(iSA)+(cv+r)*tvc+0.5*velc.^2)./...
%         (dx*S(iSA).*ro(iSA));
%     dQdt(iSA+1) = dQdt(iSA+1) - 0.5*dmSA.*(-Q(iSA+1)+(cv+r)*tvc+0.5*velc.^2)./...
%         (dx*S(iSA+1).*ro(iSA+1));
    
    drodt(iSA-1) = drodt(iSA-1) - 0.5*dmSA./(dx*S(iSA));
    drodt(iSA) = drodt(iSA) - 0.5*dmSA./(dx*S(iSA+1));
    dudt(iSA-1) = dudt(iSA-1) + 0.5*dmSA.*u(iSA)./(dx*S(iSA).*ro(iSA));
    dudt(iSA) = dudt(iSA) + 0.5*dmSA.*u(iSA+1)./(dx*S(iSA+1).*ro(iSA+1));
    dQdt(iSA-1) = dQdt(iSA-1) - 0.5*dmSA.*(-Q(iSA)+(cv+r)*tvc+0.5*velc.^2)./...
        (dx*S(iSA).*ro(iSA));
    dQdt(iSA) = dQdt(iSA) - 0.5*dmSA.*(-Q(iSA+1)+(cv+r)*tvc+0.5*velc.^2)./...
        (dx*S(iSA+1).*ro(iSA+1));    
end;