function [dmf,d2Qdt2o,dQdto,d2rodt2o,drodto,dt,d2udt2o,dudto,DT,grapi,mSA,...
    P,pBCtpSR,PBP,pCdG_0,pCF,pCFm,pCFmnv0,Pmed,Pmedo,prpi,prSA,ptarget,Q,ro,T,tBkon,u,UBP] = comp_pressure_tpSR(CA,Cfq,cstPol,cv,CVactv,...
    D,DT,dLC,dmf,dTF,d2Qdt2o,dQdto,d2rodt2o,drodto,dt,d2udt2o,dudto,dx,EOT,FOT,grapi,...
    iCV,isservice,iV,j0,j1,jj,K,kpolSA,lam,Lmedia,mSA,n,nCV,nDBV,P,pBCt,pCdG_0,pCFm,pCFmnv0,pfin,Pmed,Pmedo,prpi,prSA,ptarget,...
    Q,r,ro,rugrel,S,SA,s,segnoCA,segnoSA,segnof,splo,SR,Sut,t,T,Tamb,tBkon,t1,typeDBV,u,viscosita,vmisCF,Vbc,vDBV,vUnC)

% Sometimes the variable t is a vector.
told = t(1); tfin = t(splo)+SR; t = told; % tfin is the finishing time of integration.
% Initializations
ite = 0; itemed = 0; itenv = 0; itenvo = -1;
un = u; Pn = P; Tn = T; ron = ro; Qn = Q;

% 1e-9 can be put in input as an advanced time tolerance for pneumatic/dynamic
% synchronization
while t - tfin < -1e-9

    % DEFINING NEW DATA FOR BRAKE PIPE PRESSURE DEVELOPING FOLLOWING INPUT DBV
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if itenv == itenvo+1
        [dmf,grapi,P,Pmed,Pmedo,prpi,ptarget,Q,ro,T,u,velc] = boundaryCond2(Cfq,...
            dmf,dt,grapi,isservice,iV,nDBV,P,pfin,Pmed,Pmedo,prpi,ptarget,Q,ro,S,...
            segnof,Sut,t,T,t1,typeDBV,u,vDBV);

        % computation of the pressure losses
        Re = abs(ro.*u.*D)./viscosita+1e-15;
        unosuRe = Re.^-1;
        % 1000 provides a point continuity even if not a slope continuity among the
        % turbolent and laminar flux
        laminare = find(Re < 1000); 
        A = -2*log10(rugrel/3.7+12.*unosuRe);
        B = -2*log10(rugrel/3.7+2.51*A.*unosuRe);
        C = -2*log10(rugrel/3.7+2.51*B.*unosuRe);
        f = (A - ((B-A).^2) ./ (C-2*B+A) ).^-2;
        f(laminare) = 64.*unosuRe(laminare);
        f(Re < 1) = 0;
        % Total pressure losses quantity
        tau = -sign(u) .* (f  + K.*D/dx) .* (0.5*u.^2); %[m^2/s^2]

        % Thermal flux through pipe
        flussot = lam*(Tamb - T)/s;

        % Calculation of various space derivatives
        drodx = ( ro(j0)-ro(j1) ) ./ Lmedia;
        duSdx = ( u(j0).*S(j0)-u(j1).*S(j1) ) ./ Lmedia;
        dudx = ( u(j0)-u(j1) ) ./ Lmedia;
        dpdx = ( P(j0)-P(j1) ) ./ Lmedia;
        dQdx = ( Q(j0)-Q(j1) ) ./ Lmedia;
        drouSdx = ( ro(j0).*u(j0).*S(j0)-ro(j1).*u(j1).*S(j1) ) ./ Lmedia;
        dTdx = ( T(j0)-T(j1) ) ./ Lmedia;

        % Calculation of ro, u and Q time derivatives using the
        % relations obtained from mass, momentum and energy conservation
        romed = 0.25*(ro(jj-1) + ro(jj)*2 + ro(jj+1));
        umed = 0.25*(u(jj-1) + u(jj)*2 + u(jj+1));
        Qmed = 0.25*(Q(jj-1) + Q(jj)*2 + Q(jj+1));
        Tmed = ( Qmed - 0.5*umed.^2 ) / cv;
        
        drodt = -(  umed.*drodx  +  (romed./S(jj)).*duSdx  );
        dudt = tau(jj)./D(jj) - dpdx ./ romed - umed.*dudx;
        dQdt = - umed.*( dQdx + r*dTdx ) - r*Tmed.*drouSdx ./(romed.*S(jj)) + 4*flussot(jj)./(romed.*D(jj)) - tau(jj).*umed./D(jj);
        
        % It menages uncoupled BP, by setting this boudary condition
        dudt(vUnC) = 0;


        % DEFINING NEW DATA FOR BRAKE PIPE PRESSURE DEVELOPING
        % FOLLOWING ACTION OF CONTROL VALVE
        if CVactv && min(isservice) < 0 && abs((max(P) - 1.01325e5)/1.01325e5) > 1e-3
            [dQdt,drodt,dudt] = comp_CA(CA,dQdt,drodt,dt,dudt,dx,...
                iCV,P,Q,ro,S,segnoCA,T,u);
        end;
        if CVactv && max(isservice) > 0
            [dQdt,drodt,dudt] = comp_SA(SA,dQdt,drodt,dt,dudt,dx,...
                iCV,P,pfin,Q,ro,S,segnoSA,T,u);
        end;

        % Setting the boundary conditions in case of lateral DBV
        for kk = vDBV
            if typeDBV(kk) == 1
                % Derivatives update
                drodt(iV(1,kk)-1) = drodt(iV(1,kk)-1) - 0.5*dmf(kk)./(dx*S(iV(1,kk)));
                drodt(iV(2,kk)-1) = drodt(iV(2,kk)-1) - 0.5*dmf(kk)./(dx*S(iV(2,kk)));
                dudt(iV(1,kk)-1) = dudt(iV(1,kk)-1) + 0.5*dmf(kk).*u(iV(1,kk))./(dx*S(iV(1,kk)).*ro(iV(1,kk)));
                dudt(iV(2,kk)-1) = dudt(iV(2,kk)-1) + 0.5*dmf(kk).*u(iV(2,kk))./(dx*S(iV(2,kk)).*ro(iV(2,kk)));
                dQdt(iV(1,kk)-1) = dQdt(iV(1,kk)-1) - 0.5*dmf(kk).*(-Q(iV(1,kk))+(cv+r)*Tamb+0.5*velc(kk)^2)./...
                    (dx*S(iV(1,kk)).*ro(iV(1,kk)));
                dQdt(iV(2,kk)-1) = dQdt(iV(2,kk)-1) - 0.5*dmf(kk).*(-Q(iV(2,kk))+(cv+r)*Tamb+0.5*velc(kk)^2)./...
                    (dx*S(iV(2,kk)).*ro(iV(2,kk)));
            end;
        end;
    end;

    % CALCULATION OF NEW VALUES FOLLOWING INTEGRATION OF
    % CONSERVATION EQUATION
    d2rodt2 = (drodt-drodto)/dt; d2udt2 = (dudt-dudto)/dt; d2Qdt2 = (dQdt-dQdto)/dt;
    d3rodt3 = (d2rodt2-d2rodt2o)/dt; d3udt3 = (d2udt2-d2udt2o)/dt; d3Qdt3 = (d2Qdt2-d2Qdt2o)/dt;

    % Mass conservation
    ron(jj) = ro(jj) + dt * drodt + d2rodt2 * 0.5*dt^2 + d3rodt3 * dt^3/6;

    % Momentum conservation
    un(jj) = u(jj) + dt * dudt + d2udt2 * 0.5*dt^2 + d3udt3 * dt^3/6;
    % Limiting to the actual speed of sound
    Vs = sqrt(1.4*r*T);
    appo = abs(un) > Vs;
    un(appo) = Vs(appo).*sign(un(appo));
    % Updating the speed for uncloupled bp
    un(vUnC) = 0;


    % Energy conservation
    Qn(jj) = Q(jj) + dt * dQdt + d2Qdt2 * 0.5*dt^2 + d3Qdt3 * dt^3/6;

    % New temperature calculation
    Tn(jj) = ( Qn(jj) - 0.5*un(jj).^2 ) / cv;

    % Pressure calculation using perfect gas law
    Pn(jj) = r * ron(jj).*Tn(jj);


    % UPDATING
    dprel = max(abs( (P-Pn)./Pn ) ); % Maximum relative pressure increment
    if not(isreal(Tn)) || not(isreal(ron)) || not(isreal(Pn)) || not(isreal(un))
        error('Unreal numbers!');
    end;
    delta0 = 1e-3; % This is another advanced parameter that can be put in input
    % The time step is changed so that if dprel is lower than delta0, the time step
    % is increased, otherwise it is decreased.
    dtp = dt*(delta0/(dprel+1e-6))^(1/3); % 1e-6 is used to manage the situations where dprel is zero
    dt = min([dtp DT]); % DT is the maximum time step

    if (dt > dtp || dprel <= delta0 || dt == DT) || ite <= 1
        
        % The iteration is correct
        dt = min([dt tfin-t]); % New time step to have synchronization
        ron(jj) = ro(jj) + dt * drodt + d2rodt2 * 0.5*dt^2 + d3rodt3 * dt^3/6;
        un(jj) = u(jj) + dt * dudt + d2udt2 * 0.5*dt^2 + d3udt3 * dt^3/6;
        %Limiting to speed of sound
        Vs = sqrt(1.4*r*T);
        appo = abs(un) > Vs;
        un(appo) = Vs(appo).*sign(un(appo));
        Qn(jj) = Q(jj) + dt * dQdt + d2Qdt2 * 0.5*dt^2 + d3Qdt3 * dt^3/6;
        Tn(jj) = ( Qn(jj) - 0.5*un(jj).^2 ) / cv;
        Pn(jj) = r * ron(jj).*Tn(jj);
        
        % Ghost sections updating
        un(1) = un(2);     Pn(1) = Pn(2);   ron(1) = ron(2); Tn(1) = Tn(2);   Qn(1) = Qn(2);
        un(n) = un(n-1); Pn(n) = Pn(n-1); ron(n) = ron(n-1); Tn(n) = Tn(n-1); Qn(n) = Qn(n-1);
        if FOT == 0
            un(1) = 0;
        end;

        if EOT == 0
            un(n) = 0;
        end;

        % Updating system parameters
        P = Pn; u = un; T = Tn; ro = ron; Q = Qn;

        drodto = drodt; dudto = dudt; dQdto = dQdt;
        d2rodt2o = d2rodt2; d2udt2o = d2udt2; d2Qdt2o = d2Qdt2;

        ite = ite+1;
        itemed = itemed+1;
        t = t+dt;
        itenvo = itenv-1;
    else
        itenv = itenv+1;
    end;
    if DT == dt, DT = 1.01*DT; else DT = 0.99*DT; end

end;
% Relative pressure in BP expressed in bar, for each vehicle
PBP = 1e-5*0.25*(Pn(vmisCF(1:end)+1)+Pn(vmisCF(1:end)-1)+ 2*Pn(vmisCF(1:end))) - 1; 
% Air speed in BP, for each vehicle
UBP = 0.25*(un(vmisCF+1)+un(vmisCF-1)+ 2*un(vmisCF));

pCF = pBCt(2:end); % Pressure in brake cylinder
%======================================================================
% Calculation of brake cylinder pressure starting from brake pipe
% pressure
dtCV = t-told;
% Relative pressure in BP expressed in bar, for each vehicle at tfin
pCdG = (0.25*(Pn(vmisCF(1:end)+1)+Pn(vmisCF(1:end)-1)+ 2*Pn(vmisCF(1:end))))./1e5-1;

[pCF,pCFm,tBkon] = CalcCF(dLC,dtCV,dTF,isservice,nCV,pCdG,pCdG_0,pCF,pCFm,tBkon,t);

%==========================================
% Calculation of auxiliary reservoir pressure during braking (IT IS NOT SUPPORTED,
% BUT IT HAS TO BE USED FOR RELEASING)
pCFa = pCF;
appo = find(pCF ~= 0);
pCFa(appo) = pCFa(appo)+1;
ifren = find(pCdG_0-pCdG>0); % Indicies of vehicles that are braking (maybe a tolerance is needed)
if not(isempty(ifren))
    dmSA = (((pCFa(ifren)-pCFmnv0(ifren))*1e5).*Vbc(ifren)/r/Tamb)/dtCV;
    dmSA(dmSA <0) = 0; % to avoid the charging of AR when it's closed during braking
    dPdt = kpolSA * prSA(ifren).*(dmSA./mSA(ifren));
    prSA(ifren) = prSA(ifren)-dPdt*dtCV;
    Tsa = cstPol(ifren)./prSA(ifren).^((1-kpolSA)/kpolSA);
    roSA = prSA(ifren)./(r*Tsa);
    mSA(ifren) = roSA.*SA(2,ifren);
end;
pCFmnv0 = pCFa;
pCdG_0 = pCdG; % Updating the pressure in BP at told
%===========================================

pBCtpSR = [tfin pCF]; % Pressure in brake cylinders at t plus a sampling rate
end
