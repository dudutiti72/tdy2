function [PbcM,PbcT,PM,PMT,PsaM,roM,tempo,tempoT,TM,vdtmed,uM] = prescg2(CA,CVactv,D,...
    DBVdata,dLC,Dpy,DT,dt,dTF,dQdto,d2Qdt2o,drodto,d2rodt2o,dudto,d2udt2o,dx,EOT,FOT,...
    iCV,iDCVa,iV,j0,j1,jj,K,L,Lmedia,n,nCV,nDBV,nsim,P1,pCF,rugrel,segnoCA,segnoSA,...
    SA,SR,typeDBV,tvis,Vbc,VhcMa,vManovra,vmis,vmisCF)

%# scalar CVactv delta0 dprel Dpy dt dtmed dtp DTfix EOT FOT ii isservice ite itemed itenv itenvo itev jm nCV ndt nManv nsim 
%# scalar P1 pfin SR t t0mnv told Tsim u1

global mCA prCA % used in comp_CA & initialManv
global mSA prSA % used in comp_SA
global pCF_0 %used in CalcCF & initialManv
global cv lam r s Tamb viscosita  % generic constant
global cRch1 cRch2 % used in recharge 

% This function performs the solution solving the conservation equations

% Initialization parameters 
%# fastindex
u1 = 0; u = u1*ones(1,n); un = u;
P = P1*ones(1,n); Pn = P; 
T = Tamb*ones(1,n); Tn = T;
ro = P./(r*T); ron = ro;
S = 0.25*D.^2*pi;
V = L.*S;
Q = cv*T+0.5*u.^2; Qn = Q;

% Variables for display
tempo = zeros(1,nsim); vdtmed = zeros(1,nsim);
%# fastindex
ii = 1; ndt = tvis;

% Variables to control integration
cltvis = clock; 
%# fastindex
ite = 0; itemed = 0; dtmed = 0; itenv = 0; itenvo = -1; itev = 0;
TTsim = sum(vManovra(:,3));

% Initialization measured quantity
PM = ones(nsim,1)*Pn(vmis); uM = ones(nsim,1)*un(vmis); roM = ones(nsim,1)*ron(vmis); 
TM = ones(nsim,1)*Tn(vmis);  tempo = [0:SR:TTsim]; PbcM = []; PsaM = []; PbcT = []; PMT = []; tempoT = []; 
%MT = zeros(nsim,1); Mu = zeros(nsim,1);

% Initialization parameters for brake cylinder pressure calculation
%# fastindex
if CVactv 
    % acceleration chambers 
    prCA = CA(5,:);
    mCA = ( CA(2,:).*prCA ) / Tamb / r;
    % auxiliary reservoirs
    prSA = SA(5,:);
    mSA = ( SA(2,:).*prSA ) / Tamb / r; 
    P0 = prSA; T0 = Tamb;
    kpolSA = 1.3; cstPol = T0*P0.^((1-kpolSA)/kpolSA);     
    % Initializing output parameters control valve 
    PbcM = ones(nsim,1)*zeros(1,size(vmis,2));
    PbcM(:,iDCVa) = ones(nsim,1)*pCF(VhcMa);
    PsaM = ones(nsim,1)*((prSA(VhcMa(1))-1e5)/1e5)*ones(1,size(vmis,2));
    PsaM(:,iDCVa) = ones(nsim,1)*(prSA(VhcMa)-1e5)/1e5;
    PbcT = ones(nsim,1)*pCF;
    PMT = ones(nsim,1)*Pn(iCV);
    % Assignement to consider pressure in BC refered to nominal 3.8 bar
    pCFm = pCF;
    % Termini per calcolo pressioni in SA durante frenatura   
    pCFa = pCF;
    appo = find(pCFa ~= 0);
    pCFa(appo) = pCFa(appo)+1;
    pCFmnv0 = pCFa;   
    % Initialization of BP pressure close to control valve 
    pCdG_0 = (0.25*(Pn(vmisCF(1:end)+1)+Pn(vmisCF(1:end)-1)+ 2*Pn(vmisCF(1:end))))./1e5-1;
end;    

% Variable to control the flow through driver's brake valve
prpi = P1*ones(1,nDBV); % Starting value of external back pressure to calculate mass flow through DBV 
Pmed = P1*ones(1,nDBV); % Starting value of BP pressure to calculate mass flow rate through DBV (BP pressure close to DBV)
Pmedo = P1*ones(1,nDBV); %Pressure at time t-dt to calculate gradient of BP raising during releasing 
dmf = zeros(1,nDBV); %mass flow rate through DBV

% Variables of driver's brake valve to display
vprpi = P1*ones(nsim,nDBV); if vManovra(1,1) == -1 vprpi(1,:) = 1e5; end; % external back pressure
vPmed = P1*ones(nsim,nDBV); % BP pressure close to DBV
vaamp = zeros(nsim,nDBV); % mass flow rate through DBV

% Variables to control manoeuvres
%# fastindex
t = 0; % time elapsed at manoeuvre °1
%# fastindex
t0mnv = 0; % time elapsed at manoeuvre 0
%# fastindex
jm = 0; % index to manage the different manoeuvres
%# fastindex
DTfix = 0; %parameter to define type integration method: 0 variable integration step,  1 fixed integration step
%# fastindex
nManv = size(vManovra,1); % number of manoeuvres
%# fastindex
while jm < nManv 
    %# fastindex
    jm = jm+1;
    
    cRch1 = 0; cRch2 = 0;
    
    % Running manouvre (no calculation needs)
    if vManovra(jm,1) == 0
        %# fastindex
        isservice = 0;
        disp('On running')
        %# fastindex
        t = t+vManovra(jm,3); % % simulation time updating     
         %# fastindex
        ii = ii+round(vManovra(jm,3)/SR); % number of simulation updating      
        jm = jm+1; % activation of new manoeuvre
        if jm > nManv
            break
        end;
    end;    

    t0mnv = t0mnv+t; % elapsed time at previous manoeuvre     
    
    % Updating parameters following new manoeuvre
    %# fastindex
    Tsim = vManovra(jm,3);
    %# fastindex
    isservice = vManovra(jm,1);
    %# fastindex
    pfin = vManovra(jm,2);
    
    % Redefining parameter of DBV and CV following the actual manouvre
    [Cfq,dly,grapi,prpi,ptarget,segnof,Sut,t1,t,tBkon] = initialManv2(CA,CVactv,DBVdata,...
    dt,isservice,jm,nCV,nDBV,pCF,pfin,Pn,prpi,typeDBV);
    if t<SR,t = 0; end
   %# fastindex
    while t < Tsim
        %# fastindex
        ii = ii+1;    
        %# fastindex
        told = t;
        %aMu = 0; % Mass out in a sample rate SR
        %# fastindex
        %while dt > 0 && (t-told - SR < -1e-9)
        while t-told - SR < -1e-9
            
            % DEFINING NEW DATA FOR BRAKE PIPE PRESSURE DEVELOPING
            % FOLLOWING INPUT DBV
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            %# fastindex
            if itenv == itenvo+1
                  [dmf,grapi,P,Pmed,Pmedo,prpi,ptarget,Q,ro,T,u,velc] = boundaryCond2(Cfq,...
                      dly,dmf,dt,grapi,isservice,iV,nDBV,P,pfin,Pmed,Pmedo,prpi,ptarget,Q,ro,S,...
                      segnof,Sut,t,T,t1,typeDBV,u);             
                
                % CALCULATION OF NEW DERIVATIVES USING VALUE AT STEP I-1
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Contribution of concentrated or distributed pressure
                % losses along the BP
                % Calculation of friction factor
%                 us = u; %new
%                 fumed = find(abs(diff(P)) > 0.1e5); %new
%                 if not(isempty(fumed)) %new
%                     %if any(fumed+1 == jdx), fumed((fumed+1) == jdx) = 0; end;
%                     %if any(fumed+1 == jsx), fumed((fumed+1) == jsx) = 0; end;
%                     %fumed(fumed == 0) = [];
%                     u(fumed+1) = 0.25*(u(fumed)+2*u(fumed+1)+u(fumed+2)); %new
%                 end; %new
%                 umed = u;
%                 u(jj) = 0.25*(u(jj-1) + u(jj)*2 + u(jj+1));
                Re = abs(ro.*u.*D)./viscosita+1e-15;
                unosuRe = Re.^-1;
                laminare = find(Re < 1000); % Si usa 1000 per ragioni numeriche, vale a dire per avere continuità di punto (anche se non di tangenza)
                A = -2*log10(rugrel/3.7+12.*unosuRe);
                B = -2*log10(rugrel/3.7+2.51*A.*unosuRe);
                C = -2*log10(rugrel/3.7+2.51*B.*unosuRe);
                f = (A - ((B-A).^2) ./ (C-2*B+A) ).^-2;
                f(laminare) = 64.*unosuRe(laminare);
                f(Re < 1) = 0;
                % Total pressure losses quantity
                tau = -sign(u) .* (f  + K.*D/dx) .* (0.5*u.^2); %[m^2/s^2]              
%                 u = umed;
%                 u = us; %new
                
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
%                 drodt = -(  u(jj).*drodx  +  (ro(jj)./S(jj)).*duSdx  );
%                 dudt = tau(jj)./D(jj) - dpdx ./ ro(jj) - u(jj).*dudx;
%                 dQdt = - u(jj).*( dQdx + r*dTdx ) - r*T(jj).*drouSdx ./(ro(jj).*S(jj)) + 4*flussot(jj)./(ro(jj).*D(jj)) - tau(jj).*u(jj)./D(jj);
                romed = 0.25*(ro(jj-1) + ro(jj)*2 + ro(jj+1));
                umed = 0.25*(u(jj-1) + u(jj)*2 + u(jj+1));
                Qmed = 0.25*(Q(jj-1) + Q(jj)*2 + Q(jj+1));
                Tmed = ( Qmed - 0.5*umed.^2 ) / cv;
                drodt = -(  umed.*drodx  +  (romed./S(jj)).*duSdx  );
                dudt = tau(jj)./D(jj) - dpdx ./ romed - umed.*dudx;
                dQdt = - umed.*( dQdx + r*dTdx ) - r*Tmed.*drouSdx ./(romed.*S(jj)) + 4*flussot(jj)./(romed.*D(jj)) - tau(jj).*umed./D(jj);


                % DEFINING NEW DATA FOR BRAKE PIPE PRESSURE DEVELOPING
                % FOLLOWING ACTION OF CONTROL VALVE
                %# fastindex
                if CVactv & isservice < 0
                    [dQdt,drodt,dudt] = comp_CA(CA,dQdt,drodt,dt,dudt,dx,...
                        iCV,P,Q,ro,S,segnoCA,T,u);
                elseif CVactv & isservice > 0
                    [dQdt,drodt,dudt] = comp_SA(SA,dQdt,drodt,dt,dudt,dx,...
                        iCV,P,pfin,Q,ro,S,segnoSA,T,u);
                end;

                % Setting the boundary conditions in case of lateral DBV
                %# fastindex
                for kk = 1:nDBV
                    if typeDBV(kk) == 1
                        % Derivatives update
%                         drodt(iV(1,kk)) = drodt(iV(1,kk)) - 0.5*dmf(kk)./(dx*S(iV(1,kk)));
%                         drodt(iV(2,kk)) = drodt(iV(2,kk)) - 0.5*dmf(kk)./(dx*S(iV(2,kk)));
%                         dudt(iV(1,kk)) = dudt(iV(1,kk)) + 0.5*dmf(kk).*u(iV(1,kk))./(dx*S(iV(1,kk)).*ro(iV(1,kk)));
%                         dudt(iV(2,kk)) = dudt(iV(2,kk)) + 0.5*dmf(kk).*u(iV(2,kk))./(dx*S(iV(2,kk)).*ro(iV(2,kk)));
%                         dQdt(iV(1,kk)) = dQdt(iV(1,kk)) - 0.5*dmf(kk).*(-Q(iV(1,kk))+(cv+r)*Tamb+0.5*velc(kk).^2)./...
%                             (dx*S(iV(1,kk)).*ro(iV(1,kk)));
%                         dQdt(iV(2,kk)) = dQdt(iV(2,kk)) - 0.5*dmf(kk).*(-Q(iV(2,kk))+(cv+r)*Tamb+0.5*velc(kk).^2)./...
%                             (dx*S(iV(2,kk)).*ro(iV(2,kk)));
                        
                        drodt(iV(1,kk)-1) = drodt(iV(1,kk)-1) - 0.5*dmf(kk)./(dx*S(iV(1,kk)));
                        drodt(iV(2,kk)-1) = drodt(iV(2,kk)-1) - 0.5*dmf(kk)./(dx*S(iV(2,kk)));
                        dudt(iV(1,kk)-1) = dudt(iV(1,kk)-1) + 0.5*dmf(kk).*u(iV(1,kk))./(dx*S(iV(1,kk)).*ro(iV(1,kk)));
                        dudt(iV(2,kk)-1) = dudt(iV(2,kk)-1) + 0.5*dmf(kk).*u(iV(2,kk))./(dx*S(iV(2,kk)).*ro(iV(2,kk)));
%                         Qmed = (Q(iV(1,kk))+Q(iV(2,kk)))*0.5;
%                         dQdt(iV(1,kk)-1) = dQdt(iV(1,kk)-1) - 0.5*dmf(kk).*(-Qmed+(cv+r)*Tamb+0.5*velc(kk)^2)./...
%                           (dx*S(iV(1,kk)).*ro(iV(1,kk)));
%                         dQdt(iV(2,kk)-1) = dQdt(iV(2,kk)-1) - 0.5*dmf(kk).*(-Qmed+(cv+r)*Tamb+0.5*velc(kk)^2)./...
%                           (dx*S(iV(2,kk)).*ro(iV(2,kk)));                        
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
            %Limiting to speed of sound
            Vs = sqrt(1.4*r*T);
            appo = abs(un) > Vs;
            un(appo) = Vs(appo).*sign(un(appo));

            % Energy conservation
            Qn(jj) = Q(jj) + dt * dQdt + d2Qdt2 * 0.5*dt^2 + d3Qdt3 * dt^3/6;

            
            
            % Parameters are computed as mean values
            %ron(jj) = 0.25*(ron(jj-1) + ron(jj)*2 + ron(jj+1));
            %un(jj) = 0.25*(un(jj-1) + un(jj)*2 + un(jj+1));
            %Qn(jj) = 0.25*(Qn(jj-1) + Qn(jj)*2 + Qn(jj+1));
                


            % New temperature calculation
            Tn(jj) = ( Qn(jj) - 0.5*un(jj).^2 ) / cv;

            % Pressure calculation using perfect gas law
            Pn(jj) = r * ron(jj).*Tn(jj);

            
            
            % % Ghost sections updating
            % un(1) = un(2);     Pn(1) = Pn(2);   ron(1) = ron(2); Tn(1) = Tn(2);   Qn(1) = Qn(2); % Qn(1) = cv*Tn(1) + 0.5*un(1)^2; %
            % un(n) = un(n-1); Pn(n) = Pn(n-1); ron(n) = ron(n-1); Tn(n) = Tn(n-1); Qn(n) = Qn(n-1);
            % %# fastindex
            % if FOT == 0
            %     un(1) = 0;
            % end;
            %
            % if EOT == 0
            %     un(n) = 0;
            % end;
            
            
            
            % UPDATING
            %dprel = max(abs( (P(jj)-Pn(jj))./Pn(jj) ) );
            %# fastindex
            dprel = max(abs( (P-Pn)./Pn ) );
            %delta0 = 1e-2;
            %# fastindex
            delta0 = 1e-3;
            if not(isreal(Tn)) | not(isreal(ron)) | not(isreal(Pn)) | not(isreal(un))
                error('Non real quantity');
            end;
            %# fastindex
            dtp = dt*(delta0/(dprel+1e-6))^(1/3);
            %# fastindex
            dt = min([dtp DT]);
            %dt = 1e-5;
            
            %# fastindex
            if (dt > dtp | dprel <= delta0 | dt == DT) | ite <= 1 | DTfix > 0

                % The iteration is correct
                dt = min([dt told+SR-t]); % New time step to have synchronization
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
                un(1) = un(2);     Pn(1) = Pn(2);   ron(1) = ron(2); Tn(1) = Tn(2);   Qn(1) = Qn(2); % Qn(1) = cv*Tn(1) + 0.5*un(1)^2; %
                un(n) = un(n-1); Pn(n) = Pn(n-1); ron(n) = ron(n-1); Tn(n) = Tn(n-1); Qn(n) = Qn(n-1); 
                %# fastindex
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

                %aMu = aMu + dt * ( abs(ro(1)*u(1)*S(1)) + abs(ro(n)*u(n)*S(n)) ); % Mass out
                %# fastindex
                ite = ite+1;
                %# fastindex
                itemed = itemed+1; dtmed = dtmed+dt;
                %# fastindex
                t = t+dt;
                %# fastindex
                itenvo = itenv-1; itev = itev+1;
            else
                %# fastindex
                itenv = itenv+1;
            end;
            %# fastindex
            if DT == dt, DT = 1.01*DT; else DT = 0.99*DT; end

        end;
        %PM(ii,:)= [0.5*(Pn(vmis(1))+Pn(vmis(1)+1))  0.25*(Pn(vmis(2:end-1)+1) + Pn(vmis(2:end-1)-1) + 2*Pn(vmis(2:end-1)) )  0.5*(Pn(vmis(end))+Pn(vmis(end)+1))];
        PM(ii,:) = [0.25*(Pn(vmis(1:end)+1)+Pn(vmis(1:end)-1)+ 2*Pn(vmis(1:end)))];

        %PM(ii,:) = Pn(vmis);
        uM(ii,:) = un(vmis); roM(ii,:) = ron(vmis); TM(ii,:) = Tn(vmis);

        %======================================================================
        % Calculation of brake cylinder pressure starting from brake pipe
        % pressure 
        %# fastindex
        if CVactv
            dtCV = t-told;
            pCF_0 = pCF;
            pCdG = (0.25*(Pn(vmisCF(1:end)+1)+Pn(vmisCF(1:end)-1)+ 2*Pn(vmisCF(1:end))))./1e5-1;
            
            [pCF,pCFm,tBkon] = CalcCF(dLC,dtCV,dTF,isservice,nCV,pCdG,pCdG_0,pCF,pCFm,tBkon,t);
            pCdG_0 = pCdG;
            
            %[pCF,pCFm,tBkon] = CalcCFO(dLC,dtCV,dTF,isservice,nCV,pCdG,pCF,pCFm,tBkon,t,VCi);
            
            PbcM(ii,iDCVa) = pCF(VhcMa);
            PbcT(ii,:) = pCF;
            PMT(ii,:) = [0.25*(Pn(vmisCF(1:end)+1)+Pn(vmisCF(1:end)-1)+ 2*Pn(vmisCF(1:end)))];
            %==========================================
            % Calculation of auxiliary reservoir pressure during braking 
            pCFa = pCF;
            appo = find(pCF ~= 0);
            pCFa(appo) = pCFa(appo)+1;
            if isservice < 0                        
                dmSA = (((pCFa-pCFmnv0)*1e5).*Vbc/r/Tamb)/dtCV;
                dmSA(find(dmSA <0)) = 0; % to avoid the charging of AR when it's closed during braking
                dPdt = kpolSA * prSA.*(dmSA./mSA);
                prSA = prSA-dPdt*dtCV;
                Tsa = cstPol./prSA.^((1-kpolSA)/kpolSA);
                roSA = prSA./(r*Tsa);
                mSA = roSA.*SA(2,:);                
            end;
            PsaM(ii,iDCVa) = (prSA(VhcMa)-1e5)/1e5;
            pCFmnv0 = pCFa;
            %===========================================
        end;
        %======================================================================
        
        %Updating
        tempo(ii) = t+t0mnv;
%         Mu(ii) = aMu;
%         MT(ii) = sum( ro(jj).*V(jj) ); % actual mass
        vprpi(ii,:) = prpi; vPmed(ii,:) = Pmed; vaamp(ii,:) = dmf;

        % actual overall data display
        %# fastindex
        if t > ndt
            %# fastindex
            ndt = ndt+tvis;
            %# fastindex
            dtmed = dtmed/itemed;
            disp(sprintf('Actual time %4.2fs, front speed %7.3fm/s, rear speed %8.3fm/s, dt %11.4es, dt ratio %g, itenv %g, itev %g',...
                tempo(ii),u(1),u(end),dtmed,etime(clock,cltvis)/tvis,itenv,itev));
            cltvis = clock;
            %# fastindex
            itemed = 0; dtmed = 0; itenv = 0; itenvo = itenv-1; itev = 0;
        end;
        vdtmed(ii) = dtmed;

        % partial display of BP pressure
        %# fastindex
        if Dpy
            figure(120);
            %subplot(2,1,1);
            plot(tempo(1:ii),PM(1:ii,:)*1e-5-1); grid on; zoom on; set(gca,'xlim',[0 TTsim]); drawnow;
            %subplot(2,1,2);
            %plot(P);
        end;
    end;
    % Settings for the new phase
    %# fastindex
    ndt = 0;
end;

ifin = ii; % final index
% Scaling quantities
tempo = tempo(1:ifin); PM = PM(1:ifin,:); uM = uM(1:ifin,:); roM = roM(1:ifin,:); TM = TM(1:ifin,:);
%# fastindex
if CVactv 
    PbcM = PbcM(1:ifin,:);  
    PsaM = PsaM(1:ifin,:); 
    PbcT = PbcT(1:ifin,:); 
    PMT = PMT(1:ifin,:); 
    
    % It could be that tempo does not have data spaced by the sampling rate:
    % this error is corrected. IT IS ASSUMED THAT THE STARTING TIME IS ZERO
    PbcM = TPspline(PbcM,tempo,tempo(end),SR);
    PbcM( PbcM < 0 ) = 0;
    PM = TPspline(PM,tempo,tempo(end),SR);
    PM( PM < 0 ) = 0;    
    
    % It could be that tempo does not have data spaced by the sampling rate:
    % this error is corrected. IT IS ASSUMED THAT THE STARTING TIME IS ZERO
    % code to build DATASA for interfacing with TrainDy 
    PbcT = TPspline(PbcT,tempo,tempo(end),SR);
    PbcT( PbcT < 0 ) = 0;
    PMT = TPspline(PMT,tempo,tempo(end),SR);
    PMT( PMT < 0 ) = 0;    
    
    tempoT = 0:SR:tempo(end);
end;

if Dpy
    % Plot of counter-pressure and pressure in BP to control the flow through
    % MBV
    figure(11); clf;
    %plot(tempo,vPmed(1:ifin,:)-1e5); ylabel('BP pressure for DBV [Pa (gauge)]'); grid on; set(gca,'xlim',[0,TTsim]);
    plot(tempo,[vPmed(1:ifin,:)-1e5, vprpi(1:ifin,:)-1e5]); ylabel('[Pa (gauge)]'); grid on; set(gca,'xlim',[0,TTsim]);
    % Assignement legend in the figure
    DBVlegg = '';
    %# fastindex
    for ii = 1:nDBV
        appo = strvcat(['BP pressure for DBV #.',num2str(ii)]);
        DBVlegg = strvcat(DBVlegg,appo);
    end;
    for ii = 1:nDBV
        appo = strvcat(['Ext counter-pressure for DBV #.',num2str(ii)]);
        DBVlegg = strvcat(DBVlegg,appo);
    end;
    legend(DBVlegg);


    figure(12); clf;
    plot(tempo,vaamp(1:ifin,:)); ylabel('Mass flow rate through DBV [kg/s]'); grid on; set(gca,'xlim',[0,TTsim]);
end;

