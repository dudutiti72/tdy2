function [dmf,grapi,Pmed,Pmedo,prpi,ptarget,dQdt,drodt,dudt,P] = comp_pneu(t,y0P,CA,Cfq,cv,CVactv,D,dly,dmf,dt,dx,EOT,FOT,grapi,iCV,isservice,iV,j0,j1,jj,K,lam,Lmedia,...
    n,nDBV,pfin,Pmed,Pmedo,prpi,ptarget,r,rugrel,S,SA,s,segnoCA,segnoSA,segnof,Sut,Tamb,t1,typeDBV,...
    viscosita,vmisCF)

% TODO: Try to avoid the transpose
ro = y0P(1:n)'; u = y0P(n+1:2*n)'; Q = y0P(2*n+1:3*n)';
if FOT == 0
    u(1) = 0;
end;
if EOT == 0
    u(n) = 0;
end;

% New temperature calculation
T = ( Q - 0.5*u.^2 ) / cv;

% Pressure calculation using perfect gas law
P = r * ro.*T;

[dmf,grapi,P,Pmed,Pmedo,prpi,ptarget,Q,ro,T,u,velc] = boundaryCond2(Cfq,...
    dly,dmf,dt,grapi,isservice,iV,nDBV,P,pfin,Pmed,Pmedo,prpi,ptarget,Q,ro,S,...
    segnof,Sut,t,T,t1,typeDBV,u);

% CALCULATION OF NEW DERIVATIVES USING VALUE AT STEP I-1
Re = abs(ro.*u.*D)./viscosita+1e-15;
unosuRe = Re.^-1;
laminare = find(Re < 1000); % Si usa 1000 per ragioni numeriche, vale a dire per avere continuità di punto (anche se non di tangenza)
A = -2*log10(rugrel/3.7+12.*unosuRe);
B = -2*log10(rugrel/3.7+2.51*A.*unosuRe);
C = -2*log10(rugrel/3.7+2.51*B.*unosuRe);
f = (A - ((B-A).^2) ./ (C-2*B+A) ).^-2;
f(laminare) = 64.*unosuRe(laminare);
f(Re < 1) = 0;
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

drodt = [drodt(1) drodt drodt(end)]; dudt = [ dudt(1) dudt dudt(end)]; dQdt = [dQdt(1) dQdt dQdt(end)];
end

