function [Cfq,dly,grapi,prpi,ptarget,segnof,Sut,t1,t,tBkon] = initialManv2(CA,CVactv,DBVdata,...
    dt,isservice,jm,nCV,nDBV,pCF,pfin,Pn,prpi,typeDBV)


global ASph eftPt inpt mCA prCA %used in CalcCF
global Tamb r 

%# scalar CVactv diaDsc diaRp diaRch k kk t Tbk td1

ptarget = 0; 
segnof = zeros(1,nDBV); Sut = zeros(1,nDBV); t1 = zeros(1,nDBV); grapi = zeros(1,nDBV);
Cfq = []; grapi = []; segnof = []; Sut = []; t1 = [];

% Redefining parameter of DBV following the actual manouvre
%k = 0; zeri = zeros(1,nDBV); segnof = zeri; diapiRif = zeri; td1 = zeri; t1 = zeri; dly = zeri; grapi = zeri';
%# fastindex
k = 0; %tBkon = 0; 
dly = zeros(1,nDBV); ptarget = pfin;
%# fastindex
%for kk = 1:nDBV
for kk = find(pfin)
    %%% Define delay of manouvre
    %dly(kk) = DBVdata(3*k+1,3+jm);
    if isservice(kk) > 0
        DBVdRch =  DBVdata(3*k+3,:);            
        % Recharge
        segnof(kk) = -1; 
        if typeDBV(kk) == -1
            segnof(kk) = +1;
        end;       
        %# fastindex
        Tbk = DBVdRch(3);
        
        if pfin(kk) < 5.5e5
            ptarget(kk) = pfin(kk);
        else
            ptarget(kk) = pfin(kk)+0.4e5; % target pressure defined in function of manouvre inside servrec            
        end;
        grapi(kk,1) = ((5-3.5)*1e5)/Tbk; t1(kk) = ((pfin(kk)-0.2e5)-prpi(kk))/grapi(kk,1); 
        %# fastindex
        diaRch = DBVdRch(1); Cfq(kk) = DBVdRch(2);
        Sut(kk) = diaRch^2*pi*0.25; % Cross section of the nozzle with fixed diameter           
    elseif isservice(kk) == -2
        DBVdDsc =  DBVdata(3*k+2,:);
        % Service discharge
        segnof(kk) = +1;
        if typeDBV(kk) == -1
            segnof(kk) = -1;
        end;    
        %# fastindex
        Tbk = DBVdDsc(3); 
        %# fastindex
        td1 = DBVdDsc(4);
        
        ptarget(kk) = pfin(kk); % pressione di target uguale alla finale nel servizio           
        grapi(kk,1) = (5.5e5-6e5)/td1;
        grapi(kk,2) = -1.0e5/(Tbk-td1);
        %# fastindex
        diaDsc = DBVdDsc(1); Cfq(kk) = DBVdDsc(2);  
        Sut(kk) = diaDsc^2*pi*0.25; % Cross section of the nozzle with fixed diameter  
        %prpi(kk) = 6e5;
    elseif isservice(kk) == -1
        DBVdRp =  DBVdata(3*k+1,:); 
        segnof(kk) = +1;
        if typeDBV(kk) == -1
            segnof(kk) = -1;
        end;    
        prpi(kk) = 1e5;
        %# fastindex
        diaRp = DBVdRp(1); Cfq(kk) = DBVdRp(2);          
        Sut(kk) = diaRp^2*pi*0.25; % Cross section of the nozzle with fixed diameter            
    elseif isservice(kk) == 0 % Added for dynamics
        Cfq = []; dly = []; grapi = []; ptarget = []; segnof = []; Sut = []; t1 = []; t = []; tBkon = []; 
    end;  
    k = k+1;
end; 

% Redefining parameter of Control Valve following the actual manouvre
%# fastindex
% if CVactv     
if CVactv && any(isservice) && jm == 0
    %Initialization of acceleration chambers
    % acceleration chambers no rearmed 
    eftPt = zeros(1,nCV);
    ASph = zeros(1,nCV);
    inpt = ones(1,nCV);   
    prCA = Pn(CA(1,:));
    mCA = ( CA(2,:).*prCA ) / Tamb / r; 
    % acceleration chamers rearmed
    appo1 = find(Pn(CA(1,:)) >= CA(4,:));
    appo2 = find(pCF(appo1) == 0);
    rrmd = appo1(appo2);
    eftPt(rrmd) = 1; 
    ASph(rrmd) = 1;
    inpt(rrmd) = 0;  
    prCA(rrmd) = CA(5,rrmd);
    mCA(rrmd) = ( CA(2,rrmd).*prCA(rrmd) ) / Tamb / r;   
    tBkon = zeros(1,nCV);  % Braking time activation
end;     
%# fastindex
t = dt; % Time is re-initialized