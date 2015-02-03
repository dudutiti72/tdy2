function [Cfq,grapi,prpi,ptarget,segnof,Sut,t1] = initialManvBP(CA,CVactv,DBVdata,...
    dt,isservice,nCV,pCF,pfin,Pn,prpi,typeDBV)


global Tamb r 


Cfq = []; grapi = []; segnof = []; Sut = []; t1 = [];

k = 0;
ptarget = pfin;
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


end