function [pCF,pCFm,tBkon] = CalcCF(dLC,dt,dTF,isservice,nCV,pCdG,pCdG_0,pCF,pCFm,tBkon,temp)

global eftPt inpt

%# scalar ii

tBk = temp-tBkon; % Timimg of braking/releasing manoeuvre 
%# fastindex
for ii = 1:nCV
    if (pCdG(ii) < pCdG_0(ii)) && inpt(ii) == 0 && pCdG(ii) <= dLC(1,ii)
        inpt(ii) = 1;
        pCF(ii) = dLC(2,ii);
        tBkon(ii) = temp;
        tBk(ii) = temp-tBkon(ii); % Updating of braking manoeuvre timimg       
    elseif  inpt(ii) == 1
        [pCF(ii),pCFm(ii)] = pCF_CG(dLC(:,ii),dt,dTF(ii,:,:),eftPt(ii),ii,isservice,pCdG(ii),pCdG_0(ii),pCF(ii),pCFm(ii),tBk(ii));               
    end;        
end; 