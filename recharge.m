function [grapi,prpi,ptarget] = recharge(dt,gracg,grapi,pfin,Pmed,...
    prpi,ptarget,t,t1)

global cRch1 cRch2 

%This function computes the raising of the (pilot) valve during a service recharge
%Computation of pressure in pilot chamber
if t >= t1 & ptarget == pfin+0.4e5 & cRch1 == 0 & Pmed > pfin-0.5e5
    ptarget = pfin;
    cRch1 = 1;
else
    cRch1 = 1;
end;
grapi(prpi >= ptarget) = 0;

if ptarget == pfin+0.4e5 & Pmed >= pfin-0.2e5 & cRch2 == 0
    grapi = -2*gracg;
    cRch2 == 1;
end;
prpi = prpi+grapi*dt;
if grapi < 0 & prpi < pfin  
    prpi = pfin;
    grapi = 0;
end;

% if ptarget == pfin+0.4e5 & t >= t1 && cRch1 == 0
%     if Pmed > pfin-0.5e5
%         ptarget = pfin; 
%     end;
%      cRch1 = 1;
% end;
% if prpi >= ptarget
%     grapi = 0;
% end;
% 
% if ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 && cRch2 == 0
%     grapi = -2*gracg;
% end;
% prpi = prpi+grapi*dt;
% if grapi < 0 && prpi < pfin  
%     prpi = pfin;
%     grapi = 0;
%     cRch2 = 1;
% end;













% if ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 & prpi-Pmed <= 0.12e5 && cRch2 == 0 
%     grapi = -1*(0.15e5/75);
% elseif ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 && cRch2 == 0
%     grapi = -2*gracg;
% end;
% prpi = prpi+grapi*dt;
% if grapi < 0 && prpi < pfin  
%     prpi = pfin;
%     grapi = 0;
%     cRch2 = 1;
% end;