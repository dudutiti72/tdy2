function [grapi,prpi,ptarget] = recharge(dt,gracg,grapi,pfin,Pmed,...
    prpi,ptarget,t,t1)

% RECHARGE This function computes the raising of the (pilot) valve during a 
%          release manoeuvre [?]
% 
% INPUTS   dt:      Current time step [s]
%          gracg:   Pressure gradient in Brake Pipe around Driver's Brake 
%                   Valves (instanteous) [Pa/s]
%          grapi:   Pressure gradient in pilot chambers [Pa/s]
%          pfin:    Final absolute pressure of manoeuvre [Pa] 
%          Pmed:    Mean pressure of Brake Pipe around Driver's Brake 
%                   Valves [Pa]
%          prpi:    Pressure in pilot chambers [Pa]
%          ptarget: [?]
%          t:       Current time [s] 
%          t1:      Time needed to reach the target pressure in Brake Pipe 
%                   during releasing [s]

global cRch1 cRch2 

% Computation of pressure in pilot chamber

if t >= t1 && ptarget == pfin+0.4e5 && cRch1 == 0 && Pmed > pfin-0.5e5         
    ptarget = pfin;
    cRch1   = 1;
else
    cRch1   = 1;
end
grapi(prpi >= ptarget) = 0;

if ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 && cRch2 == 0
    grapi   = -2*gracg;
    cRch2   = 1;                                                           
end
prpi = prpi+grapi*dt;
if grapi < 0 && prpi < pfin  
    prpi    = pfin;
    grapi   = 0;
end

% if ptarget == pfin+0.4e5 & t >= t1 && cRch1 == 0
%     if Pmed > pfin-0.5e5
%         ptarget = pfin; 
%     end
%      cRch1 = 1;
% end
% if prpi >= ptarget
%     grapi = 0;
% end
% 
% if ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 && cRch2 == 0
%     grapi = -2*gracg;
% end
% prpi = prpi+grapi*dt;
% if grapi < 0 && prpi < pfin  
%     prpi = pfin;
%     grapi = 0;
%     cRch2 = 1;
% end




% if ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 & prpi-Pmed <= 0.12e5 && cRch2 == 0 
%     grapi = -1*(0.15e5/75);
% elseif ptarget == pfin+0.4e5 && Pmed >= pfin-0.2e5 && cRch2 == 0
%     grapi = -2*gracg;
% end
% prpi = prpi+grapi*dt;
% if grapi < 0 && prpi < pfin  
%     prpi = pfin;
%     grapi = 0;
%     cRch2 = 1;
% end