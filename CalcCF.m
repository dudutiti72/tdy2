function [pCF,pCFm,tBkon] = CalcCF(dLC,dt,dTF,isservice,nCV,pCdG,pCdG_0,pCF,pCFm,tBkon,temp)

% CALCCF  This function calculates the pressure in the Brake Cylinders at
%         tf, taking into account the previous Brake Cylinder pressure, the 
%         current pressure in Brake Pipe and the Control Valve data
%
% INPUTS  dLC:       Array with limiting curve data of Control Valves. It
%                    has as many columns as vehicles. See PneumDevices.m 
%                    for details
%         dt:        Time between the beginning and the end of the
%                    integration in comp_pressure_tpSR (tfin - told) [s]    
%         dTF:       Array with transfer function data of Control Valves.
%                    It has as many rows as vehicles
%         isservice: Variable that indicates the type of manoeuvre for each
%                    locomotive (EB,SB,R)
%         nCV:       Number of Control Valves
%         pCdG:      Relative mean pressure in Brake Pipe around the 
%                    Control Valve of each vehicle at tfin [bar]
%         pCdG_0:    Previous relative pressure, at the beginning of
%                    integration in comp_pressure_tpSR [bar]
%         pCF:       Pressure in Brake Cylinders at time told (when 
%                    comp_pressure_tpSR is called) [bar]
%         pCFm:      Used to distinguish if Brake Cylinder is filled 
%                    following the limiting curve or the Brake Pipe [bar]
%         tBkon:     (Absolute) initial time of Brake Cylinder filling [s]                  
%         temp:      Current time t which is actually equal to tfin, since 
%                    the function is called after the integration [s]
%
% OUTPUTS pCF:       Updated pressure in Brake Cylinders at time tfin.
%                    Changes only if Control Valve is active 
%         pCFm:      Updated pCFm at tfin
%         tBkon:     Updated time tBkon. Reset in case an application
%                    stroke phase begins

global eftPt inpt    % inpt: ON-OFF indexing for activated Control Vavles [SBB]

%# scalar ii

tBk = temp-tBkon; %  Time elapsed since beginning of Brake Cylinder filling

%# fastindex

for ii = 1:nCV
    
    if (pCdG(ii) < pCdG_0(ii)) && inpt(ii) == 0 && pCdG(ii) <= dLC(1,ii)    % dLC(1,:) = Pactv
        
        % 'Activation' of Control Valve and beginning of Application stroke
        inpt(ii)  = 1;
        pCF(ii)   = dLC(2,ii);                                              % dLC(2,:) = pCFAs
        tBkon(ii) = temp;
        tBk(ii)   = temp-tBkon(ii); % Updating of braking manoeuvre timimg  % [s!] this is zero.. why not just write 0?   
        
    elseif  inpt(ii) == 1
        
        % Calculate new BC pressure as long as Control Valve is 'active'
        [pCF(ii),pCFm(ii)] = pCF_CG(dLC(:,ii),dt,dTF(ii,:,:),eftPt(ii),ii, ...
           isservice,pCdG(ii),pCdG_0(ii),pCF(ii),pCFm(ii),tBk(ii));   
        
    end     
end