function [dmf] = fxNzleVcq(coeq,Pdn,Pup,segno,Sut,Tdn,Tup)

% FXNZLEVCQ This function computes the mass flow rate between Brake Pipe  
%           and pilot chambers of Driver's Brake Valves. For this purpose 
%           the flow coefficient Cq and the mass flow parameter Cm have to
%           be computed. Note that the equation for the mass flow rate
%           is bidireectional and allows the filling as well as the
%           emptying of the chambers. 
%
% INPUTS    coeq:  Flow coefficient Cq. If -1, it is not defined by user 
%                  and therefore calculated in this function
%           Pdn:   Downstream pressure of nozzle
%           Pup:   Upstream   pressure of nozzle
%           segno: Direction of flow
%           Sut:   Cross section of nozzle [m^2] 
%           Tdn:   Downstream temperature of nozzle [K]
%           Tup:   Upstream   temperature of nozzle [K]

global Cms gam prac r

% This function computes the mass flow for a nozzle with fixed diameter and variable
% flow coefficient; Pdn and Pup represents the pressure upstream and downstream of
% this nozzle; this pressures change according to the manoeuvre: Pdov is the pressure
% of the chamber between the nozzles for a breaking whereas it is the pressure in
% main Brake Pipe for a release; Pupv is the pressure in the main Brake Pipe for a
% breaking whereas it is the pressure in hte chamber between the two nozzles. The
% same behaviour is true for the temperatures.

praf = Pdn/Pup; % Pressure ratio

% If pressure in upstream of orifice is lower than pressure in downstream,
% the air flow change direction.
% if praf > 1
%     segno = -segno; praf = 1/praf;
%     Pup = Pdn; Tup = Tdn;
% end

appo        = find(praf > 1);
praf(appo)  = 1/praf;  
segno(appo) = -segno; 
Pup(appo)   = Pdn; 
Tup(appo)   = Tdn;

% Assigning Cq
% if coeq == -1
%     coeqf = 0.8414-0.1002*praf+0.8415*praf^2-3.9*praf^3+4.6001*praf^4-1.6827*praf^5; % Cq Perry
% else
%     coeqf = coeq;
% end
coeqf = coeq;
CqPy  = find(coeq == -1);
coeqf(CqPy) = 0.8414-0.1002*praf+0.8415*praf^2-3.9*praf^3+4.6001*praf^4-1.6827*praf^5; % Cq Perry
% Assigning Cm
% if praf > prac
%     % Subsonic
%     Cm = sqrt(2*gam/(r*(gam-1)))*sqrt(praf^(2/gam)-praf^((gam+1)/gam));
% else
%     % Sonic
%     Cm = Cms;
% end
effl     = find(praf > prac);
Cm       = Cms;  % Sonic
Cm(effl) = sqrt(2*gam/(r*(gam-1)))*sqrt(praf(effl).^(2/gam)-praf(effl).^((gam+1)/gam)); % Subsonic

% Calculation of mass flow rate
dmf = segno*Sut*coeqf*Cm*Pup/sqrt(Tup);