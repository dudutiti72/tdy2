function Frestot = fresistenza(CCtrac,fitrac,N,pesoz,smt,y)

% FRESISTENZA This function calculate the force applied on the train due to 
%             slope, curve and aerodynamic resistance
%
% INPUTS      CCtrac:  Vector with curvature for each vehicle
%             fitrac:  Vector with slopes for each vehicle
%             N:       Number of vehicles
%             pesoz:   Weight of each vehicle (m*g)
%             smt:     Smoothing function
%             y:       First half of this vector includes the position of 
%                      each vehicle and second half the velocities
%                       
% OUTPUT      Frestot: Total force due to resistance

% Effect of the trasck slope
pesox = pesoz.*sin(fitrac);

% Curve running resistence using Rockl formulation rc=650/(R-55) [kg/tonn]. 
rescurve = (smt(1)*y(N+1)*0.65)*( (CCtrac./(1-55*CCtrac)).*pesoz );

% Running resistence of a straight track
resatt = smt.*(1.1+0.00047*(y(N+1:2*N).^2)).*(pesoz*1e-3);
    
% Aerodynamic resistence
% TODO Put in input
a = 1384.39; b = 8.24; c = 0.28;
Faero = smt(1)*a + abs(smt(1))*b*y(N+1) + c*y(N+1)^2;

% Running resistences
Frestot    = rescurve+resatt+pesox;
Frestot(1) = Frestot(1) + Faero;

end