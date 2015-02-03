function Frestot = fresistenza(CCtrac,fitrac,N,pesoz,smt,y)

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
Frestot = rescurve+resatt+pesox;
Frestot(1) = Frestot(1) + Faero;

end