function prpi = discharge(dt,grapi,prpi,ptarget)

% DISCHARGE This function computes the lift of the exhaust valve during a 
%           service discharge
%
% INPUTS    dt:      Current time step [s]   
%           grapi:   Pressure gradient in pilot chambers [Pa/s]
%           prpi:    Absolute pressure in pilot chambers [Pa]
%           ptarget: Target pressure to control discharge [Pa]
%
% OUTPUT    prpi:    Updated pressure in pilot chambers [Pa]

% Calculation of pressure in pilot chamber
if prpi > 5.5e5
    prpi = prpi+grapi(1)*dt;
elseif prpi > ptarget
    prpi = prpi+grapi(2)*dt;
end