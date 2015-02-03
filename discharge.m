function prpi = discharge(dt,grapi,prpi,ptarget)

% This function computes the lift of the exaust valve during a service
% discharge
% Calculation of pressure in pilot chamber
if prpi > 5.5e5
    prpi = prpi+grapi(1)*dt;
elseif prpi > ptarget
    prpi = prpi+grapi(2)*dt;
end;