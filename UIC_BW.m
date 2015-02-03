function [B,k,Ft,SFd] = UIC_BW(Ff,Fr,I,na,nbs,Pbrake,rtim,S,typeSh)

% UIC_BW    This function computes the braked weight and the normal brake 
%           force from the physical parameters of a block brake system.
%           Only usable for diagnostic purposes since the output does not
%           need to be used in the calculations
%
% INPUTS    Ff:     Brake rigging return force (usually 1.5 [kN])
%           Fr:     Counteracting force of brake rigging regulator (2[kN])
%           I:      rigging ratio 
%           na:     Number of axes
%           nbs:    Number of bring shoes (4*na)
%           Pbrake: Braking target pressure
%           rtim:   Rigging efficiency (usually 0.83)
%           S:      Brake Cylinder cross section 
%           typeSh: Type of shoes 
%
% OUTPUTS   B:      Braked weight [t]
%           k:      UIC assessment factor 
%           Ft:     Force applied by the Brake Cylinder
%           SFd:    Total normal force from shoes to wheels


if strcmp(typeSh,'Bg')          % Bg shoes

    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
    
elseif strcmp(typeSh,'Bgu')     % Bgu shoes

    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
    
end
                                            % In Traindy user guide
istar = 2*na;                               % I:        i_G
Ft    = Pbrake*S - Ff;                      % Pbrake:   P_BC 
SFd   = (Ft*I - istar*Fr)*rtim;             % S:        S_BC 
Fd    = SFd/nbs;                            %                   [SBB]
k     = a0 + a1*Fd + a2*Fd^2 + a3*Fd^3;
B     = (k*nbs*Fd)/9.81;

end
