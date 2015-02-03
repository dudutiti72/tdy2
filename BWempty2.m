function Pempty = BWempty2(Bempty,typeSh,na,nbs,Ff,Fr,S,I)

% BWEMPTY2 This function computes the Brake Cylinder pressure that must be
%          applied on a wagon with an Empty-Load system in empty state, in
%          order to obtain the given empty braked weight.
% 
% INPUTS   Bempty: Braked weight when empty [t]
%          typeSh: Type of shoes
%          na:     Number of axes
%          nbs:    Number of bring shoes (4*na)
%          Ff:     Brake rigging return force (usually 1.5 [kN])
%          Fr:     Counteracting force of brake rigging regulator (2 [kN]) 
%          S:      Brake Cylinder section [mm^2]
%          I:      Rigging ratio
%
% OUTPUT   Pempty: Empty pressure [bar]

rtim  = 0.83;
istar = na*2;

if strcmp(typeSh,'Bg')
    % Bg shoes
    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
end

Pempty = fzero(@brakeweight,2);

    function f = brakeweight(Pempty)
        Fd = (Pempty*S*I*1e-2-Ff*I-istar*Fr)*rtim;
        Fd = Fd/nbs;
        k  = a0+a1*Fd+a2*Fd^2+a3*Fd^3;
        f  = Bempty*9.81-k*nbs*Fd;
    end

end