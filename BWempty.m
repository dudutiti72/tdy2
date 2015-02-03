function Pempty = BWempty (Bempty,typeSh,na,nbs,Ff,Fr,SI)
% Fr = 2; %[kN]
% Ff = 1.5; %[kN]
rtim = 0.83;
istar = na*2;
if strcmp(typeSh,'Bg')
    % Bg shoes
    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
end
Pempty = fzero(@brakeweight,3);
    function f = brakeweight(Pempty)
        Fd = (Pempty*SI*1e-2-Ff*11.87-istar*Fr)*rtim;
        Fd = Fd/nbs;
        k = a0+a1*Fd+a2*Fd^2+a3*Fd^3;
        f = Bempty*9.81-k*nbs*Fd;
    end

end