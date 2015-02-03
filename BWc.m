function SI = BWc(B,typeSh,na,nbs,Ff,Fr,Pbrake)
% This function computes the product of the section of the braking cylinder by
% the timonery ratio (SI). The input is the braked mass (B). The computation is
% achieved using the standard UIC 544-1
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
SI = fzero(@brakeweight,11.87*854); %La sezione è in cm^2 %11.87*854
    function f = brakeweight(SI)
        
        Fd = (Pbrake*SI*1e-2-Ff*11.87-istar*Fr)*rtim;
        Fd = Fd/nbs;
        k = a0+a1*Fd+a2*Fd^2+a3*Fd^3;
        f = B*9.81-k*nbs*Fd;
    end

end
