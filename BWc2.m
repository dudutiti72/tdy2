function [Bcomp,k,I,S,SFd] = BWc2(B,typeSh,na,nbs,Ff,Fr,Pbrake)
% This function computes the rigging ratio. The input is the braked weight
% (B). The computation is achieved using the standard UIC 544-1
rtim = 0.83; % In agreement with UIC 544-1
istar = na*2;
if strcmp(typeSh,'Bg')
    % Bg shoes
    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
end
% Section is in cm^2
if na == 2
    S = 707;
elseif na == 4
    S = 1295;
else
    disp('Check the brake system parameter computation from the braked weight');
%     S = 0.5*(707+1295);
    S = (1295-707)*0.5*na;
end;
I = fzero(@brakeweight,11.87); % Starting value 11.87
if isnan(I)
    error_line = 'The system characteristics are not computed from the BW';
    error(error_line);
end
SFd = (Pbrake*S*I*1e-2-Ff*I-istar*Fr)*rtim;
Fd = SFd/nbs;
k = a0+a1*Fd+a2*Fd^2+a3*Fd^3;
Bcomp = k*SFd/9.81;

    function f = brakeweight(I)
        
        Fd = (Pbrake*S*I*1e-2-Ff*I-istar*Fr)*rtim;
        Fd = Fd/nbs;
        k = a0+a1*Fd+a2*Fd^2+a3*Fd^3;
        f = B*9.81-k*nbs*Fd;
    end

end
