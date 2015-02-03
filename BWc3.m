function [I,na,nbs,S,wrn] = BWc3(B,typeSh,na,nbs,Ff,Fr,Pbrake)
% This function computes the rigging ratio. The input is the braked weight
% (B). The computation is achieved using the standard UIC 544-1
rtim = 0.83; % In agreement with UIC 544-1
% Section is in cm^2
if na == 2
    S = 707;
elseif na == 4
    S = 1295;
else
    disp('Check the braked weight');
    S = 0.5*(707+1295);
end;

wrnup = 'The braked weight has been limited to its maximum standardized value';
wrndown = 'The braked weight has been limited to its minimum standardized value';
wrn = '';
r_nbs_na = nbs/na;
if strcmp(typeSh,'Bg')
    % Bg shoes
    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
    if B/nbs > 3.661
        B = nbs*3.661;
        wrn = wrnup;
        %warning(wrnup); 
    elseif B/nbs < 0.966
        B = nbs*0.966;
        wrn = wrndown;
        %warning(wrndown); 
    end
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
    if B/nbs > 4.508 % TODO: THIS (4.608) VALUE HAS BEEN CHANGED WITH RESPECT TO THE LEAFLET ON 30TH MAY 2011
        B = nbs*4.508;
        wrn = wrnup;
        %warning(wrnup); 
    elseif B/nbs < 0.968
        B = nbs*0.968;
        wrn = wrndown;
        %warning(wrndown); 
    end
end
% TODO: Following assignments are useless.
nbs = na*r_nbs_na;
istar = na*2;

I = fzero(@brakeweight,11.87); % Starting value 11.87
if isnan(I) || ~isreal(I), error('Problem in FZERO'); end
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
