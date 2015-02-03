function [mCA,prCA,tBkon] = initialManvBC(CA,CVactv,isservice,nCV,pCF,Pn,r,Tamb,tBkon)

global ASph eftPt inpt

% Redefining parameter of Control Valve following the actual manouvre
% if CVactv     
if CVactv && any(isservice)
    %Initialization of acceleration chambers
    % acceleration chambers no rearmed 
    % eftPt = zeros(1,nCV);
    % ASph = zeros(1,nCV);
    % inpt = ones(1,nCV);
    % TODO: check if it is possible to remove the following two rows
    prCA = Pn(CA(1,:));
    mCA = ( CA(2,:).*prCA ) / Tamb / r; 
    % acceleration chamers rearmed
    appo1 = find(Pn(CA(1,:)) >= CA(4,:));
    appo2 = find(pCF(appo1) == 0);
    rrmd = appo1(appo2);
    eftPt(rrmd) = 1; 
    ASph(rrmd) = 1;
    inpt(rrmd) = 0;  
    prCA(rrmd) = CA(5,rrmd);
    mCA(rrmd) = ( CA(2,rrmd).*prCA(rrmd) ) / Tamb / r;   
    tBkon(rrmd) = 0;  % Braking time activation
end;     

end