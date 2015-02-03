function [mCA,prCA,tBkon] = initialManvBC(CA,CVactv,isservice,nCV,pCF,Pn,r,Tamb,tBkon)

% INITIALMANVBC This function initializes the manoeuvre indices for the 
%               Brake Cylinders. It locates the re-armed Acceleration
%               Chambers and updates their data
% 
% INPUTS        CA:        Matrix with Acceleration Chamber data
%               CVactv:    Scalar with status of Control Valves
%               isservice: Variable that indicates the type of manoeuvre
%                          for each locomotive (EB,SB,R)
%               nCV:       Number of Control Valves 
%               pCF:       Brake Cylinder pressure
%               Pn:        Vector with pressure in Brake Pipe sections
%               r:         Gas constant
%               Tamb:      Atmospheric temperature
%               tBkon:     (Absolute) initial time of Brake Cylinder 
%                          filling [s]                     
%
% OUTPUTS       mCA:       Air mass inside the Acceleration Chambers
%               prCA:      Pressure in Acceleration Chambers
%               tBkon:     Updated time index

global ASph eftPt inpt 

% Redefining parameter of Control Valve following the actual manouvre
% if CVactv    

if CVactv && any(isservice)
    %Initialization of Acceleration Chambers
    % Acceleration Chamberacs no rearmed 
    % eftPt = zeros(1,nCV);
    % ASph = zeros(1,nCV);
    % inpt = ones(1,nCV);
    % TODO: check if it is possible to remove the following two rows
    prCA  = Pn(CA(1,:));
    mCA   = (CA(2,:).*prCA) / Tamb / r; 
    % Acceleration Chambers re-armed
    appo1 = find(Pn(CA(1,:)) >= CA(4,:));
    appo2 = find(pCF(appo1) == 0);
    rrmd  = appo1(appo2);
    
    eftPt(rrmd) = 1; 
    ASph(rrmd)  = 1;
    inpt(rrmd)  = 0;  
    prCA(rrmd)  = CA(5,rrmd);
    mCA(rrmd)   = (CA(2,rrmd).*prCA(rrmd)) / Tamb / r; % Volume * (Density for ideal gas) = Air mass in Accleration Chambers [SBB]
    tBkon(rrmd) = 0;  % Braking time activation
end   

end