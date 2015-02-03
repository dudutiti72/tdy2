function [CA,dQdto,d2Qdt2o,drodto,d2rodt2o,DT,dt,dudto,d2udt2o,D,EOT,FOT,iCV,iV,j0,j1,jj,K,...
    L,Lmedia,n,nCV,nDBV,nsim,pmis,rugrel,SA,segnoCA,segnoSA,SR,typeDBV,vmis,vmisCF,vUnC] = comp_data(CA,...
    CVactv,D1,DBVdata,Dcoll,dx,epsilon,gmis,gmisCF,kcoll,Lcoll,Lvago,nveicoli,SA,SR,train,Tsim)

%# scalar c cdt CVactv dt EOT FOT ii k Ltubo n nCV nSR nsim nveicoli pm R SR Tsim

global Cms cv gam kpol prac r Tamb tFpt viscosita   

%GENERIC CONSTANT
%# fastindex
R = 8.314472;% [J/(mole*K)]
%# fastindex
pm = 28.92;% [g/mole]
%# fastindex
r = R*1000/pm; % Costante dell'aria [J/(Kg*K)]
gam = 1.4; prac = (2/(gam+1))^(gam/(gam-1)); % Rapporto critico delle pressioni
cv = 2.5*r; %[J/(kg*K)] % (gas perfetti)
Cms = sqrt(gam/r*(2/(gam+1))^((gam+1)/(gam-1))); % Sonic Cm

% Determinazione della velocità del suono alla temperatura iniziale di svuotamento
Vs = (1.4*r*Tamb)^(1/2); % Velocità del suono nell'aria [m/s]

% Si può impostare una variabilità dalla temperatura della viscosità: si utilizza la
% relazione di Sutherland
%# fastindex
T0 = 288.15; viscosita0 = 1.78e-5; cS = 110;
%# fastindex
viscosita = viscosita0 * (Tamb/T0)^1.5 * (T0+cS)/(Tamb+cS);

kpol = 1; % politropic index


%
%    ----------        ----------
%    |        |        |        |
% ---         ----------        ----------
%
% ---         ----------        ----------
%    |        |        |        |
%    ----------        ----------
%

L = []; D = []; K = []; %epsilon = [];
vUnC = [];
for ii = 1:nveicoli
    % Si aggiunge il vagone corrente
    ndx = ceil(Lvago(ii)/dx);
    dxa = Lvago(ii)/ndx;
    L = [L dxa*ones(1,ndx)];
    if ii < nveicoli
        D = [D D1(ii)*ones(1,ndx-1) Dcoll];
        %epsilon = [epsilon eps1*ones(1,ndx-1) epscoll];
        if strcmp(train(ii).type,'loco') && not(isempty(train(ii).K))
            K = [K zeros(1,ndx-1) train(ii).K];
        else
            K = [K zeros(1,ndx-1) kcoll];
        end
        
        % Si aggiunge il collegamento
        ndx = ceil(Lcoll/dx);
        dxa = Lcoll/ndx;
        L = [L dxa*ones(1,ndx)];
        D = [D Dcoll*ones(1,ndx)];
        %epsilon = [epsilon epscoll*ones(1,ndx)];
        if strcmp(train(ii+1).type,'loco') && not(isempty(train(ii+1).K))
            appo = [zeros(1,ndx-1) train(ii+1).K];
        else
            appo = [zeros(1,ndx-1) kcoll];
        end
        K = [K appo];        
        if train(ii).UnC == 1
            vUnC = [vUnC size(D,2)-ndx:size(D,2)];
        end
    else
        D = [D D1(ii)*ones(1,ndx)];
        %epsilon = [epsilon eps1*ones(1,ndx)];
        K = [K zeros(1,ndx)];
    end;	
end;
%# fastindex
Ltubo = sum(L);
cumL = cumsum(L);

% Determinazione delle sezioni da cui leggere l'informazione
vmis = []; 
%# fastindex
x = 0; 
ind = 1; 
%# fastindex
c = 0; 
while x <= Ltubo & c < length(gmis)	
	c = c+1;
	x = gmis(c);
	% Si deve trovare la sezione che è vicina a x
	while cumL(ind) < x
		ind = ind+1;
	end;
	if ind > 1 & abs(cumL(ind)-x) > abs(cumL(ind-1)-x)
		ind = ind-1;
	end;
	vmis = [vmis ind];
end;

% vmisCF is a vector that stores the sections where the brake cylinders are located
vmisCF = []; x = 0; ind = 1; c = 0; 
while x <= Ltubo & c < length(gmisCF)	
	c = c+1;
	x = gmisCF(c);
	% Si deve trovare la sezione che è vicina a x
	while cumL(ind) < x
		ind = ind+1;
	end;
	if ind > 1 & abs(cumL(ind)-x) > abs(cumL(ind-1)-x)
		ind = ind-1;
	end;
	vmisCF = [vmisCF ind];
end;

% Defition of section number where is located the MBV
% Defining index for DBV control
nDBV = size(DBVdata,1)/3;% Defining index for DBV control
vmisMBV = []; x = 0; ind = 1; c = 0; k = 0;
while x <= Ltubo & c < nDBV	
	c = c+1;
    x = DBVdata(3*k+1,3);
	% Si deve trovare la sezione che è vicina a x
	while cumL(ind) < x
		ind = ind+1;
	end;
	if ind > 1 & abs(cumL(ind)-x) > abs(cumL(ind-1)-x)
		ind = ind-1;
	end;
	vmisMBV(c) = [ind];
    k = k+1;
end;

% Sezioni fantasma
L = [L(1) L L(end)]; 
D = [D(1) D D(end)]; K = [K(1) K K(end)]; vmis = vmis+1; vmisCF = vmisCF+1; vmisMBV = vmisMBV+1; %epsilon = [epsilon(1) epsilon epsilon(end)];

A = cumsum(L);
pmis = A(vmis)-L(1);

% Data integration method
%# fastindex
cdt = 3e-4;
%# fastindex
dt = cdt*min(L)/Vs;
nSR = round(SR/dt);
%# fastindex
dt = SR/nSR; 
% ADDED to speed up the numerical integration
dt = 1e-6;
DT = dt;
nsim = round(Tsim/SR)+1;

% Number of computation sections for the pneumatics
n = size(D,2);
j0 = 1:n-2; jj = 2:n-1; j1 = 3:n; % Vectors to integrate the pneumatic parameters of the brake pipe

% Control correct discretization
Lmedia = 2*L(jj);
Lmedia2 = L(j0) + L(jj); 
if max(abs((Lmedia-Lmedia2)./Lmedia2)) > 1e-7
	error('Il codice lavora solo con griglia a spaziatura costante');
end;

% Matrix CA is re-defined
%# fastindex
nCV = size(CA,2); %number of control valve = CA = SA
%# fastindex
if CVactv
    % Acceleration chambers and Auxiliary reservoirs 
    % Si passa dalla posizione del foro laterale (espressa in m), alla posizione in
    % termini di indici
%     for ii = 1:nCV
%         appo = find(cumsum(L) <= CA(1,ii));
%         CA(1,ii) = appo(end); 
%         SA(1,ii) = appo(end); 
%         vmisCF
%     end;
    CA(1,:) = vmisCF; 
    SA(1,:) = vmisCF; 
    
    % Defining section of acceleration chamber instead starting from diameter
    CA(3,:) = 0.25*pi*CA(3,:).^2;
    
    segnoCA = ones(1,nCV);       
    segnoSA = ones(1,nCV);   
    iCV = CA(1,:);  
    
    % Control valve parameters
    % Inizializzazione termini calcolo pressioni CF
    tFpt = zeros(1,nCV);    
else
     iCV = [];  segnoSA = []; segnoCA = []; %nCV = size(CA,2); ...
     tFpt = [];
end;

%%% Defining index for DBV control
k = 0;
% iV is the array that manages the sections of the Driver's Brake Valve. The first
% two rows contain information on the sections and the third row is different by zero
% only if the valve is a FOT or and EOT (this is not supported by the GUI).
iV = zeros(3,nDBV);
EOT = 0; FOT = 0; % Parameters to define if a FOT or a EOT is in the train configuration
%# fastindex
for kk = 1:nDBV
    posDBV = DBVdata(3*k+1,3);
    if posDBV > 1 % lateral discharging
        % This vector stores the positions of the lateral inputs according to an index
        % notation
        iV(1:2,kk) = vmisMBV(kk)+[0 1]';
        typeDBV(kk) = 1; % lateral DBV
    elseif posDBV == 0
        iV(:,kk) = [2 3 1]'; 
        typeDBV(kk) = 0; % FOT - front of train
        %# fastindex
        FOT = 1;
    else
        iV(:,kk) = [n-1 n-2 n]'; 
        typeDBV(kk) = -1; % EOT - end of train
        %# fastindex
        EOT = 1;
    end;
    %# fastindex
    k = k+1;
end;

rugrel = epsilon./D; 

% Initializations of derivatives in integration method
zerijj = zeros(size(jj));
drodto = zerijj; dudto = zerijj; dQdto = zerijj;
d2rodt2o = zerijj; d2udt2o = zerijj; d2Qdt2o = zerijj;