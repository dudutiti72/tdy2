function [CA,dQdto,d2Qdt2o,drodto,d2rodt2o,DT,dt,dudto,d2udt2o,D,EOT,FOT,iCV,iV,j0,j1,jj,K,...
    L,Lmedia,n,nCV,nDBV,nsim,pmis,rugrel,SA,segnoCA,segnoSA,SR,typeDBV,vmis,vmisCF,vUnC] = comp_data(CA,...
    CVactv,D1,DBVdata,Dcoll,dx,epsilon,gmis,gmisCF,kcoll,Lcoll,Lvago,nveicoli,SA,SR,train,Tsim)

% COMP_DATA This function does the necessary data manipulation and
%           initializations for the pneumatic integration. The data 
%           manipulation mostly takes place to keep the compatibility with 
%           TrainPneu
%
% INPUTS    CA:       Array with Acceleration Chamber data. It has as many
%                     columns as Accelerator Chambers in defined
%                     configuration and 6 rows of data. See PneumDevices.m 
%                     for details
%           CVactv:   Here CVactv is just a scalar which is equal to one if
%                     there is at least one active Control Valve in the
%                     train and zero otherwise. CVactv in PneumDevices was
%                     a vector containing the indexes of vehicles with
%                     active Control Valves and it was subsuquently
%                     substituted by the described scalar in the same
%                     function. The reason is that currently it is supposed
%                     that all Control Valves in the train are active       [n]
%           D1:       Vector with internal Brake Pipe diameter of each
%                     vehicle [m]
%           DBVdata:  Array with Driver's Brake Valve data
%           Dcoll:    Internal diameter of hose couplings
%           dx:       Discretization of Brake Pipe [m]
%           epsilon:  Roughness of Brake Pipe 
%           gmis:     Vector with middle point of each vehicle along the
%                     Brake Pipe [m].
%           gmisCF:   Vector with locations of Brake Cylinders along the
%                     Brake Pipe. To be converted into vmisCF indicating
%                     section numbers instead [m]
%           kcoll:    Concetrated pressure loss factor of hose couplings
%           Lcoll:    Discretized length of hose couplings [m]
%           Lvago:    Vector with length of Brake Pipe for each vehicle [m]
%                     (Built in PneumDevices, discretized values)
%           nveicoli: Number of vehicles
%           SA:       Matrix containing Auxiliary Reservoir data
%           SR:       Sampling rate
%           train:    Struct array with info about every vehicle
%           Tsim:     Simulation time
%
% OUTPUTS   CA:       Updated CA matrix
%           dQdto:    Initialization of specific energy derivative to zero 
%                     to be used in the integration process later on
%           d2Qdt2o:  Same for the second derivative of specific energy
%           drodto:   Same for the first  derivative of density
%           d2rodt2o: Same for the second derivative of density
%           dudto:    Same for the first  derivative of axial velocity
%           d2udt2o:  Same for the second derivative of axial velocity
%           DT
%           dt
%           D:        Vector with internal Brake Pipe diameter of each 
%                     pipe section [m]
%           EOT:      For End of Train devices (DBV)                        [n]
%           FOT:      For Front of Train devices            
%           iCV:      Section with section numbers where active control 
%                     valves are located    
%           iV:       Matrix information about Driver's Brake Valves.
%                     Number of columns is equal to number of DBV's in the
%                     train. First two rows include the section numbers of
%                     the valve. Third row is zero for lateral devices and
%                     non zero for EOT-FOT devices, which are not supported
%                     by the GUI
%           j0:       j0, jj and j1 are vectors with indexing of the brake 
%                     pipe sections to implement the numerical integration 
%                     later on. If n is the total number of sections,
%                     j0 = [1:n-2]
%           j1:       j1 = [3:n]
%           jj:       jj = [2:n-1]
%           K:        Vector with pressure loss factor of each pipe
%                     section. Basically zero along vehicles and non zero 
%                     on the hose couplings
%           L:        Vector with exact lenght of each pipe section [m].
%                     Calculated in a way that the starting and ending 
%                     point of every vehicle coincides with the beggining
%                     and the end of a section respectively. However, since
%                     there has already been a rounding of the data in
%                     PneumDevices, L here is always a vector with all
%                     elements equal to dx. In previous versions the component of L
%                     were not all equal to dx.
%           Lmedia:   Two times L, used to calculate the derivative
%                     approximations in comp_pressure_tpSR
%           n:        Number of Brake Pipe sections
%           nCV:      Number of Control Valves
%           nDBV:     Number of Driver's Brake Valves
%           nsim
%           pmis
%           rugrel:   Relative roughness of Brake Pipe (ratio of roughness
%                     over diameter)
%           SA:       Updated SA matrix (1st row modified)
%           segnoCA:  Vector with signs to indicate direction of air flow 
%                     between Brake Pipe and Acceleration Chambers.
%                     Initialized to ones in this function
%           segnoSA:  The same for the air flow between Brake Pipe and
%                     Auxiliary Reservoirs
%           SR:       Sampling rate, actually unchanged
%           typeDBV:  Type of Driver's Brake Valves. 
%                      1 for lateral devices
%                      0 for FOT [n]
%                     -1 for EOT [n]
%           vmis:     Vector with section numbers where middle of each
%                     vehicle is
%           vmisCF:   Vector with section numbers where Brake Cylinders are
%                     located
%           vUnC:     Vector containing information about uncoupled
%                     vehicles [?]

% Notes: Lcoll and Lvago have already been discretized in Pneumdevices and
%        therefore at this point L will always be a vector with ones. The
%        manipulations with ndx and dxa that follow were probably important
%        for a previous version and perharps they can be removed


%# scalar c cdt CVactv dt EOT FOT ii k Ltubo n nCV nSR nsim nveicoli pm R SR Tsim

global Cms cv gam kpol prac r Tamb tFpt viscosita   

% GENERIC CONSTANTS
%# fastindex
R    = 8.314472;                         % [J/(mole*K)]
%# fastindex
pm   = 28.92;                            % [g/mole]
%# fastindex
r    = R*1000/pm;                        % Costante dell'aria [J/(Kg*K)]
gam  = 1.4; 
prac = (2/(gam+1))^(gam/(gam-1));        % Rapporto critico delle pressioni
                                         % If Pd/Pu > prac, Subsonic regime [SBB]
                                         
cv   = 2.5*r;                            %[J/(kg*K)] % (gas perfetti)
Cms  = sqrt(gam/r*(2/(gam+1))^((gam+1)/(gam-1))); % Sonic Cm

% Determinazione della velocità del suono alla temperatura iniziale di svuotamento
Vs = (1.4*r*Tamb)^(1/2); % Velocità del suono nell'aria [m/s]

% Sutherland's formula is used to derive the dynamic viscosity of the gas 
% as a function of temperature

%# fastindex
T0 = 288.15; viscosita0 = 1.78e-5; cS = 110;
%# fastindex
viscosita = viscosita0 * (Tamb/T0)^1.5 * (T0+cS)/(Tamb+cS);

kpol = 1; % polytropic index

%
%    ----------        ----------
%    |        |        |        |
% ---         ----------        ----------
%
% ---         ----------        ----------
%    |        |        |        |
%    ----------        ----------
%

L = []; D = []; K = []; vUnC = []; %epsilon = [];

for ii = 1:nveicoli
    % Si aggiunge il vagone corrente
    
    % [s!] Maybe make the computations below simpler since Lvago has already been rounded in previous function. Probably these lines deal with older versions of Traindy
    ndx = ceil(Lvago(ii)/dx);
    dxa = Lvago(ii)/ndx;
    L   = [L dxa*ones(1,ndx)];
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
        L   = [L dxa*ones(1,ndx)];
        D   = [D Dcoll*ones(1,ndx)];
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
    end	
end
%# fastindex
Ltubo = sum(L); 
cumL  = cumsum(L);

% Determinazione delle sezioni da cui leggere l'informazione
vmis = []; 
%# fastindex
x    = 0; 
ind  = 1; 
%# fastindex
c    = 0; 

while x <= Ltubo && c < length(gmis)                                         
	c = c+1;
	x = gmis(c);
	% Si deve trovare la sezione che è vicina a x
	while cumL(ind) < x
		ind = ind+1;
	end
	if ind > 1 && abs(cumL(ind)-x) > abs(cumL(ind-1)-x)
		ind = ind-1;
	end
	vmis = [vmis ind];
end

% vmisCF is a vector that stores the sections where the Brake Cylinders are located
vmisCF = []; x = 0; ind = 1; c = 0; 
while x <= Ltubo && c < length(gmisCF)	
	c = c+1;
	x = gmisCF(c);
	% Si deve trovare la sezione che è vicina a x
	while cumL(ind) < x
		ind = ind+1;
	end
	if ind > 1 && abs(cumL(ind)-x) > abs(cumL(ind-1)-x)
		ind = ind-1;
	end
	vmisCF  = [vmisCF ind];
end

% Defition of section number where is located the MBV
% Defining index for DBV control
nDBV    = size(DBVdata,1)/3; % Defining index for DBV control
vmisMBV = []; x = 0; ind = 1; c = 0; k = 0;
while x <= Ltubo && c < nDBV	
	c = c+1;
    x = DBVdata(3*k+1,3);
	% Si deve trovare la sezione che è vicina a x
	while cumL(ind) < x
		ind = ind+1;
	end
	if ind > 1 && abs(cumL(ind)-x) > abs(cumL(ind-1)-x)
		ind = ind-1;
	end
	vmisMBV(c) = [ind];
    k = k+1;
end

% Sezioni fantasma
L = [L(1) L L(end)]; 
D = [D(1) D D(end)]; 
K = [K(1) K K(end)]; 
vmis    = vmis+1; 
vmisCF  = vmisCF+1; 
vmisMBV = vmisMBV+1; % epsilon = [epsilon(1) epsilon epsilon(end)];

A    = cumsum(L);
pmis = A(vmis)-L(1);

% Data integration method
%# fastindex
cdt  = 3e-4;
%# fastindex
dt   = cdt*min(L)/Vs;
nSR  = round(SR/dt);
%# fastindex
dt   = SR/nSR; 
% ADDED to speed up the numerical integration
dt   = 1e-6;
DT   = dt;
nsim = round(Tsim/SR)+1;

% Number of computation sections for the pneumatics
n  = size(D,2);
j0 = 1:n-2; jj = 2:n-1; j1 = 3:n; % Vectors to integrate the pneumatic parameters of the Brake Pipe

% Control correct discretization
Lmedia  = 2*L(jj);
Lmedia2 = L(j0) + L(jj); 
if max(abs((Lmedia-Lmedia2)./Lmedia2)) > 1e-7
	error('Il codice lavora solo con griglia a spaziatura costante');
end

% Matrix CA is re-defined
%# fastindex
nCV = size(CA,2); %number of Control Valve = CA = SA
%# fastindex
if CVactv
    % Acceleration Chambers and Auxiliary Reservoirs 
    % Si passa dalla posizione del foro laterale (espressa in m), alla posizione in
    % termini di indici
%     for ii = 1:nCV
%         appo = find(cumsum(L) <= CA(1,ii));
%         CA(1,ii) = appo(end); 
%         SA(1,ii) = appo(end); 
%         vmisCF
%     end
    CA(1,:) = vmisCF; 
    SA(1,:) = vmisCF; 
    
    % Storing section of Acceleration Chamber instead of diameter
    CA(3,:) = 0.25*pi*CA(3,:).^2;
    
    segnoCA = ones(1,nCV);       
    segnoSA = ones(1,nCV);   
    iCV     = CA(1,:);  
    
    % Control Valve parameters
    % Inizializzazione termini calcolo pressioni CF
    tFpt = zeros(1,nCV);    
else
     iCV = [];  segnoSA = []; segnoCA = []; tFpt = []; %nCV = size(CA,2); ...
end

%%% Defining index for DBV control
k = 0;
% iV is the array that manages the sections of the Driver's Brake Valve. The first
% two rows contain information on the sections and the third row is different by zero
% only if the valve is a FOT or and EOT (this is not supported by the GUI)  [n]
iV  = zeros(3,nDBV);
EOT = 0; FOT = 0; % Parameters to define if a FOT or a EOT is in the train configuration
%# fastindex
for kk = 1:nDBV
    posDBV = DBVdata(3*k+1,3);
    if posDBV > 1 % lateral discharging
        % This vector stores the positions of the lateral inputs according to an index
        % notation
        iV(1:2,kk)  = vmisMBV(kk)+[0 1]';
        typeDBV(kk) = 1; % lateral DBV
    elseif posDBV == 0
        iV(:,kk)    = [2 3 1]'; 
        typeDBV(kk) = 0; % FOT - front of train
        %# fastindex
        FOT = 1;
    else
        iV(:,kk)    = [n-1 n-2 n]'; 
        typeDBV(kk) = -1; % EOT - end of train
        %# fastindex
        EOT = 1;
    end
    %# fastindex
    k = k+1;
end

rugrel = epsilon./D; 

% Initializations of derivatives in integration method
zerijj   = zeros(size(jj));
drodto   = zerijj; 
dudto    = zerijj; 
dQdto    = zerijj;
d2rodt2o = zerijj; 
d2udt2o  = zerijj; 
d2Qdt2o  = zerijj;