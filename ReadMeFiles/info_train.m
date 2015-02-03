
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
%                          Train struct array                             %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % [SBB] % % % % %


%% General info                                         (name in conf file)

% nconf:  Name of configuration file (without file extension)   (confName)

% name:   Model of vehicle
% type:   'Loco' for locomotive, 'Wago' for wagon
% mano:   Manoeuvre name of locomotive, empty field for wagons  (manoeuvre)
% lwag:   Length of vehicle    [m]                              (wagLength)
% lBP:    Length of Brake Pipe defined in config. table [m]     (brPLength)
% UnC:    Not pneumatically connected vehicle                   (gpCStat)
% load:   Load of vehicle, zero for locomotives [t]             (load)
% tare:   Mass of empty vehicle [t]                             (tare)
% Fk:     Sum of maximum contact forces between shoes (or disk) 
%         and wheel. Braking  force  is  computed  by scaling  the 
%         maximum contact force with the current pressure in the 
%         Brake Cylinder (Also seen as SFdyn in code)           (fk)
% pBCexp: Brake Cylinder experimental target pressure [bar]     (bcExpTp)
% pBC     Brake Cylinder nominal      target pressure [bar]     (bcNomTp)
% bblp:   Block Brake Cylinder pressure for load condition
%         3.8 [bar] Imposed by UIC 544.1
% dblp:   Disk Brake Cylinder pressure for load condition
%         3.8 [bar] Imposed by UIC 544.1
% CV:     Control Valve model                                   (cv)
% CVactv: Control Valve status                                  (cvStat)
% RgBK:   Brake regime for each vehicle                         (brReg)
% t95:    Time to reach 95%  of maximum pressure in BC     [s]  (ft95)
% tmx:    Time to reach 100% of maximum pressure in BC     [s]  (ft100)
% pbb:    Percentage of block brake system                      (contBl)
% pdb:    Percentage of disk  brake system                      (contDi)
% gap:    If positive, distance between rear buffers of vehicle
%         and front buffers of subsequent vehicle If negative,
%         it defines the preload of the system and the value
%         corresponds to the relative approach of the draw
%         gears when the buffers are coupled                    (gap)
% nbgf:   Model of front buffer gears                           (bufGearsF)
% nbgr:   Model of rear  buffer gears                           (bufGearsR)
% ndgf:   Model of front draw   gear                            (drGearsF)
% ndgr:   Model of rear  draw   gear                            (drGearsR)
% frlaw:  BLOCK friction law 
%         If vehicle is disk braked, set to [] (CONST_FRICT_COEFF in conf)
%         If standard block friction law is selected in configuration,
%         set to number ($EXT_BLOCK_FRICTION_LAWS[number] in config). 0 for 
%         Karwatzki, 1 for BZA, 2 for OSS.
%         Note that when a vehicle is added in configuration, the default
%         friction law is loaded and it can either be changed or not.
%         If user defined fr. law is defined, set to name of law (frictLaw) 
% test:   Test name, assigned only to first vehicle


% prot:   Percentage of rotary masses [0,1]
% na:     Number of axes
% L:      Length of Brake Pipe. Equal to lBP field and if it is empty,
%         extracted from vehicle's txt [m]                                  [s!] Why don't just assign a value to lBP if it is empty? We don't need two fields for the same thing
% D:      Brake Pipe internal diameter [m]

%% Locomotive info

% K:       Concentrated pressure loss factor of hose couplings (Empty if 0)  
% DBV:     Driver's Brake Valve model

% BRAKE SYSTEM FIELDS FOR LOCOMOTIVES (5 modes)

% 1.BLOCK BRAKES

% pbb:     Block brake contribution [0,1] 
% nbs:     Number of brake shoes
% typeSh:  Type   of brake shoes
% sB:      Section of brake shoe (Default) [mm^2]
% rendtim: Rigging efficiency (0.83)
% Ff:      Brake rigging return force (usually 1.5 [kN])
% Fr:      Counteracting force of brake rigging regulator (usually 2 [kN])

% bfl:     Block friction law
%          First, in a temp variable, bfl_temp = frlaw. If field is empty,  
%          bfl_temp is read from vehicle's txt file. Then, the following 
%          convention is applied:
%          Set to 2 for 'Karwatzki' (i.e when frlaw = 0)
%                 4 for 'OSS'
%                 5 for 'BZA'
%                99 for 'User defined law' with additional fields: mbfc 
%                   (model of block fricrion coefficient - name stored), 
%                   Pfc (Array with friction coefficients), fcv (Vector
%                   with velocity values), fcp (Vector with specific 
%                   pressure values)

% 1.1 Block brake with Brake Cylinder characteristics
%
% bbtype:  Set to 'BLOCK_SI'
% bbiG:    Rigging ratio
% bbs:     Block cylinder section [dm^2]
%
% 1.2 Block brake with Braked Weight characteristics
%
% bbtype:  Set to 'BLOCK_BW'
% bbbw:    Block braked weight


% 2.DISK BRAKES

% pdb:     Disk brake contribution [0,1] 
% dfl:     Disk friction law
%          Either scalar with a constant friction coeff (usually 0.35)
%          or a friction law described by a piecewise 3rd order 
%          polynomial approximation. dfl(1,2:end) contains the polynomial 
%          terms stack together (4 terms for each pol). dfl(2,1) contains
%          the current position index to speed up search process. 
%          dfl(2,2:4:end) contains the discrete X values (V*Rm/Rr). 
%          dfl(1,1) = 100 (To distinguish from constant friction 
%          coefficient in brakeforce.m)                                     [s!] could check dimension instead  

% 2.1 Disk brake with Brake Cylinder characteristics    
%
% dbtype:  Set to 'DISK_SI'
% dbS:     Disk Cylinder Section [dm^2]
% dbfr:    Disk brake first rigging
% dbsr:    Disk brake second rigging
% dbfe:    Disk brake first efficiency
% dbse:    Disk brake second efficiency
% dbcf:    Counter acting force [kN]
% dbdr:    Disk radius  [m]
% dbwr:    Wheel Radius [m]
%
% 2.2 Disk brake with Braked Weight characteristics 
%
% dbtype:  Set to 'DISK_BW'
% dbbw:    Disk brake braked weight


% 3. ELECTRODYNAMIC BRAKES   
%
% elbcv:   Electrodynamic braking characteristic velocity   (Array)
% elbct:   Electrodynamic braking characteristic time       (Array)


% TRACTION
%
% eltcv:   Electrodynamic traction characteristic velocity  (Array)
% eltct:   Electrodynamic traction characteristic time      (Array)

% tig:     Traction insertion gradient
% trg:     Traction removal   gradient

%% Wagon info

% BRAKE SYSTEM ELEMENTS FOR WAGONS (6 modes)

% 1.BLOCK BRAKES 

% pbb:     Block brake contribution [0,1] 
% nbs:     Number of brake shoes
% typeSh:  Type   of brake shoes
% sB:      Section of brake shoe (Default) [mm^2]

% rendtim: Rigging efficiency
% Ff:      Brake rigging return force (usually 1.5 kN)
% Fr:      Counteracting force of brake rigging regulator (usually 2kN)

% bfl:     Block friction law (Same as bfl for locomotives) 


% 1.1 Block brake with Brake Cylinder characteristics + 'Empty\Load' device
%
% bbtype:  Set to 'BLOCK_SI'
% bbbwi:   Block brake braked weight inversion (mass)
% bbS:     Block Cylinder Section [dm^2]
% bbiG:    Rigging ratio
% bbep:    Empty pressure [bar]

% 1.2 Block brake with Braked Weight characteristics  + 'Empty\Load' device
%
% bbtype:  Set to 'BLOCK_BW_EL'
% bbbwe:   Braked weight when empty [t]
% bbbwl:   Braked weight when loaded [t]
% bbbwi:   Block inversion mass [t]

% 1.3 Block brake with 'Auto-Continous' system
%
% bbtype:  Set to 'BLOCK_BW_AC'
% bbauto:  Array containing points of block brake autocontinous system


% 2.DISK BRAKES 

% pdb:     Disk brake contribution [0,1] 
% dfl:     Disk friction law (See dfl for locomotives) 

% 2.1 Disk brake with Brake Cylinder characteristics  + 'Empty\Load' device
%
% bbtype:  Set to 'DISK_SI'
% dbS:     Disk Cylinder Section [dm^2]
% dbbwi:   Disk brake braked weight inversion (mass)
% dbep:    EmptyPressure [bar]
% dbfr:    Disk brake first rigging
% dbsr:    Disk brake second rigging
% dbfe:    Disk brake first efficiency
% dbse:    Disk brake first efficiency
% dbcf:    Counter acting force [kN]
% dbdr:    Disk radius  [m]
% dbwr:    Wheel Radius [m]

% 2.2 Disk brake with Braked Weight characteristics   + 'Empty\Load' device
%
% bbtype:  Set to 'DISK_BW_EL'
% bbbwe:   Braked weight when empty
% bbbwl:   Braked weight when loaded
% bbbwi:   Block inversion mass

% 2.3 Disk brake with 'Auto-Continous' system
%
% bbtype:  Set to 'DISK_BW_AC'
% dbauto:  Array containing points of block brake autocontinous system

%% Coupling info

% nbgf:    Buffer gear front (Model)
% bgdf:    Buffer gear damping coefficient      
%          Note: 0 if not defined. In this case both loading and unloading
%          curves must be provided, otherwise just the loading curve and 
%          the damping coefficient
% bgff:    Buffer gear force.  Two-column array with force values [N]
% bgsf:    Buffer gear stroke. Vector with stroke values [m]
% bgvplf:  Buffer gear load   limiting velocity [m/s]
% bgvpuf:  Buffer gear unload limiting velocity [m/s]
% bgccf:   Buffer gear central coupling         
%          Note: Must coincide with the bgccr of the next vehicle
% bgvdf:   Buffer gear viscous damping coefficient [kN s/m]   

% nbgr:    Buffer gear rear (Model). Same fields as Buffer gear front
% bgdr:    Buffer gear damping coefficient    
% bgfr:    Buffer gear force
% bgsr:    Buffer gear stroke
% bgvplr:  Buffer gear load   limiting velocity 
% bgvpur:  Buffer gear unload limiting velocity
% bgccr:   Buffer gear central coupling 
% bgvdr:   Buffer gear viscous damping coefficient

% ndgf:    Draw gear front (Model). Same fields as Buffer gear front
% dgdf:    Draw gear damping coefficient    
% dgff:    Draw gear force
% dgsf:    Draw gear stroke
% dgvplf:  Draw gear load   limiting velocity  
% dgvpuf:  Draw gear unload limiting velocity
% dgccf:   Draw gear central coupling 
% dgvdf:   Draw gear viscous damping coefficient

% ndgr:    Draw gear rear (Model). Same fields as Buffer gear front
% dgdr:    Draw gear damping coefficient
% dgfr:    Draw gear force
% dgsr:    Draw gear stroke
% dgvplr:  Draw gear load   limiting velocity  
% dgvpur:  Draw gear unload limiting velocity
% dgccr:   Draw gear central coupling 
% dgvdr:   Draw gear viscous damping coefficient

% op:      Order of the polynomial approximation.
%          In Etrain is 1, in TrainDy is set to 3

% dampdg:  Grouped damping coefficients, draw gear   [dgdr dgdf]
% dampbg:  Grouped damping coefficients, buffer gear [bgdr bfdf]            [s!] Not used in computations. Maybe redundant fields

% Pbgfl:   Piecewise polynomial approx of load   curve for front buffer
% Pbgfu:   Piecewise polynomial approx of unload curve for front buffer
% Pbgrl:   Piecewise polynomial approx of load   curve for rear  buffer
% Pbgru:   Piecewise polynomial approx of unload curve for rear  buffer
% Pdgfl:   Piecewise polynomial approx of load   curve for front draw gear
% Pdgfu:   Piecewise polynomial approx of unload curve for front draw gear
% Pdgrl:   Piecewise polynomial approx of load   curve for rear  draw gear
% Pdgru:   Piecewise polynomial approx of unload curve for rear  draw gear


%% Control Valve info

% pCFAs:    Application stroke data of CV, pressure in Brake Cylinder [bar]
% tAS:      Application stroke data of CV, time [s]
% pCGAs:    Application stroke data of CV, pressure in Brake Pipe [Pa]
% pCFIf:    Inshot function    data of CV, pressure in Brake Cylinder [Pa]
% tIf:      Inshot function    data of CV, time [s]
% VCi:      Was scaling of pBC for empty-load device by 3.8 but then 
%           bypassed (=1)                                                   [?]
% timeBkP:  Brake timing, P regime, 1st element is the time to reach 
%           95% of Pmax and 2nd the time to reach Pmax [s]
% timeBkG:  Brake timing, G regime, same as timeBkP [s]
% timeRlsP: Releasing timing, P regime, 1st element is the time to reach
%           110% of Pmin and 2nd the time to reach Pmin [s]
% timeRlsG: Releasing timing, G regime, same as timeTlsP [s]
% tfBr:     Transfer function, Braking,  1st column is the pressure in 
%           Brake Pipe and 2nd the pressure in Brake Cylinder [bar]
% tfRe:     Transfer function, Releasing, 1st column is the pressure in
%           Brake Pipe and 2nd the pressure in Brake Cylinder [bar]
% DPbUPbc:  dP Brake Pipe pressure activation building-up Brake Cylinder
%           [Pa] (Given in bar to GUI and transformed in PneumDevices)
% Stkbc:    Stroke of Brake Cylinders [m]
% Vac:      Volume of Acceleration Chamber [m^3]
% Dac:      Diameter of Acceleration Chamber [m]
% DPactvCV: dP Brake Pipe pressure for activation of Acceleration Chamber
%           [Pa]
% PbUPbc:   (Pinit[bar] - train.DPbUPbc[bar] Pressure in BP below which
%           Acceleration Chamber is activated
% Pcac:     Minimum absolute pressure closing of Acceleration Chamber [Pa] 
%           Converted from relative pressure in [bar] from GUI
% Var:      Volume of Auxiliary Reservoir [m^3]
% Dar:      Diameter of Auxiliary Reservoir [m]
% dParbp:   dP imposed by check valve AR-BP [Pa]                            [?] As written in GUI
% Prnar:    Absolute pressure on running of Auxiliary Reservoir [Pa]
%           Converted from relative pressure in GUI [bar]


%% Driver's Brake Valve info for locomotives

% ebd:  Emergency braking, diameter of equivalent orifice [mm]
% sbd:  Service   braking, diameter of equivalent orifice [mm]
% red:  Releasing, diameter of equivalent orifice [mm]
% ebfl: Emergency braking, flow coefficient (-1 if Perry's law is used)
% sbfl: Service   braking, flow coefficient (-1 if Perry's law is used)
% ebfl: Releasing, flow coefficient (-1 if Perry's law is used)
% sb15: Service braking, time to achieve a drop of 1.5 bar in
%       counter-pressure shape (From 5 to 3.5 bar) [s]
% sbtd: Service braking, time of first decreasing in counter-pressure shape
%       (From 5 to 4.5 bar) [s]
% re15: Releasing, time to achieve an increasing of 1.5 bar in
%       counter-pressure shape (from 3.5 to 5 bar) [s]

%% Extra fields for Disk braked locos and wagons for brake force calculation

% X:    X value for disk braked vehicles, used to calculate the braking
%       force. Calculated by BW/(h*P) for systems described by Braked 
%       Weight characteristics and by the comp_X function when Fk is given. 
%       If the vehicle is of DISK_SI type X1 and X2 variables are 
%       calculated instead
% X1:   Quantity calculated for disk braked vehicles described by Brake 
%       Cylinder characteristics. Used to calculate the braking force in 
%       brakeforce.m
% X2:   Quantity calculated for disk braked vehicles described by Brake 
%       Cylinder characteristics. Used to calculate the braking force in 
%       brakeforce.m

%% NOT SUPPORTED FIELDS

% cc:   Used for complex model of couplings. Currently not supported by GUI [n]
% Fdyn: For a special case, see brakeforce.m. Not suppoted by GUI           [n]
% nbs2: For the same case                         

