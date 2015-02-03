
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
%                          LOCO struct array                              %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % [SBB] % % % % %


% fv:         THESE TWO FIELDS ARE INITIALIZED IN CAR_LOCOGUI
% ft:         BUT THEY ARE NOT USED ANYWHERE. PROBABLY CAN BE REMOVED       [s!]

% eltcv:      Electrodynamic traction characteristic, velocity vs force
% pfv:        Polynomial terms for the approximation stack together. First 
%             element is the number of third order polynomials.  
%             Note that velocity is first converted to [m/s]  
% ftv:        Force    points respecting the defined delta for the approx.
% v:          Velocity points respecting the defined delta for the approx.
% cv:         Set to 1 if characteristic is defined

% eltct:      Electrodynamic traction characteristic, time vs force 
% pft:        Polynomial terms for the approximation stack together
% ftt:        Force points respecting the defined delta for the approx.
% t:          Time  points respecting the defined delta for the approx.
% ct:         Set to 1 if characteristic is defined

% elbcv:      Electrodynamic braking  characteristic, velocity vs force
% pfvb:       Polynomial terms for the approximation stack together
% fbv:        Force    points respecting the defined delta for the approx.
% vb:         Velocity points respecting the defined delta for the approx.
% cvb:        Set to 1 if characteristic is defined

% elbct:      Electrodynamic braking  characteristic, time vs force
% pftb:       Polynomial terms for the approximation stack together
% fbt:        Force points respecting the defined delta for the approx.
% tb:         Time  points respecting the defined delta for the approx.
% ctb:        Set to 1 if characteristic is defined

% eltts:      Manages the delay of activation but it is not usefull because
%             a delay can be substituted by a submanoeuvre where the loco
%             does nothing                                                  [?]
% elbts:      The same but for electrodynamic braking instead of traction  
% ploco:      Position of locomotive in the train
% tig:        Traction insertion gradient
% trg:        Traction removal gradient
% nm:         Number of manoeuvres

% man:        10-element row vector initialized to -1 with the following
%             structure
%             - man(1):   4-digit integer (bit logic). MSB -> LSB:
%                         traction status,  e.d brake status,  
%                         e.p brake status, pneum. brake status
%                         i.e: man(1) = 1 means only pneum. brake active
%             - man(2):   Pressure [bar]. Set to -1 if train doing nothing  
%                         (man(1)=0) or pneumatic brake inactive           
%             - man(3-7): The control value is stored in the appropriate
%                         cell. For example, if control type is 2, the
%                         value wll be stored in man(4) etc
%             - man(8):   Vehicle must be specified if the control variable
%                         is pressure (last two types)
%             - man(9):   Delay [sec]
%             - man(10):  Percentage of application [0,1]

% trac:       Line vector with nm elements, indicating traction status 
%             during each manoeuvre
% elettrodyn: Line vector with nm elements, indicating e.d brake status  
% elettropn:  Line vector with nm elements, indicating e.p brake status 
% pn:         Line vector with nm elements, indicating pneum. brake status 
% ts:         Time index to manage force-time characteristic                [?]
% tg:         Used to manage changing in percentage of application          [?]
% Feldyn:     Calculated Electrodynamic braking force
% Ftrac:      Calculated traction force

% ds:         Extra field, added if a manoeuvre is controlled by distance.
%             Position of the loco at the beginning of the sub-manoeuvre 

