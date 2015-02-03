function [indexes,lwag,Mvt,posinput,train] = extr_var2(nveicoli,train)

% EXTR_VAR2 This function calculates all the train fields necessary for the
%           brake force calculation later on, taking into account the brake 
%           system of each vehicle as well as its description (Through
%           Braked Weight or Brake Cylinder characteristics). Moreover the
%           structure indexes is built to speed up the computation of the
%           braking forces.
%
% INPUTS    nveicoli: Number of vehicles
%           train:    Struct array with info about every vehicle
%
% OUTPUTS   indexes:  Struct array with position of wagons equipped with
%                     disk or block brake system
%           lwag:     Vector with vehicle lengths [m]
%           Mvt:      Vector with total mass of every vehicle including
%                     rotary masses [kg]
%           posinput: Vector with indices, from 1 to nveicoli 
%           train:    Updated struct array

z       = zeros(1,nveicoli);
indexes = struct('blockbrake',[],'diskbrake',[]);

% TODO: this function is NOT general, because it assumes same bg dg for     % [s!] This comment does not make sense here any more since extr_var2 does not  
% front and rear                                                            % deal with coupling gear info any more (extr_var used to..) 

lwag = z; Mvt = z; posinput = 1:nveicoli;

for ii = 1:nveicoli
    lwag(ii) = train(ii).lwag;
    Mvt(ii)  = 1000*(train(ii).tare *(train(ii).prot + 1) + train(ii).load);

    % Initializations
    h = 1.18; % UIC 540 disk brake coefficient
    
    if not(isempty(train(ii).bbtype)) && train(ii).pbb > 0
        if (not(isempty(train(ii).Fk)) && train(ii).Fk > 0) || isempty(train(ii).Fk)
            indexes.blockbrake = [indexes.blockbrake ii];
        end
    end
    if not(isempty(train(ii).dbtype)) && train(ii).pdb > 0
        if (not(isempty(train(ii).Fk)) && train(ii).Fk > 0) || isempty(train(ii).Fk)
            indexes.diskbrake = [indexes.diskbrake ii];
        end
    end
    
    % Computation of the physical parameters when they are not defined by the User
    if not(isempty(train(ii).Fk)) && train(ii).Fk > 0
        if not(isempty(train(ii).bbtype))
            % The brake force will be computed by means of an activation pressure,
            % which is equal to the pressure in Brake Cylinder for the application
            % stroke. The model is not fully physical but the results in terms of
            % stopping distance can be made accurate by means of UIC leaflet.
        end        
        if not(isempty(train(ii).dbtype))
            % The vehicle has a disk brake device
            % train(ii).X = comp_X(train(ii).Fk,train(ii).dblp);
            % TODO It can be improved by setting in input the load
            % pressuree
            train(ii).X = comp_X(train(ii).Fk,3.8);                         % [b] comp_X function was missing from the directory 
        end                                                                 % of the latest version
    elseif isempty(train(ii).Fk)
        % It is necessary to distiguish between loco and wago
        if strcmp(train(ii).type,'loco')
            if strcmp(train(ii).bbtype,'BLOCK_BW')
                [I,na,nbs,S,wrn] = BWc3(train(ii).bbbw,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                if not(isempty(wrn)), wrn = [wrn ' at vehicle ' num2str(ii)]; warning(wrn); end
               
                train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
                train(ii).bbiG    = I; 
                train(ii).bbS     = S*1e-2; % [dm^2]                
                train(ii).na      = na; 
                train(ii).nbs     = nbs; 
            end
            if strcmp(train(ii).dbtype,'DISK_BW')
                %h=1.18; % As in UIC 544-1
                train(ii).X = train(ii).dbbw/(train(ii).dblp*h);
            end
            if strcmp(train(ii).dbtype,'DISK_SI')
                train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                    train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
                train(ii).X2 = (train(ii).dbcf*train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
            end
        elseif strcmp(train(ii).type,'wago')
            wmass = train(ii).tare + train(ii).load;
            if strcmp(train(ii).bbtype,'BLOCK_BW_EL')
                [I,na,nbs,S,wrn] = BWc3(train(ii).bbbwl,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                if not(isempty(wrn)), wrn = [wrn ' at vehicle ' num2str(ii)]; warning(wrn); end
                Pempty = BWempty2(train(ii).bbbwe,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,S,I);
                if wmass < train(ii).bbbwi
                    [I,na,nbs,S,wrn] = BWc3(train(ii).bbbwe,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                        train(ii).Ff,train(ii).Fr,Pempty);
                    if not(isempty(wrn)), wrn = [wrn ' at vehicle ' num2str(ii)]; warning(wrn); end
                    S = S*Pempty/train(ii).bblp; 
                end                    
                train(ii).bbep    = Pempty;
                train(ii).rendtim = 0.83; % As in UIC 544-1
                train(ii).bbiG    = I; 
                train(ii).bbS     = S*1e-2;
                train(ii).na      = na; 
                train(ii).nbs     = nbs; 
            end
            if strcmp(train(ii).bbtype,'BLOCK_BW_AC')
                Bauto = compB(train(ii).bbauto,wmass);
                train(ii).bbbwl   = Bauto; % For diagnostic pourpose
                [I,na,nbs,S,wrn]  = BWc3(Bauto,train(ii).typeSh,train(ii).na, ...
                    train(ii).nbs,train(ii).Ff,train(ii).Fr,train(ii).bblp);
                if not(isempty(wrn)), wrn = [wrn ' at vehicle ' num2str(ii)]; warning(wrn); end
                train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
                train(ii).bbiG    = I; 
                train(ii).bbS     = S*1e-2;
                train(ii).na      = na; 
                train(ii).nbs     = nbs; 
                
%                 % Computation of Fdyn
%                 [Fdyn,nbs] = comp_Fdyn(Bauto,train(ii).typeSh);
%                 train(ii).Fdyn = Fdyn; train(ii).rendtim = 0.83+1e-15; train(ii).nbs2 = nbs;
            end

            if strcmp(train(ii).bbtype,'BLOCK_SI')                          % [b] This if condition is still inside the condition
                % Useful only for diagnostic purposes                       % type == wago and therefore it does not make sense  
                                                                            % to check for the type again. It should be transfered
                                                                            % after the end of the current if condition to affect both
                if strcmp(train(ii).type,'wago')                            % types of vehicles. Not important for the results though..

                    train(ii).bbbwl = UIC_BW(train(ii).Ff,train(ii).Fr,train(ii).bbiG,...
                        train(ii).na,train(ii).nbs,train(ii).bblp,train(ii).rendtim,...
                        train(ii).bbS,train(ii).typeSh);
                    train(ii).bbbwe = UIC_BW(train(ii).Ff,train(ii).Fr,train(ii).bbiG,...
                        train(ii).na,train(ii).nbs,train(ii).bbep,train(ii).rendtim,...
                        train(ii).bbS,train(ii).typeSh);
                    if wmass < train(ii).bbbwi
                        %train(ii).bbS = train(ii).bbS*train(ii).bbep/train(ii).bblp;
                        train(ii).bbS = train(ii).bbS*train(ii).bbep/train(ii).pBC;
                    end
                elseif strcmp(train(ii).type,'loco')
                    train(ii).bbbwl = UIC_BW(train(ii).Ff,train(ii).Fr,train(ii).bbiG,...
                        train(ii).na,train(ii).nbs,train(ii).bblp,train(ii).rendtim,...
                        train(ii).bbS,train(ii).typeSh);
                    train(ii).bbbwe = train(ii).bbbwl;
                end
            end
            
            if strcmp(train(ii).dbtype,'DISK_BW_EL')
                if wmass < train(ii).dbbwi
                    train(ii).X = train(ii).dbbwe/(train(ii).dblp*h);
                else
                    train(ii).X = train(ii).dbbwl/(train(ii).dblp*h);
                end
            end
            if strcmp(train(ii).dbtype,'DISK_BW_AC')
                Bauto = compB(train(ii).dbauto,wmass);
                train(ii).X = Bauto/(train(ii).dblp*h);
                train(ii).dbbwl = Bauto;
            end
            if strcmp(train(ii).dbtype,'DISK_SI')                           % [b] The same as before, this condition should be transfered
                if strcmp(train(ii).type,'wago')                            % outside the current if condition and in addition, the part 
                    if wmass < train(ii).dbbwi                              % where the DISK_SI type is checked for locomotives above can be
                                                                            % removed as X1 and X2 are calculated by the same equations here
                                                                            
                        %train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                        %    train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr)...
                        %    *(train(ii).dbep/train(ii).dblp);
                        train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                            train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr)...
                            *(train(ii).dbep/train(ii).pBC);
                    else
                        train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                            train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
                    end
                elseif strcmp(train(ii).type,'loco')
                    train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                        train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
                end
                train(ii).X2 = (train(ii).dbcf*train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
                % It is necessary to compute the braked weight
            end
                
        end
    end
    
end

end

% [s!] Remove these two functions as they are not used any more in the code

function BW = modif_bw(Fk,nbs,typeSh)
Fd = Fk/nbs;
if strcmp(typeSh,'Bg')
    % Bg shoes
    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
end
k = a0+a1*Fd+a2*Fd^2+a3*Fd^3;
BW = k* Fk / 9.81;
end

function [Fdyn,nbs] = comp_Fdyn(B,typeSh)
if strcmp(typeSh,'Bg')
    % Bg shoes
    a0 = 2.145; a1 = -5.38e-2; a2 = 7.8e-4; a3 = -5.36e-6;
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    a0 = 2.137; a1 = -5.14e-2; a2 = 8.32e-4; a3 = -6.04e-6;
end
nbs = 1; trovato = 1;
while trovato && nbs < 100
    nbs = nbs + 1;
    c = [a3 a2 a1 a0 -9.81*B/nbs];
    dummy = roots(c);
    if any(imag(dummy) == 0)
        r = zeros(4,1);
        for ii = 1:4
            if isreal(dummy(ii))
                r(ii) = dummy(ii);
            end
        end
        r(r==0) = []; % Removing uneuseful entries
        trovato = 0;
    end
end
if nbs >= 100
    error('The value of braked weight cannot be handled');
end
% The solution has to fall in the prescribed interval (according to the UIC 544)
if strcmp(typeSh,'Bg')
    % Bg shoes
    Fdyn = nbs * r(r > 5 & r < 40);
elseif strcmp(typeSh,'Bgu')
    % Bgu shoes
    Fdyn = nbs * r(r > 5 & r < 55);
end


end