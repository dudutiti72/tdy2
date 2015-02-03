function [corsabg,corsadg,dampbg,dampdg,indexes,lwag,Mvt,nveicoli,posinput,...
    stiffbg,stiffdg,train,vpl,vpu] = extr_var(nveicoli,train)

znm1 = zeros(1,nveicoli-1); z = zeros(1,nveicoli);
corsabg = znm1; corsadg = znm1; dampbg = znm1; dampdg = znm1;
stiffbg = znm1; stiffdg = znm1; vpl = znm1; vpu = znm1; 
indexes = struct('blockbrake',[],'diskbrake',[]);
% TODO: this funciotn is NOT generale, because it assumes same bg dg for front and
% rear
lwag = z; Mvt = z; posinput = 1:nveicoli;
for ii = 1:nveicoli
    lwag(ii) = train(ii).lwag;
    Mvt(ii) = 1000 * (train(ii).tare *(train(ii).prot + 1) + train(ii).load);
    corsabg(1:length(train(ii).bgsf),ii) = 0.001 * train(ii).bgsf';
    corsadg(1:length(train(ii).dgsf),ii) = 0.001 * train(ii).dgsf';
    dampbg(ii) = train(ii).bgdf; dampdg(ii) = train(ii).dgdf;
    stiffbg(1:length(train(ii).bgff),ii) = 1e3 * train(ii).bgff(:,1);
    stiffdg(1:length(train(ii).dgff),ii) = 1e3 * train(ii).dgff(:,1);
    vpu(1:2,ii) = [train(ii).bgvpuf;train(ii).dgvpuf];
    vpl(1:2,ii) = [train(ii).bgvplf;train(ii).dgvplf];

    h=1.18;

    if not(isempty(train(ii).bbtype)) && train(ii).pbb > 0
        if isempty(train(ii).Fk)
            indexes.blockbrake = [indexes.blockbrake ii];
        elseif train(ii).Fk > 0
            indexes.blockbrake = [indexes.blockbrake ii];
        end
    end
    if not(isempty(train(ii).dbtype)) && train(ii).pdb > 0
        if isempty(train(ii).Fk)
            indexes.diskbrake = [indexes.diskbrake ii];
        elseif train(ii).Fk > 0
            indexes.diskbrake = [indexes.diskbrake ii];
        end
    end
    
    % Computation of the physical parameters when them are not defined by the User
    if not(isempty(train(ii).Fk)) && train(ii).Fk > 0
        % The User defined the maximum normal force and TrainDy adjusts the parameters
        % in order to obtain that force.
        if not(isempty(train(ii).bbtype))
            % The vehicle has a block brake device
            BWbFk = modif_bw(train(ii).Fk,train(ii).nbs,train(ii).typeSh);
            [Bcomp,k,I,S,SFd] = BWc2(BWbFk,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                train(ii).Ff,train(ii).Fr,train(ii).bblp);
            train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
            train(ii).bbiG = I; train(ii).bbS = S*1e-2; 
        end;
        if not(isempty(train(ii).dbtype))
            % The vehicle has a disk brake device
            %train(ii).X = comp_X(train(ii).Fk,train(ii).dblp);
            % TODO It can be improved by setting in input the load
            % pressuree
            train(ii).X = comp_X(train(ii).Fk,3.8);
        end
    elseif isempty(train(ii).Fk)
        % It is necessary to distiguish among loco and wago
        if strcmp(train(ii).type,'loco')
            if strcmp(train(ii).bbtype,'BLOCK_BW')
                [Bcomp,k,I,S,SFd] = BWc2(train(ii).bbbw,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
                train(ii).bbiG = I; train(ii).bbS = S*1e-2;
            end;
            if strcmp(train(ii).dbtype,'DISK_BW')
                %h=1.18; % As in UIC 544-1
                train(ii).X = train(ii).dbbw/(train(ii).dblp*h);
            end
        elseif strcmp(train(ii).type,'wago')
            wmass = train(ii).tare + train(ii).load;
            if strcmp(train(ii).bbtype,'BLOCK_BW_EL')
                [Bcomp,k,I,S,SFd] = BWc2(train(ii).bbbwl,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                if wmass < train(ii).bbbwi
                    Pempty = BWempty2(train(ii).bbbwe,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                        train(ii).Ff,train(ii).Fr,S,I);
                    [Bcomp,k,I,S,SFd] = BWc2(train(ii).bbbwe,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                        train(ii).Ff,train(ii).Fr,Pempty);
                    S = S*Pempty/train(ii).bblp;
                end                    
                train(ii).rendtim = 0.83; % As in UIC 544-1
                train(ii).bbiG = I; train(ii).bbS = S*1e-2; %train(ii).pbe = [];
            end;
            if strcmp(train(ii).bbtype,'BLOCK_BW_AC')
                Bauto = compB(train(ii).bbauto,wmass);
                [Bcomp,k,I,S,SFd] = BWc2(Bauto,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
                train(ii).bbiG = I; train(ii).bbS = S*1e-2;
            end;
            if strcmp(train(ii).bbtype,'BLOCK_SI')
                % Useful only for diagnostic purposes
                if strcmp(train(ii).type,'wago')
                    train(ii).bbbwl = UIC_BW(train(ii).Ff,train(ii).Fr,train(ii).bbiG,...
                        train(ii).na,train(ii).nbs,train(ii).bblp,train(ii).rendtim,...
                        train(ii).bbS,train(ii).typeSh);
                    train(ii).bbbwe = UIC_BW(train(ii).Ff,train(ii).Fr,train(ii).bbiG,...
                        train(ii).na,train(ii).nbs,train(ii).bbep,train(ii).rendtim,...
                        train(ii).bbS,train(ii).typeSh);
                    if wmass < train(ii).bbbwi
                        train(ii).bbS = train(ii).bbS*train(ii).bbep/train(ii).bblp;
                    end
                elseif strcmp(train(ii).type,'loco')
                    train(ii).bbbwl = UIC_BW(train(ii).Ff,train(ii).Fr,train(ii).bbiG,...
                        train(ii).na,train(ii).nbs,train(ii).bblp,train(ii).rendtim,...
                        train(ii).bbS,train(ii).typeSh);
                    train(ii).bbbwe = train(ii).bbbwl;
                end
            end;
            
            
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
            end
            if strcmp(train(ii).dbtype,'DISK_SI')
                if strcmp(train(ii).type,'wago')
                    if wmass < train(ii).dbbwi
                        train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                            train(ii).dbsr*train(ii).dbse*train(ii).dbdr*train(ii).dbwr)...
                            *(train(ii).dbep/train(ii).dblp);
                    else
                        train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                            train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
                    end
                elseif strcmp(train(ii).type,'loco')
                    train(ii).X1 = (train(ii).dbS*train(ii).dbfr*train(ii).dbfe*...
                        train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
                end
                train(ii).X2 = (train(ii).dbcf*train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr);
            end
                
        end
    end
    
end

end

function Bauto = compB(auto,wmass)

% Index of the last mass below wmass in brake definition
r = find(auto(:,1) < wmass,1);
if isempty(r), error('Mass too low'); end;
if r == size(auto,1), error('Mass too high'); end;
d = (auto(r+1,1)-auto(r,1));
s = (auto(r+1,2)-auto(r,2))/d;
p = auto(r,2) + s*wmass; % Percentage of brake weight
if p > 3, p = p*1e-2; end
Bauto = p*wmass;

end

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
