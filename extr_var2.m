function [indexes,lwag,Mvt,posinput,train] = extr_var2(nveicoli,train)

z = zeros(1,nveicoli);
indexes = struct('blockbrake',[],'diskbrake',[]);
% TODO: this funciotn is NOT generale, because it assumes same bg dg for front and
% rear
lwag = z; Mvt = z; posinput = 1:nveicoli;
for ii = 1:nveicoli
    lwag(ii) = train(ii).lwag;
    Mvt(ii) = 1000 * (train(ii).tare *(train(ii).prot + 1) + train(ii).load);

    % Initializations
    h=1.18; % UIC 540 disk brake coefficient
    
    % Updating the indexes that speed up the computation of brake force
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
    
    % Computation of the physical parameters when them are not defined by the User
    if not(isempty(train(ii).Fk)) && train(ii).Fk > 0
        if not(isempty(train(ii).bbtype))
            % The brake force will be computed by means of an activation pressure,
            % which is equal to the pressure in brake cylinder for the application
            % stroke. The model is not fully physical but the results in terms of
            % stopping distance can be made accurate by means of UIC leaflet.
        end        
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
                [I,na,nbs,S,wrn] = BWc3(train(ii).bbbw,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                if not(isempty(wrn)), wrn = [wrn ' at vehicle ' num2str(ii)]; warning(wrn); end
                train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
                train(ii).bbiG = I; train(ii).bbS = S*1e-2;
                train(ii).na = na; train(ii).nbs = nbs; 
            end;
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
                train(ii).bbep = Pempty;
                train(ii).rendtim = 0.83; % As in UIC 544-1
                train(ii).bbiG = I; train(ii).bbS = S*1e-2;
                train(ii).na = na; train(ii).nbs = nbs; 
            end;
            if strcmp(train(ii).bbtype,'BLOCK_BW_AC')
                Bauto = compB(train(ii).bbauto,wmass);
                train(ii).bbbwl = Bauto; % For diagnostic pourpose
                [I,na,nbs,S,wrn] = BWc3(Bauto,train(ii).typeSh,train(ii).na,train(ii).nbs,...
                    train(ii).Ff,train(ii).Fr,train(ii).bblp);
                if not(isempty(wrn)), wrn = [wrn ' at vehicle ' num2str(ii)]; warning(wrn); end
                train(ii).rendtim = 0.83; % This value is assumed in accordance with the leaflet UIC 544-1
                train(ii).bbiG = I; train(ii).bbS = S*1e-2;
                train(ii).na = na; train(ii).nbs = nbs; 
                
%                 % Computation of Fdyn
%                 [Fdyn,nbs] = comp_Fdyn(Bauto,train(ii).typeSh);
%                 train(ii).Fdyn = Fdyn; train(ii).rendtim = 0.83+1e-15; train(ii).nbs2 = nbs;
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
                        %train(ii).bbS = train(ii).bbS*train(ii).bbep/train(ii).bblp;
                        train(ii).bbS = train(ii).bbS*train(ii).bbep/train(ii).pBC;
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
                train(ii).dbbwl = Bauto;
            end
            if strcmp(train(ii).dbtype,'DISK_SI')
                if strcmp(train(ii).type,'wago')
                    if wmass < train(ii).dbbwi
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