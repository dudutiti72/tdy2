function [indexes,lwag,Mvt,nveicoli,posinput,traccia,train] = translate_input(fsl,ntrack,nzug,petr)

% Initialization of the struct that contains all the information of the train
train = struct('name','loco','type','loco');

[nloco,nwago,nveicoli,ploco,pwago,train] = configuration(nzug,petr,train);
% Filling "train" with the information about locos and wagons
train = fill_train_w_l(fsl,nloco,petr,ploco,train);
train = fill_train_w_w(fsl,nwago,petr,pwago,train);
% Filling train with informations about buffing gears and draw gears.
train = fill_train_w_bg_dg(nveicoli,petr,train);
% Extraction from "train" of the variable used in tRTU
%[indexes,lwag,Mvt,nveicoli,posinput,train] = extr_var(nveicoli,train);
[indexes,lwag,Mvt,posinput,train] = extr_var2(nveicoli,train);

% % Braking force and longitudinal efforts
% [Fbrake,FlongET] = etrain_braking_force(jumpbrake,nres,pres,sibr,sifl);
traccia = tracciato(ntrack,petr);
end

function [nloco,nwago,nveicoli,ploco,pwago,train] = configuration(nzug,petr,train)
% This function "extracts" the configuration of the train from the *zug file
% It is considered the first file of the folder zug
npath = [petr '\Configuration\'];
if isempty(nzug)
    d = dir(npath);
    for ii = 3:length(d)
        if strcmp('.zug',d(ii).name(end-3:end))
            nzug = d(ii).name;
            break
        end
    end
end
train.nconf = nzug;
nfile = [npath nzug];
nf = fopen(nfile,'r');
% Number of locos
cline = trova_info('Number of Locomotives',nf);
nloco = salta_spazi(cline,'Number of Locomotives');
% Number of wagons
cline = trova_info('Number of wagons',nf);
nwago = salta_spazi(cline,'Number of wagons');
nveicoli = nloco + nwago;
if nloco >= 1
    % Loco name and position
    [ploco,train] = loco_n_p(nf,nloco,train);
else
    ploco = [];
end
if nwago >= 1
    % wagons name, position and load
    [pwago,train] = wago_n_p_l(nf,nveicoli,nwago,ploco,train);
else
    pwago = [];
end
fclose(nf);

end

function cline = trova_info(strdatrovare,nf)

nrew = 0; trovato = 0;
while nrew < 2
    while trovato == 0
        cline = fgetl(nf);
        if ~ischar(cline)
            trovato = 1;
        end;
        if not(isempty(strfind(cline,strdatrovare))) && cline(1) == strdatrovare(1)
            trovato = 1; nrew = 3;
        end;
    end
    if ~ischar(cline)
        frewind(nf);
        nrew = nrew + 1; trovato = 0;
    end
end
if nrew == 2
    cline = [];
    %error('Sorry, unable to find Your string :-(');
end
end

function iline = salta_spazi(cline,strinfo)
s = strfind(cline,strinfo) + size(strinfo,2);
e = size(cline,2); iline = [];
while not(isempty(s)) && e > s && isempty(iline)
    iline = str2num(cline(s:e));
    if isempty(iline);
        e = e-1;
    end
end

end

function [ploco,train] = loco_n_p(nf,nloco,train)

trova_info('LOCOMOTIVES:',nf); ploco = zeros(1,nloco);
for ii = 1:nloco
    cline = trova_info('Position=',nf);
    ploc = salta_spazi(cline,'Position=');
    lloco = salta_stringa(cline,'Name=');
    train(ploc,1).name = lloco; ploco(ii) = ploc;
    pbb = salta_spazi(cline,'Perc.B=');
    if not(isempty(pbb)), train(ploc).pbb = pbb*0.01; else train(ploc).pbb = 1; end;
    pdb = salta_spazi(cline,'Perc.D=');
    if not(isempty(pdb)), train(ploc).pdb = pdb*0.01; else train(ploc).pdb = 1; end;
    gapw = salta_spazi(cline,'wGap=');
    if not(isempty(gapw)), train(ploc).gap = gapw; else train(ploc).gap = 0; end;
    Fk = salta_spazi(cline,'Fk=');
    if not(isempty(Fk)), train(ploc).Fk = Fk; else train(ploc).Fk = []; end;

    UnC = salta_stringa(cline,'Coupl.BP='); % The loco ii is not pneumatically linked to the following vehicles
    if not(isempty(UnC))
        train(ploc).UnC = str2double(UnC);
    else
        train(ii).UnC = 0;
    end;
    bgfr = salta_stringa(cline,'B.front=');
    if not(isempty(bgfr)), train(ploc).nbgf = bgfr; end;
    bgre = salta_stringa(cline,'B.rear=');
    if not(isempty(bgre)), train(ploc).nbgr = bgre; end;
    dgfr = salta_stringa(cline,'D.front=');
    if not(isempty(dgfr)), train(ploc).ndgf = dgfr; end;
    dgre = salta_stringa(cline,'D.rear=');
    if not(isempty(dgre)), train(ploc).ndgr = dgre; end;
    frlaw = salta_stringa(cline,'Fr.Law='); % locos friction law from configuration file
    if not(isempty(frlaw)), train(ploc).frlaw = frlaw; end;
end

end

function sline = salta_stringa(cline,strinfo)
appo = strfind(cline,strinfo);
if not(isempty(appo))
    s =  appo (1) + size(strinfo,2);
    ind = 1;
    e = s+ind;
    sline = cline(s:e);
    %while cline(e) == ' ' && e
    while isstrprop(cline(e), 'wspace') && e
        ind = ind+1;
        e = s+ind;
    end;
    %while cline(e) ~= ' ' && e < size(cline,2)
    while not(isstrprop(cline(e), 'wspace')) && e < size(cline,2)
        e = e+1;
        sline = cline(s:e);
    end;
    % Suppression of any space ' '
    sline(isstrprop(sline, 'wspace')) = [];
else
    sline = [];
end

end

function [pwago,train] = wago_n_p_l(nf,nveicoli,nwago,ploco,train)

trova_info('WAGONS:',nf);
wago = struct('name','','load',0,'Fk',1,'frlaw',''); pwago = zeros(1,nwago);
for ii = 1:nwago
    while 1
        cline = fgetl(nf);
        if not(isempty(cline)) && cline(1) ~= '#'
            break
        end
    end
    wago(ii).name = namewag(cline);
    wago(ii).load = salta_spazi(cline,'Mlad=');
    if isempty(wago(ii).load), wago(ii).load = 0; end
    gapw = salta_spazi(cline,'wGap=');
    if not(isempty(gapw)), wago(ii).gap = gapw; else wago(ii).gap = 0; end;
    Fk = salta_spazi(cline,'Fk=');
    if not(isempty(Fk)), wago(ii).Fk = Fk; else wago(ii).Fk = []; end;
    pbb = salta_spazi(cline,'Perc.B='); % Percentage block brake
    if not(isempty(pbb)), wago(ii).pbb = pbb*0.01; else wago(ii).pbb = 1; end;
    pdb = salta_spazi(cline,'Perc.D='); % Percentage disk brake
    if not(isempty(pdb)), wago(ii).pdb = pdb*0.01; else wago(ii).pdb = 1; end;
    frlaw = salta_stringa(cline,'Fr.Law='); % wagon friction law from zug file
    if not(isempty(frlaw)), wago(ii).frlaw = frlaw; end;
    UnC = salta_stringa(cline,'Coupl.BP='); % The wagon ii is not pneumatically linked to the following vehicles
    if not(isempty(UnC))
        wago(ii).UnC = str2double(UnC);
    else
        wago(ii).UnC = 0;
    end;
    bgfr = salta_stringa(cline,'B.front=');
    if not(isempty(bgfr))
        wago(ii).nbgf = bgfr;
    else
        wago(ii).nbgf = [];
    end;
    bgre = salta_stringa(cline,'B.rear=');
    if not(isempty(bgre))
        wago(ii).nbgr = bgre;
    else
        wago(ii).nbgr = [];
    end;
    dgfr = salta_stringa(cline,'D.front=');
    if not(isempty(dgfr))
        wago(ii).ndgf = dgfr;
    else
        wago(ii).ndgf = [];    
    end;
    dgre = salta_stringa(cline,'D.rear=');
    if not(isempty(dgre))
        wago(ii).ndgr = dgre;
    else
        wago(ii).ndgr = [];         
    end;

end
train(nveicoli).load = 0;
w = 0; c = 1;
for ii = 1:nveicoli
    if nveicoli == 1 || ii ~= ploco(c)
        w = w+1;
        train(ii).name = wago(w).name;
        train(ii).load = wago(w).load;
        train(ii).gap = wago(w).gap;
        train(ii).Fk = wago(w).Fk;
        train(ii).pbb = wago(w).pbb;
        train(ii).pdb = wago(w).pdb;
        train(ii).frlaw = wago(w).frlaw;
        train(ii).UnC = wago(w).UnC;
        train(ii).nbgf = wago(w).nbgf;
        train(ii).nbgr = wago(w).nbgr;
        train(ii).ndgf = wago(w).ndgf;
        train(ii).ndgr = wago(w).ndgr;
        pwago(w) = ii;
    elseif c < length(ploco)
        c = c+1;
    end

end

end

function nwag = namewag(cline)
s = 1;
while isstrprop(cline(s), 'wspace') %cline(s) == ' '
    s = s+1;
end;
e = s+1;
while not(isstrprop(cline(e), 'wspace')) %cline(e) ~= ' '
    nwag = cline(s:e);
    e = e+1;
end;

end

function train = fill_train_w_l(fsl,nloco,petr,ploco,train)

% This function "extracts" the information about the locos in the train
%#RK-modified npath = [petr '/Lok/'];
% npath = [petr '/Lok/'];
npath = [petr fsl 'Lok' fsl];
for ii = 1:nloco
    train(ploco(ii)).tig = 0; train(ploco(ii)).trg = 0; % Backwards compaatibility
    train(ploco(ii)).type = 'loco';
    nfile = [npath train(ploco(ii)).name '.lok'];
    nf = fopen(nfile,'r');
    cline = trova_info('Length',nf);
    lwag = salta_spazi(cline,'Length');
    train(ploco(ii)).lwag = lwag;
    cline = trova_info('Mass',nf);
    mloc = salta_spazi(cline,'Mass');
    train(ploco(ii)).tare = mloc;
    train(ploco(ii)).load = 0;
    cline = trova_info('Rotary perc.',nf);
    prot = salta_spazi(cline,'Rotary perc.');
    train(ploco(ii)).prot = prot/100;
    cline = trova_info('Concentrated pressure loss factor in hose couplings [-]',nf);
    if not(isempty(cline))
        K = salta_spazi(cline,'Concentrated pressure loss factor in hose couplings [-]');
        train(ploco(ii)).K = K;
    else
        train(ploco(ii)).K = [];
    end
    % TrainDy manages 5 types of brake input for locos
    % 1) Block Brake with section, rigging ratio and so on (higt-low pressure device)
    % 2) Block Brake with brake weight
    % 3) Disk brake with physical parametres  (higt-low pressure device)(Karbstein)
    % 4) Disk brake with brake weight
    % 5) Elettrodynamic brake and traction
    % TrainDy can manage locos that brake at the same time with disk brake,
    % block brake and elettrodynamic brake

    train(ploco(ii)).bbtype = ''; train(ploco(ii)).dbtype = '';
    cline = trova_info('BLOCK_SI',nf);
    if not(isempty(cline))
        % The loco has a block brake device defined in terms of physical parameters
        train(ploco(ii)).bbtype = 'BLOCK_SI';
        train = read_common_info_block(ii,nf,ploco,petr,train);
        S = trova_info('Block Cylinder Section [dm^2]',nf);
        train(ploco(ii)).bbS = salta_spazi(S,'Block Cylinder Section [dm^2]');
        cline = trova_info('RiggingRatio',nf);
        train(ploco(ii)).bbiG = salta_spazi(cline,'RiggingRatio');
        cline = trova_info('Rigging Efficiency',nf);
        train(ploco(ii)).rendtim = salta_spazi(cline,'Rigging Efficiency');
        train(ploco(ii)).bbep = []; % Loco has a fixed weight
        
%         %SNCF MODIFICATION 
%         %For the definition of the number of axles 
%             % - Delete of the calculation numberOfShoes*0.25
%             % - Find the number of axles in parameter files
%         %Old Line   
        train(ploco(ii)).na = train(ploco(ii)).nbs*0.25; % Number of axles
%         %New Lines :         
%         cline = trova_info('Number of axes',nf);
%         naP = salta_spazi(cline,'Number of axes');
%         train(ploco(ii)).na = naP;
%         %END MODIFICATION

    end
    cline = trova_info('BLOCK_BW',nf);
    if not(isempty(cline)) && isempty(train(ploco(ii)).bbtype)
        % The loco has a block brake device defined in terms of braked weight
        train(ploco(ii)).bbtype = 'BLOCK_BW';
        train = read_common_info_block(ii,nf,ploco,petr,train);
        cline = trova_info('Block Braked Weight',nf);
        train(ploco(ii)).bbbw = salta_spazi(cline,'Block Braked Weight');
       
%         %SNCF MODIFICATION 
%         %For the definition of the number of axles 
%             % - Delete of the calculation numberOfShoes*0.25
%             % - Find the number of axles in parameter files
%         %Old Line   
        train(ploco(ii)).na = train(ploco(ii)).nbs*0.25; % Number of axles
%         %New Lines :         
%         cline = trova_info('Number of axes',nf);
%         naP = salta_spazi(cline,'Number of axes');
%         train(ploco(ii)).na = naP;
%         %END MODIFICATION
    end

    cline = trova_info('DISK_SI',nf); train(ploco(ii)).dfl = [];
    if not(isempty(cline))
        % The wagon has a disk brake device defined in terms of physical parameters
        train(ploco(ii)).dbtype = 'DISK_SI';
        cline = trova_info('Disk Friction Law',nf);
        dfl = salta_stringa(cline,'Disk Friction Law');
        if (isempty(dfl))
            error('Disk friction low or value is not defined');
        elseif isfinite(str2double(dfl))
            train(ploco(ii)).dfl = str2double(dfl);
        else
            dfl = extr_dfl(dfl,petr);
            train(ploco(ii)).dfl = dfl;
        end
        cline = trova_info('Disk Cylinder Section [dm^2]',nf);
        train(ploco(ii)).dbS = salta_spazi(cline,'Disk Cylinder Section [dm^2]'); % Disk brake Section
        %         cline = trova_info('Disk Inversion Mass',nf);
        %         train(ploco(ii)).dbbwi = salta_spazi(cline,'Disk Inversion Mass'); % Disk Brake Braked Weight Inversion
        %         cline = trova_info('EmptyPressure [bar]',nf);
        %         train(ploco(ii)).dbep = salta_spazi(cline,'EmptyPressure [bar]'); % Disk Brake Empty Pressure
        cline = trova_info('LoadPressure [bar]',nf);
        train(ploco(ii)).dblp = salta_spazi(cline,'LoadPressure [bar]'); % Disk Brake Load Pressure
        cline = trova_info('First Rigging',nf);
        train(ploco(ii)).dbfr = salta_spazi(cline,'First Rigging'); % Disk Brake First Rigging
        cline = trova_info('Second Rigging',nf);
        train(ploco(ii)).dbsr = salta_spazi(cline,'Second Rigging'); % Disk Brake Second Rigging
        cline = trova_info('First Efficiency',nf);
        train(ploco(ii)).dbfe = salta_spazi(cline,'First Efficiency'); % Disk Brake First Efficiency
        cline = trova_info('Second Efficiency',nf);
        train(ploco(ii)).dbse = salta_spazi(cline,'Second Efficiency'); % Disk Brake Second Efficiency
        cline = trova_info('Counteracting Force [kN]',nf);
        train(ploco(ii)).dbcf = salta_spazi(cline,'Counteracting Force [kN]'); % Disk Brake Counteracting forca
        cline = trova_info('Disk Radius',nf);
        train(ploco(ii)).dbdr = salta_spazi(cline,'Disk Radius'); % Disk Brake Disk Radius
        cline = trova_info('Wheel Radius',nf);
        train(ploco(ii)).dbwr = salta_spazi(cline,'Wheel Radius'); % Disk Brake Wheel Radius

    end
    cline = trova_info('DISK_BW',nf);
    if not(isempty(cline)) && isempty(train(ploco(ii)).dbtype)
        % The loco has a disk brake device defined in terms of braked weight
        train(ploco(ii)).dbtype = 'DISK_BW';
        cline = trova_info('LoadPressure [bar]',nf);
        train(ploco(ii)).dblp = salta_spazi(cline,'LoadPressure [bar]'); % Pressure for load condition
        cline = trova_info('Disk Friction Law',nf);
        dfl = salta_stringa(cline,'Disk Friction Law');
        if (isempty(dfl))
            error('Disk friction low or value is not defined');
        elseif isfinite(str2double(dfl))
            train(ploco(ii)).dfl = str2double(dfl);
        else
            dfl = extr_dfl(dfl,petr);
            train(ploco(ii)).dfl = dfl;
        end
        cline = trova_info('Disk Braked Weight',nf);
        train(ploco(ii)).dbbw = salta_spazi(cline,'Disk Braked Weight');
        train(ploco(ii)).dbdr = [];
        train(ploco(ii)).dbwr = [];
    end

    cline = trova_info('ELECTRIC_BRAKE',nf);
    train(ploco(ii)).npebv = []; train(ploco(ii)).elbcv = [0 0]; train(ploco(ii)).elbts = 0;
    train(ploco(ii)).npebt = 0; train(ploco(ii)).elbct = [0 0];
    if not(isempty(cline))
        % The loco brakes with the electrodynamic brake
        % Number of points electrodynamic brake chracteristic curve
        cline = trova_info('Number of points braking F[kN] v[km/h]',nf);
        train(ploco(ii)).npebv = salta_spazi(cline,'Number of points braking F[kN] v[km/h]');
        A = fscanf(nf,'%g %g',[2,train(ploco(ii)).npebv]);
        % Electrodynamic braking characteristic velocity
        train(ploco(ii)).elbcv = A';
        % cline = trova_info('Time braking shift',nf);
        % % Elettrodynamic time braking shift
        % train(ploco(ii)).eltbs = salta_spazi(cline,'Time braking shift');
        cline = trova_info('Number of points braking F[kN] t[s]',nf);
        train(ploco(ii)).npebt = salta_spazi(cline,'Number of points braking F[kN] t[s]');
        A = fscanf(nf,'%g %g',[2,train(ploco(ii)).npebt]);
        % Electrodynamic braking characteristic time
        train(ploco(ii)).elbct = A';
        %%% Elettrodynamic locomotive slope regime
        %cline = trova_info('Slope to reach regime',nf);
        %train(ploco(ii)).elsr = salta_spazi(cline,'Slope to reach regime');
    else
        train(ploco(ii)).npebv = [];
    end

    cline = trova_info('ELECTRIC_TRACTION',nf);
    train(ploco(ii)).npetv = 0; train(ploco(ii)).eltcv = [0 0]; train(ploco(ii)).eltts = 0;
    train(ploco(ii)).npett = 0; train(ploco(ii)).eltct = [0 0];
    if not(isempty(cline))
        % The User defined also the traction characteristic of the loco
        % Number of points electrodynamic brake chracteristic curve
        cline = trova_info('Number of points traction F[kN] v[km/h]',nf);
        train(ploco(ii)).npetv = salta_spazi(cline,'Number of points traction F[kN] v[km/h]');
        A = fscanf(nf,'%g %g',[2,train(ploco(ii)).npetv]);
        % Electrodynamic braking traction velocity
        train(ploco(ii)).eltcv = A';
        % cline = trova_info('Time traction shift',nf);
        % % Elettrodynamic time traction shift
        % train(ploco(ii)).eltts = salta_spazi(cline,'Time traction shift');
        cline = trova_info('Number of points traction F[kN] t[s]',nf);
        train(ploco(ii)).npett = salta_spazi(cline,'Number of points traction F[kN] t[s]');
        A = fscanf(nf,'%g %g',[2,train(ploco(ii)).npett]);
        % Electrodynamic locomotive characteristic
        train(ploco(ii)).eltct = A';
        %%% Elettrodynamic locomotive slope regime
        %cline = trova_info('Slope to reach regime',nf);
        %train(ploco(ii)).elsr = salta_spazi(cline,'Slope to reach regime');
    else
        train(ploco(ii)).npetv = [];
    end
    %     cline = trova_info('Buffers',nf);
    %     nbg = salta_stringa(cline,'Buffers');
    %     train(ploco(ii)).nbgf = nbg;
    %     train(ploco(ii)).nbgr = nbg;
    %     cline = trova_info('Draw gear',nf);
    %     ndg = salta_stringa(cline,'Draw gear');
    %     train(ploco(ii)).ndgf = ndg;
    %     train(ploco(ii)).ndgr = ndg;
    %     fclose(nf);


    cline = trova_info('Front buffers',nf);
    if isempty(cline)
        cline = trova_info('Buffers',nf);
        nbgf = salta_stringa(cline,'Buffers');
        if isempty(train(ploco(ii)).nbgf)
            train(ploco(ii)).nbgf = nbgf;
        end
        if isempty(train(ploco(ii)).nbgr)
            train(ploco(ii)).nbgr = nbgf;
        end
    else
        nbgf = salta_stringa(cline,'Front buffers');
        if isempty(train(ploco(ii)).nbgf)
            train(ploco(ii)).nbgf = nbgf;
        end
        cline = trova_info('Rear buffers',nf);
        nbgr = salta_stringa(cline,'Rear buffers');
        if isempty(train(ploco(ii)).nbgr)
            train(ploco(ii)).nbgr = nbgr;
        end
    end;

    cline = trova_info('Front draw gear',nf);
    if isempty(cline)
        cline = trova_info('Draw gear',nf);
        ndgf = salta_stringa(cline,'Draw gear');
        if isempty(train(ploco(ii)).ndgf)
            train(ploco(ii)).ndgf = ndgf;
        end
        if isempty(train(ploco(ii)).ndgr)
            train(ploco(ii)).ndgr = ndgf;
        end
    else
        ndgf = salta_stringa(cline,'Front draw gear');
        if isempty(train(ploco(ii)).ndgf)
            train(ploco(ii)).ndgf = ndgf;
        end
        cline = trova_info('Rear draw gear',nf);
        ndgr = salta_stringa(cline,'Rear draw gear');
        if isempty(train(ploco(ii)).ndgr)
            train(ploco(ii)).ndgr = ndgr;
        end
    end
    fclose(nf);


end

end


function train = fill_train_w_w(fsl,nwago,petr,pwago,train)

% This function "extracts" the information about the wagons in the train
% npath = [petr '/Wagon/'];
npath = [petr,fsl,'Wagon',fsl];
for ii = 1:nwago
    train(pwago(ii)).type = 'wago';
    nfile = [npath train(pwago(ii)).name '.fzg'];
    nf = fopen(nfile,'r');
    cline = trova_info('Tare',nf);
    mloc = salta_spazi(cline,'Tare');
    train(pwago(ii)).tare = mloc;
    cline = trova_info('Length',nf);
    lwag = salta_spazi(cline,'Length');
    train(pwago(ii)).lwag = lwag;
    cline = trova_info('Rotary perc.',nf);
    prot = salta_spazi(cline,'Rotary perc.');
    train(pwago(ii)).prot = prot/100;
    % TrainDy manages 6 types of brake input for wagons
    % 1) Block Brake with brake weight (empty load device)
    % 2) Block Brake with section, rigging ratio and so on (empty load device)
    % 3) Block brake with auto-continous device (100% percentage brake weight)
    % 4) Disk brake with auto-continous device: percentage of brake weight variable
    % according actual mass
    % 5) Disk brake with brake weight (empty load device)
    % 6) Disk brake with physical parametres (Karbstein)
    % TrainDy can manage wagons that brake at the same time with disk brake and
    % block brake.
    train(pwago(ii)).bbtype = ''; train(pwago(ii)).dbtype = '';
    cline = trova_info('BLOCK_SI',nf);
    if not(isempty(cline))
        % The wagon has a block brake device defined in terms of physical parameters
        train(pwago(ii)).bbtype = 'BLOCK_SI';
        train = read_common_info_block(ii,nf,pwago,petr,train);
        cline = trova_info('Block Inversion Mass',nf);
        train(pwago(ii)).bbbwi = salta_spazi(cline,'Block Inversion Mass');
        S = trova_info('Block Cylinder Section [dm^2]',nf);
        train(pwago(ii)).bbS = salta_spazi(S,'Block Cylinder Section [dm^2]');
        cline = trova_info('RiggingRatio',nf);
        train(pwago(ii)).bbiG = salta_spazi(cline,'RiggingRatio');
        cline = trova_info('RiggingEfficiency',nf);
        train(pwago(ii)).rendtim = salta_spazi(cline,'RiggingEfficiency');
        cline = trova_info('EmptyPressure [bar]',nf);
        train(pwago(ii)).bbep = salta_spazi(cline,'EmptyPressure [bar]');
        
    end
    cline = trova_info('BLOCK_BW_EL',nf);
    if not(isempty(cline)) && isempty(train(pwago(ii)).bbtype)
        % The wagon has a block brake device defined in terms of empty/load braked
        % weight
        train(pwago(ii)).bbtype = 'BLOCK_BW_EL';
        train = read_common_info_block(ii,nf,pwago,petr,train);
        bwe = trova_info('Block Braked Weight Empty',nf);
        train(pwago(ii)).bbbwe = salta_spazi(bwe,'Block Braked Weight Empty');
        cline = trova_info('Block Inversion Mass',nf);
        train(pwago(ii)).bbbwi = salta_spazi(cline,'Block Inversion Mass');
        cline = trova_info('Block Braked Weight Load',nf);
        train(pwago(ii)).bbbwl = salta_spazi(cline,'Block Braked Weight Load');
    end
    cline = trova_info('BLOCK_BW_AC',nf);
    if not(isempty(cline)) && isempty(train(pwago(ii)).bbtype)
        % The wagon has a block brake device defined in terms of autocontinous system
        train(pwago(ii)).bbtype = 'BLOCK_BW_AC';
        train = read_common_info_block(ii,nf,pwago,petr,train);
        nptab = trova_info('Number of points block brake autocontinous',nf);
        train(pwago(ii)).bbnptab = salta_spazi(nptab,'Number of points block brake autocontinous');
        A = fscanf(nf,'%g %g',[2,train(pwago(ii)).bbnptab]);
        train(pwago(ii)).bbauto = A';
        train(pwago(ii)).bbS = []; train(pwago(ii)).bbbwe = [];
    end
    
    cline = trova_info('DISK_SI',nf); train(pwago(ii)).dfl = [];
    if not(isempty(cline))
        % The wagon has a disk brake device defined in terms of physical parameters
        train(pwago(ii)).dbtype = 'DISK_SI';
        if isempty(train(pwago(ii)).dfl)
            cline = trova_info('Disk Friction Law',nf);
            dfl = salta_stringa(cline,'Disk Friction Law');
        end
        if (isempty(dfl))
            error('Disk friction low or value is not defined');
        elseif isfinite(str2double(dfl))
            train(pwago(ii)).dfl = str2double(dfl);
        else
            dfl = extr_dfl(dfl,petr);
            train(pwago(ii)).dfl = dfl;
        end
        cline = trova_info('Disk Cylinder Section [dm^2]',nf);
        train(pwago(ii)).dbS = salta_spazi(cline,'Disk Cylinder Section [dm^2]'); % Disk brake Section
        cline = trova_info('Disk Inversion Mass',nf);
        train(pwago(ii)).dbbwi = salta_spazi(cline,'Disk Inversion Mass'); % Disk Brake Braked Weight Inversion
        cline = trova_info('EmptyPressure [bar]',nf);
        train(pwago(ii)).dbep = salta_spazi(cline,'EmptyPressure [bar]'); % Disk Brake Empty Pressure
        cline = trova_info('LoadPressure [bar]',nf);
        train(pwago(ii)).dblp = salta_spazi(cline,'LoadPressure [bar]'); % Disk Brake Load Pressure
        cline = trova_info('First Rigging',nf);
        train(pwago(ii)).dbfr = salta_spazi(cline,'First Rigging'); % Disk Brake First Rigging
        cline = trova_info('Second Rigging',nf);
        train(pwago(ii)).dbsr = salta_spazi(cline,'Second Rigging'); % Disk Brake Second Rigging
        cline = trova_info('First Efficiency',nf);
        train(pwago(ii)).dbfe = salta_spazi(cline,'First Efficiency'); % Disk Brake First Efficiency
        cline = trova_info('Second Efficiency',nf);
        train(pwago(ii)).dbse = salta_spazi(cline,'Second Efficiency'); % Disk Brake Second Efficiency
        cline = trova_info('Counteracting Force [kN]',nf);
        train(pwago(ii)).dbcf = salta_spazi(cline,'Counteracting Force [kN]'); % Disk Brake Counteracting forca
        cline = trova_info('Disk Radius',nf);
        train(pwago(ii)).dbdr = salta_spazi(cline,'Disk Radius'); % Disk Brake Disk Radius
        cline = trova_info('Wheel Radius',nf);
        train(pwago(ii)).dbwr = salta_spazi(cline,'Wheel Radius'); % Disk Brake Wheel Radius

    end
    cline = trova_info('DISK_BW_EL',nf);
    if not(isempty(cline)) && isempty(train(pwago(ii)).dbtype)
        % The wagon has a disk brake device defined in terms of empty/load braked
        % weight
        train(pwago(ii)).dbtype = 'DISK_BW_EL';
        cline = trova_info('LoadPressure [bar]',nf);
        train(pwago(ii)).dblp = salta_spazi(cline,'LoadPressure [bar]'); % Pressure for load condition
        if isempty(train(pwago(ii)).dfl)
            cline = trova_info('Disk Friction Law',nf);
            dfl = salta_stringa(cline,'Disk Friction Law');
        end
        if (isempty(dfl))
            error('Disk friction low or value is not defined');
        elseif isfinite(str2double(dfl))
            train(pwago(ii)).dfl = str2double(dfl);
        else
            dfl = extr_dfl(dfl,petr);
            train(pwago(ii)).dfl = dfl;
        end
        bwe = trova_info('Disk Braked Weight Empty',nf);
        train(pwago(ii)).dbbwe = salta_spazi(bwe,'Disk Braked Weight Empty');
        cline = trova_info('Disk Inversion Mass',nf);
        train(pwago(ii)).dbbwi = salta_spazi(cline,'Disk Inversion Mass');
        cline = trova_info('Disk Braked Weight Load',nf);
        train(pwago(ii)).dbbwl = salta_spazi(cline,'Disk Braked Weight Load');
        train(pwago(ii)).dbdr = [];
        train(pwago(ii)).dbwr = [];
    end
    cline = trova_info('DISK_BW_AC',nf);
    if not(isempty(cline)) && isempty(train(pwago(ii)).dbtype)
        % The wagon has a disk brake device defined in terms of autocontinous system
        train(pwago(ii)).dbtype = 'DISK_BW_AC';
        cline = trova_info('LoadPressure [bar]',nf);
        train(pwago(ii)).dblp = salta_spazi(cline,'LoadPressure [bar]'); % Pressure for load condition
        if isempty(train(pwago(ii)).dfl)
            cline = trova_info('Disk Friction Law',nf);
            dfl = salta_stringa(cline,'Disk Friction Law');
        end
        if (isempty(dfl))
            error('Disk friction low or value is not defined');
        elseif isfinite(str2double(dfl))
            train(pwago(ii)).dfl = str2double(dfl);
        else
            dfl = extr_dfl(dfl,petr);
            train(pwago(ii)).dfl = dfl;
        end
        nptab = trova_info('Number of points disk brake autocontinous',nf);
        train(pwago(ii)).dbnptab = salta_spazi(nptab,'Number of points disk brake autocontinous');
        A = fscanf(nf,'%g %g',[2,train(pwago(ii)).dbnptab]);
        train(pwago(ii)).dbauto = A';
        train(pwago(ii)).dbS = []; train(pwago(ii)).dbbwe = [];

    end
    %train(pwago(ii)).na = train(pwago(ii)).nbs*0.25;


    cline = trova_info('Front buffers',nf);
    if isempty(cline)
        cline = trova_info('Buffers',nf);
        nbgf = salta_stringa(cline,'Buffers');
        if isempty(train(pwago(ii)).nbgf)
            train(pwago(ii)).nbgf = nbgf;
        end
        if isempty(train(pwago(ii)).nbgr)
            train(pwago(ii)).nbgr = nbgf;
        end
    else
        nbgf = salta_stringa(cline,'Front buffers');
        if isempty(train(pwago(ii)).nbgf)
            train(pwago(ii)).nbgf = nbgf;
        end
        cline = trova_info('Rear buffers',nf);
        nbgr = salta_stringa(cline,'Rear buffers');
        if isempty(train(pwago(ii)).nbgr)
            train(pwago(ii)).nbgr = nbgr;
        end
    end;

    cline = trova_info('Front draw gear',nf);
    if isempty(cline)
        cline = trova_info('Draw gear',nf);
        ndgf = salta_stringa(cline,'Draw gear');
        if isempty(train(pwago(ii)).ndgf)
            train(pwago(ii)).ndgf = ndgf;
        end
        if isempty(train(pwago(ii)).ndgr)
            train(pwago(ii)).ndgr = ndgf;
        end
    else
        ndgf = salta_stringa(cline,'Front draw gear');
        if isempty(train(pwago(ii)).ndgf)
            train(pwago(ii)).ndgf = ndgf;
        end
        cline = trova_info('Rear draw gear',nf);   %TO DO: dovrebbe essere Rear draw gear
        ndgr = salta_stringa(cline,'Rear draw gear');
        if isempty(train(pwago(ii)).ndgr)
            train(pwago(ii)).ndgr = ndgr;
        end
    end
    fclose(nf);

end

end

function train = fill_train_w_bg_dg(nveicoli,petr,train)

npath = [petr '\Couplings\'];
% curdir = pwd;
% cd(npath);
for ii = 1:nveicoli
    nfile = [npath train(ii).nbgf '.fdr'];
    nf = fopen(nfile,'r');
    [damp,force,stroke,vpl,vpu] = extrbgdg(nf);
    fclose(nf);
    train(ii).bgdf = damp; train(ii).bgff = force; train(ii).bgsf = stroke;
    train(ii).bgvplf = vpl; train(ii).bgvpuf = vpu; train(ii).bgccf = 0; train(ii).bgvdf = 0;

    nfile = [npath train(ii).nbgr '.fdr'];
    nf = fopen(nfile,'r');
    [damp,force,stroke,vpl,vpu] = extrbgdg(nf);
    fclose(nf);
    train(ii).bgdr = damp; train(ii).bgfr = force; train(ii).bgsr = stroke;
    train(ii).bgvplr = vpl; train(ii).bgvpur = vpu; train(ii).bgccr = 0; train(ii).bgvdr = 0;


    nfile = [npath train(ii).ndgf '.fdr'];
    nf = fopen(nfile,'r');
    [damp,force,stroke,vpl,vpu] = extrbgdg(nf);
    fclose(nf);
    train(ii).dgdf = damp; train(ii).dgff = force; train(ii).dgsf = stroke;
    train(ii).dgvplf = vpl; train(ii).dgvpuf = vpu; train(ii).dgccf = 0; train(ii).dgvdf = 0;

    nfile = [npath train(ii).ndgr '.fdr'];
    nf = fopen(nfile,'r');
    [damp,force,stroke,vpl,vpu] = extrbgdg(nf);
    fclose(nf);
    train(ii).dgdr = damp; train(ii).dgfr = force; train(ii).dgsr = stroke;
    train(ii).dgvplr = vpl; train(ii).dgvpur = vpu; train(ii).dgccr = 0; train(ii).dgvdr = 0;
    % Order of the polynomial approximation: in Etrain is 1, in TrainDy is set to 3
    train(ii).op = 3;
end

end

function [damp,force,stroke,vpl,vpu] = extrbgdg(nf)

% Initializations
stroke = zeros(2,1); force = zeros(2,2);
cline = trova_info('xp0_load',nf);
vpl = salta_spazi(cline,'xp0_load');
cline = trova_info('xp0_unload',nf);
vpu = salta_spazi(cline,'xp0_unload');
cline = trova_info('Damping coefficient',nf);
if isempty(cline)
    damp = 0;
else
    damp = 0.01*salta_spazi(cline,'Damping coefficient');
end;
cline = trova_info('Load and unload characteristics',nf); c = 1;
while isempty(findstr(cline,'end'))
    cline = fgetl(nf);
    appo = str2num(cline);
    if not(isempty(appo))
        stroke(c) = appo(1); force(c,1) = appo(2);
        if damp == 0
            % If there is no damping it means that loading force and unloading force
            % are both provided.
            force(c,2) = appo(3);
        end
    end;
    c = c+1;
end

end

function [corsabg,corsadg,dampbg,dampdg,indexes,lwag,Mvt,nveicoli,posinput,stiffbg,stiffdg,train,vpl,vpu] = extr_var(nveicoli,train)

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
            % TODO: load pressure (train(ii).dblp) must be set as
            % locomotive input if disk si is used 
            train(ii).X = comp_X(train(ii).Fk,3.8);
        end
    elseif isempty(train(ii).Fk)
        % It is necessary to distiguish among loco and wago
%         if strcmp(train(ii).type,'loco')
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
%         elseif strcmp(train(ii).type,'wago')
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
                            train(ii).dbsr*train(ii).dbse*train(ii).dbdr/train(ii).dbwr)...
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
                
%         end
    end
    
end

end


function [Fbrake,FlongET] = etrain_braking_force(jumpbrake,nres,pres,sibr,sifl)

D = dir(pres);
for ii = 2:size(D,1)
    if strcmp(D(ii).name,nres)
        break
    end
end
if D(ii).bytes == 0
    Fbrake = []; FlongET = [];
else
    nfile = [pres '/' nres];
    nf = fopen(nfile,'r');
    cline = fgetl(nf);
    appo = str2num(cline);
    while isempty(appo)
        cline = fgetl(nf);
        appo = str2num(cline);
    end;
    A = appo; c = 2;
    cline = fgetl(nf);
    appo = str2num(cline);
    while not(isempty(appo))
        A(c,:) = appo; c = c+1;
        cline = fgetl(nf);
        if ischar(cline)
            appo = str2num(cline);
        else
            break
        end
    end
    fclose(nf);

    Fbrake = [A(:,1) A(:,sibr:jumpbrake:end-1)];
    FlongET = A(:,sifl:jumpbrake:end-2);
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

function XdFk = comp_X(Fkmod,loadP)
% Fkmod is the product of the normal force by the ratio of the radius of the disk
% brake and the radius of the wheel.
XdFk = Fkmod*0.35/loadP;
end

function Bauto = compB(auto,wmass)

% Index of the last mass below wmass in brake definition
r = find(auto(:,1) <= wmass,1);
if isempty(r), error('Mass too low'); end;
if r == size(auto,1), error('Mass too high'); end;
d = (auto(r+1,1)-auto(r,1));
s = (auto(r+1,2)-auto(r,2))/d;
p = auto(r,2) + s*wmass; % Percentage of brake weight
if p > 3, p = p*1e-2; end
Bauto = p*wmass;

end

function train = read_common_info_block(ii,nf,position,petr,train)

cline = trova_info('Block Friction Law',nf); %Type of friction law
if isempty(train(position(ii)).frlaw)
    bfl = salta_stringa(cline,'Block Friction Law');
else
    bfl = train(position(ii)).frlaw;
end
% if not(isempty(train(position(ii)).frlaw)) && strcmp(train(position(ii)).bbtype(1:4),'BLOC'), bfc=train(position(ii)).frlaw; end;
if  strcmp(bfl,'Karwatzki')
    train(position(ii)).bfl = 2;
elseif strcmp(bfl,'OSS')
    train(position(ii)).bfl = 4;
elseif strcmp(bfl,'BZA')
    train(position(ii)).bfl = 5;
else
    % The User defined a friction coefficient by textfile
    train(position(ii)).mbfc = bfl; % The name of the model of block friction coefficient is stored
    [fcv,fcp,fcf,Pfc] = fc_inp(petr,bfl);
    train(position(ii)).Pfc = Pfc;
    train(position(ii)).fcv = fcv;
    train(position(ii)).fcp = fcp;
    train(position(ii)).fcf = fcf;
    train(position(ii)).bfl = 99;
end;

cline = trova_info('Number of bring shoe',nf);
nbs = salta_spazi(cline,'Number of bring shoe'); % Number of bring shoes
train(position(ii)).nbs = nbs;

%    %SNCF MODIFICATION 
%    %For the definition of the number of axles 
%    % - Delete of the calculation numberOfShoes*0.25
%    % - Find the number of axles in parameter files
%    %Old Line   
   train(position(ii)).na = train(position(ii)).nbs*0.25; % Number of axles
%    %New Lines :         
%    cline = trova_info('Number of axes',nf);
%    naP = salta_spazi(cline,'Number of axes');
%    train(position(ii)).na = naP;
%    %END MODIFICATION


cline = trova_info('Type of shoe',nf);
typeSh = salta_stringa(cline,'Type of shoe'); % Type of brake shoe
train(position(ii)).typeSh = typeSh;
if strcmp(typeSh,'Bg')
    train(position(ii)).sB = 25600; %Section of the shoe expressed in mm^2 (type bg)
elseif strcmp(typeSh,'Bgu')
    train(position(ii)).sB = 40000; %Section of the shoe expressed in mm^2 (type bgu)
end

cline = trova_info('LoadPressure [bar]',nf);
train(position(ii)).bblp = salta_spazi(cline,'LoadPressure [bar]'); % Pressure for load condition
cline = trova_info('Ff [kN]',nf);
train(position(ii)).Ff = salta_spazi(cline,'Ff [kN]');
cline = trova_info('Fr [kN]',nf);
train(position(ii)).Fr = salta_spazi(cline,'Fr [kN]');
end

function dfl = extr_dfl(dfl,petr)

% This function reads the disk friction law provided in the file "dfl.dfl" and
% computes its piecewis polynomial
A = load([petr '\FricD\' dfl '.dfl']);
v = A(:,1)'/3.6; mu = A(:,2)'; % Speed [m/s] and friction coefficient
delta = 0.1; % Minimum distance among the speeds in the characteristic description
[P,newx,newy] = poly_trac(delta,v,mu);
velo = zeros(1,length(P));
velo(1:4:end) = newx;
dfl = [100 P;1 velo];

end

function [fcv,fcp,fcf,Pfc] = fc_inp(petr,bfl)

Pfc = zeros(1,1); % Initialization of the polinomial array.

nfile = [petr '/FricB/' bfl '.bfl'];
if ~exist(nfile,'file')
    error('The requested block brake friction coefficient model has not been provided yet');
end;
A = load(nfile);
sizA = size(A,1)-1;
vv = A(1,2:end)/3.6; % Speeds [m/s]
fcv = vv;
fcp = A(2:end,1);
fcf = A(2:end,2:end);
np = sizA;
% Fitting is made using a cubic spline interpolation: the array Pfc will be filled.
for ii = 1:np
    P = comp_poly_fc(fcv,fcf(ii,:));
    Pfc(1:length(P),ii) = P;
end;
end

function traccia = tracciato(ntrack,petr)

trackGUI = load([petr '/Track/' ntrack]);
% Info in trackGUI:
% Type	Lenght	Curvature radius	Slope (per mil)	Elevetion (cm)	length
% of previous parabolic curve
% Info in traccia:
% Length	Curvature radius	Slocpe	Elevetion	Vertical radious
% (computed)	Type
traccia = [trackGUI(1,[2 3 4 5]) 0 trackGUI(1,1)]; c = 1;
for ii = 2:size(trackGUI,1)
    if trackGUI(ii,6) > 0
        c = c+1;
        traccia(c,:) = [trackGUI(ii,6) 0 0 0 0 3];
        c = c+1;
        traccia(c,:) = [trackGUI(ii,[2 3 4 5]) 0 trackGUI(ii,1)];
    else
        c = c+1;
        traccia(c,:) = [trackGUI(ii,[2 3 4 5]) 0 trackGUI(ii,1)];
    end
end
end
