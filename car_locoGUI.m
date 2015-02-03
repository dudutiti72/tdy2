function loco = car_locoGUI(fsl,pathGUI,ploco,sep,train)

% This function reds the informations from train and creates a new struct array loco
% that will contain all the informations that deal with locos.
loco = struct('fv',zeros(2,1),'ft',zeros(2,1));

c = 0; % loco counter
% All vehicles are processed in order to update loco
for ii = 1:length(train)
    if strcmp(train(ii).type,'loco')% && not(isempty(train(ii).npetv))
        % Vehicle ii is a loco
        c = c+1;
        % Traction
        loco(c).eltcv = train(ii).eltcv;
        loco(c).eltts = 0; % It manages the delay of activation
        loco(c).eltct = train(ii).eltct;
        % Braking
        loco(c).elbcv = train(ii).elbcv;
        loco(c).elbts = 0; % It manages the delay of activation
        loco(c).elbct = train(ii).elbct;
        %loco(c).elsr = train(ii).elsr;
        loco(c).ploco = ii;
        % The input is in kN, but the N are used for the force
        loco(c).eltcv(:,2) = loco(c).eltcv(:,2)*1e3;
        loco(c).eltct(:,2) = loco(c).eltct(:,2)*1e3;
        loco(c).elbcv(:,2) = loco(c).elbcv(:,2)*1e3;
        loco(c).elbct(:,2) = loco(c).elbct(:,2)*1e3;
        % Gradient insertion and removal
        loco(c).tig = train(ii).tig; loco(c).trg = train(ii).trg;
    end
end
% The above characteristics are managed in order to compute the approximating
% polynomials: it is followed the same way of buffers and draw gears.
deltav = 0.1; % Minimum allowed speed distance among the points of the characteristic
deltat = 0.01; % Minimum allowed time distance among the points of the characteristic
% nx = 1000;
for ii = 1:c
    % TRACTION
    if sum(loco(ii).eltcv(:,1)) > 0
        [P,x,y] = poly_trac(deltav,loco(ii).eltcv(:,1)/3.6,loco(ii).eltcv(:,2));
        loco(ii).v = x; loco(ii).ftv = y;
        loco(ii).pfv = P; loco(ii).cv = 1;
    end
    if sum(loco(ii).eltct(:,1)) > 0
        [P,x,y] = poly_trac(deltat,loco(ii).eltct(:,1),loco(ii).eltct(:,2));
        loco(ii).t = x; loco(ii).ftt = y;
        loco(ii).pft = P; loco(ii).ct = 1;
    end
    
    % BRAKING
    if sum(loco(ii).elbcv(:,1)) > 0
        [P,x,y] = poly_trac(deltav,loco(ii).elbcv(:,1)/3.6,loco(ii).elbcv(:,2));
        loco(ii).vb = x; loco(ii).fbv = y;
        loco(ii).pfvb = P; loco(ii).cvb = 1;
    end
    if sum(loco(ii).elbct(:,1)) > 0
        [P,x,y] = poly_trac(deltav,loco(ii).elbct(:,1),loco(ii).elbct(:,2));
        loco(ii).tb = x; loco(ii).fbt = y;
        loco(ii).pftb = P; loco(ii).ctb = 1;
    end

end
lsep = numel(sep);
for ii = 1:c
    nfile = [pathGUI fsl 'Manoeuvre' fsl train(ploco(ii)).mano '.txt'];
    nf = fopen(nfile,'r');
    % mtype manages the type of control: 1...time, 2...position; 3...speed;
    % 4...relative pressure [bar] in BP; 5...relative pressure [bar] in BC
    strtofind = 'mtype='; dol = '$MAN_TYPE'; mtype = mat_var_dol(lsep,nf,sep,strtofind,dol);
    strtofind = 'ctrl='; ctrl = mat_var(lsep,nf,sep,strtofind);
    strtofind = 'pnBr='; pnBr = mat_var_boo(lsep,nf,sep,strtofind);
    strtofind = 'edBr='; edBr = mat_var_boo(lsep,nf,sep,strtofind);
    strtofind = 'epBr='; epBr = mat_var_boo(lsep,nf,sep,strtofind);
    strtofind = 'tract='; tract = mat_var_boo(lsep,nf,sep,strtofind);
    strtofind = 'pres='; pres = mat_var(lsep,nf,sep,strtofind);
    strtofind = 'vehicle='; vehicle = mat_var(lsep,nf,sep,strtofind); % Vehicle where the control of the pressure in BP or BC is applied
    strtofind = 'delay='; delay = mat_var(lsep,nf,sep,strtofind);
    strtofind = 'applic='; applic = mat_var(lsep,nf,sep,strtofind);
    fclose(nf);
    % Informations are stored in loco(ii).mano
    loco(ii).nm = length(mtype);
    for jj = 1:loco(ii).nm
        numline = -ones(1,10);
        numline(1) = pnBr(jj)+epBr(jj)*10+edBr(jj)*100+tract(jj)*1000;
        numline(2) = pres(jj);
        % Workaround
        if numline(1) == 0, numline(2) = -1; end
        if pnBr(jj) == 0, numline(2) = -1; end
        numline(2+mtype(jj)) = ctrl(jj);
        % If the manoeuvre is controlled through pressure in BP or BC, the vehicle
        % must be provided
        if mtype(jj) == 4 || mtype(jj) == 5, numline(8) = vehicle(jj); end
        numline(9:10) = [delay(jj) applic(jj)];
        loco(ii).man(jj,1:10) = numline; % Update of the loco field "man"
        if loco(ii).man(jj,5) ~= -1, loco(ii).man(jj,5) = loco(ii).man(jj,5)/3.6; end
        % The input uses the percentage of application force
        loco(ii).man(jj,10) = loco(ii).man(jj,10)*1e-2;
        if numline(1) == 0
            % The locomotive is running and doing "nothing"
            loco(ii).trac(jj) = 0; loco(ii).elettrodyn(jj) = 0; loco(ii).elettropn(jj) = 0; loco(ii).pn(jj) = 0;
        else
            % The identificative number is used in order to understand the type of the
            % sub-manoeuvres
            loco(ii).trac(jj) = floor(numline(1)/1000);
            loco(ii).elettrodyn(jj) = floor((numline(1)-loco(ii).trac(jj)*1000)/100);
            loco(ii).elettropn(jj) = floor((numline(1)-loco(ii).trac(jj)*1000-loco(ii).elettrodyn(jj)*100)/10);
            loco(ii).pn(jj) = (numline(1)-loco(ii).trac(jj)*1000-loco(ii).elettrodyn(jj)*100-loco(ii).elettropn(jj)*10);
        end
        if 0 
            % The previous zero has been imposed on 02/02/2012 in order to
            % give the possibility to model a traction / electrodynamic
            % brake and to stop it when a certain pressure in BP is
            % reached.
            if loco(ii).trac(jj) == 1, loco(ii).man(jj,6) = 0; end
            if loco(ii).elettrodyn(jj) == 1, loco(ii).man(jj,6) = 0; end
        end
        % Initialization
        loco(ii).ts = 0; % It is used to manage the force,time characteristic
        loco(ii).tg = 0; % It is used to manage the changing in percentage of application
    end
    % The manoeuvre are analyzed further. If there is a traction and then an
    % electrodynamic brake or viceversa a new manoeuvre is added.
    jj = 1;
    while jj < loco(ii).nm
        if ((loco(ii).trac(jj) == 1 && loco(ii).elettrodyn(jj+1) == 1) || ...
                (loco(ii).trac(jj+1) == 1 && loco(ii).elettrodyn(jj) == 1)) ...
                && loco(ii).man(jj,10) ~= 0 && (loco(ii).tig > 0 && loco(ii).trg > 0)
            % It is necessary to add another manoeuvre
            loco(ii).trac(jj+2:loco(ii).nm+1) = loco(ii).trac(jj+1:loco(ii).nm);
            loco(ii).elettrodyn(jj+2:loco(ii).nm+1) = loco(ii).elettrodyn(jj+1:loco(ii).nm);
            loco(ii).elettropn(jj+2:loco(ii).nm+1) = loco(ii).elettropn(jj+1:loco(ii).nm);
            loco(ii).pn(jj+2:loco(ii).nm+1) = loco(ii).pn(jj+1:loco(ii).nm);
            loco(ii).man(jj+2:loco(ii).nm+1,:) = loco(ii).man(jj+1:loco(ii).nm,:);
            if loco(ii).trac(jj) == 1
                % A traction until the application of a zero force is added and then
                % the elettrodynamic brake can occur
                loco(ii).trac(jj+1) = 1; loco(ii).elettrodyn(jj+1) = 0; 
            elseif loco(ii).elettrodyn(jj) == 1
                % An elettrodynamic brake is applied until a zero force and then the
                % traction can occur
                loco(ii).elettrodyn(jj+1) = 1; loco(ii).trac(jj+1) = 0;
            end
            loco(ii).man(jj+1,10) = 0; loco(ii).man(jj+1,1) = loco(ii).man(jj,1);
            loco(ii).man(jj+1,2:8) = -ones(1,7);
            jj = jj + 2; loco(ii).nm = loco(ii).nm + 1;
        else
            jj = jj + 1;
        end
    end
    
end

end

