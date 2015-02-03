function loco = car_loco(namedirfile,train)

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
        loco(c).eltts = train(ii).eltts;
        loco(c).eltct = train(ii).eltct;
        % Braking
        loco(c).elbcv = train(ii).elbcv;
        loco(c).elbts = train(ii).elbts;
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
    if size(loco(ii).eltcv,1) > 1
        [P,x,y] = poly_trac(deltav,loco(ii).eltcv(:,1)/3.6,loco(ii).eltcv(:,2));
        loco(ii).v = x; loco(ii).ftv = y;
        loco(ii).pfv = P; loco(ii).cv = 1;
    end
    % figure(ii);
    % x = linspace(0,loco(ii).v(end),nx);
    % Fv = zeros(size(x)); pos = 1;
    % for jj = 1:nx
    %     [F,pos] = interpbgdg(loco(ii).pfv,pos,loco(ii).v,x(jj));
    %     Fv(jj) = F;
    % end
    % plot(loco(ii).v,loco(ii).ftv,x,Fv);
    % TODO: Add the possibility to use a curve and not simply a slope to describe the
    % un-steady behaviour of the locos
    
    if size(loco(ii).eltct,1) > 1
        [P,x,y] = poly_trac(deltat,loco(ii).eltct(:,1),loco(ii).eltct(:,2));
        loco(ii).t = x; loco(ii).ftt = y;
        loco(ii).pft = P; loco(ii).ct = 1;
    end
    
%     m = loco(ii).elsr*1e3; % The slope is in [kN/s]
%     q = 0;
%     loco(ii).t = [0 1e4]; loco(ii).ft = [0 1e4*m]; 
%     loco(ii).pft = [1 0 0 m q]; loco(ii).ct = 1;


    % BRAKING
    if size(loco(ii).elbcv,1) > 1
        [P,x,y] = poly_trac(deltav,loco(ii).elbcv(:,1)/3.6,loco(ii).elbcv(:,2));
        loco(ii).vb = x; loco(ii).fbv = y;
        loco(ii).pfvb = P; loco(ii).cvb = 1;
    end
    if size(loco(ii).elbct,1) > 1
        [P,x,y] = poly_trac(deltav,loco(ii).elbct(:,1),loco(ii).elbct(:,2));
        loco(ii).tb = x; loco(ii).fbt = y;
        loco(ii).pftb = P; loco(ii).ctb = 1;
    end

end
nfile = [namedirfile 'ManoeuvreLoco.txt'];
nf = fopen(nfile,'r');
numline = gotonumeric(nf);
%cline = str2num(cline);
for ii = 1:c
    loco(ii).nm = numline(ii);
end
for ii = 1:c
    for jj = 1:loco(ii).nm
        numline = gotonumeric(nf);
        if numline(1) == 0, numline(2) = -1; end
        loco(ii).man(jj,1:5) = numline(1:5);
        loco(ii).man(jj,6:8) = -1;
        loco(ii).man(jj,9:10) = numline(6:7);
        
        if loco(ii).man(jj,5) ~= -1, loco(ii).man(jj,5) = loco(ii).man(jj,5)/3.6; end
        % The input uses the percentage of application force
        loco(ii).man(jj,10) = loco(ii).man(jj,10)*1e-2;
        if numline(1) == 0
            loco(ii).trac(jj) = 0; loco(ii).elettrodyn(jj) = 0; loco(ii).elettropn(jj) = 0; loco(ii).pn(jj) = 0;
        else
            % The identificative number is used in order to understand the type of the
            % sub-manoeuvres
            loco(ii).trac(jj) = floor(numline(1)/1000);
            loco(ii).elettrodyn(jj) = floor((numline(1)-loco(ii).trac(jj)*1000)/100);
            loco(ii).elettropn(jj) = floor((numline(1)-loco(ii).trac(jj)*1000-loco(ii).elettrodyn(jj)*100)/10);
            loco(ii).pn(jj) = (numline(1)-loco(ii).trac(jj)*1000-loco(ii).elettrodyn(jj)*100-loco(ii).elettropn(jj)*10);
        end
        % Initialization
        loco(ii).ts = 0; % It is used to manage the force,time characteristic
        loco(ii).tg = 0; % It is used to manage the changing in percentage of application
    end
end
fclose(nf);


end

function numline = gotonumeric(nf)

while 1
    cline = fgetl(nf);
    if ~ischar(cline)
        break; 
    end
    numline = str2num(cline);
    if not(isempty(numline)), break; end;
end

end