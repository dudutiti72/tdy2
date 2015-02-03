function visres(nomesolu,appf)

appftxt = [appf,'.txt'];
appfmat = [appf,'.mat'];
load(['lastrun',appfmat]);
shw = 0;
Y = sol.y'; T = sol.x;
trfn = SR;
tspan = T(1):trfn:T(end);
Y = (interp1(T,Y,tspan));
T = tspan;
% Relative distance and speed
if N > 1
    Yr = diff(-Y(:,1:N),1,2); Ypr = diff(-Y(:,N+1:end),1,2);
else
    Yr = Y(:,1); Ypr = Y(:,N+1);
end
if shw == 1
    % Kinematic plots
    figure(1); clf;
    subplot(2,1,1); plot(T,1e3*Yr); grid on; xlabel('Time [s]'); ylabel('Rel displ [mm]');
    subplot(2,1,2); plot(T,Ypr); grid on; xlabel('Time [s]'); ylabel('Rel speed [m/s]');
    figure(2); clf;
    plot(T,3.6*Y(:,N+1:2*N))
    xlabel('Time [s]'); ylabel('Speed [km/h]'); grid on
end;
% Computation of the acceleration
Acc = zeros(numel(T),N);
Acc(2:end-1,:) = (Y(3:end,N+1:end)-Y(1:end-2,N+1:end))./...
    ((T(3:end)-T(1:end-2)).'*ones(1,N));
Acc(1,:) = (Y(2,N+1:end)-Y(1,N+1:end))./(T(2)-T(1));
Acc(end,:) = (Y(end,N+1:end)-Y(end-1,N+1:end))./(T(end)-T(end-1));

ilow = 2*onv; iup = onv; posfc = onv; ipos = onv; ptb = 1; 
Flong = zeros(N-1,length(T)); Flongd = Flong; Flongs = Flong; Flong10 = Flong;
Fbrake = zeros(length(T),N); matTrac = Fbrake; mfricoef = zeros(length(T),N);
vxr = Flong'; velo = zeros(1,length(T)); space = zeros(1,length(T));

for ii = 1:N
    bgdg(1,ii).c = 1; bgdg(2,ii).c = 1; bgdg(3,ii).c = 1;
end;
if isempty(swcf)
    % % The pneumatics is computed with the dynamics
    % MPneu = load('PneumaticData.txt');
    % BC = MPneu(:,2:N+1); BP = MPneu(:,N+2:2*N+1); UBP = MPneu(:,2*N+2:3*N+1);
    % MF = MPneu(:,3*N+2:end); TP = MPneu(:,1)';
    % The pneumatics is computed with the dynamics

    MPneu = load(['PneumaticData',appftxt]);
    BC = MPneu(:,2:N+1); BP = MPneu(:,N+2:2*N+1); UBP = MPneu(:,2*N+2:3*N+1); 
    MF = MPneu(:,3*N+2:end); TP = MPneu(:,1)';
    % Sometimes TP has some repetitions; following rows fix it
    appo = diff(TP); 
    appo2 = find(appo==0);
    if not(isempty(appo2))
        increments = 1:length(TP);
        increments(appo2 + 1) = []; 
        TP = TP(increments); BC = BC(increments,:); BP = BP(increments,:); UBP = UBP(increments,:); MF = MF(increments,:);
    end
    if length(T) ~= length(TP)
       BC = interp1(TP,BC,T); BP = interp1(TP,BP,T); UBP = interp1(TP,UBP,T); MF = interp1(TP,MF,T); TP = T;
       BC(BC<0) = 0; BP(BP<0) = 0; % Only positive values are meaningful
    end
    
else
    %nfile = strrep(nomesolu,'flong.mat','DataSA.txt');
    %load(nfile);
    BC = []; BP = []; UBP = []; MF = []; TP = [];
end

% Initialization
cp = 1; jm = 1; znl = zeros(size(ploco'));
% Wrapping locos time indicies
for ii = 1:length(loco), loco(ii).ct = 1; end

for jj = 1:length(T)
    Feldyn = znl; % Updating
    Ftrac = znl; % Updating
    Ffren = znv;
    while isempty(Mano(jm).tspan)
        jm = jm+1;
    end
    if T(jj) > Mano(jm).tspan(end), jm = jm+1; end
    while isempty(Mano(jm).tspan)
        jm = jm+1;
    end
    eldyn = Mano(jm).eldyn; trac = Mano(jm).trac; jmloc = Mano(jm).jmloc;
    for ii = [trac eldyn]
        loco(ii).ts = Mano(jm).ts(ii);
        loco(ii).Ftrac = Mano(jm).Ftrac(ii);
        loco(ii).Feldyn = Mano(jm).Feldyn(ii);
        loco(ii).tg = Mano(jm).tg(ii);
    end
    
    speed_vehicle=vel_0(2);
    y = Y(jj,:); velo(jj)=Y(jj,N+speed_vehicle); space(jj)=Y(jj,1);
    
    if isempty(swcf) 
        if jj > length(TP) 
            % It is necessary to add data
            TP(end+1) = T(jj);
            % Linear approximation
            deltat = TP(end)-TP(end-1); incre = deltat/(TP(end-1)-TP(end-2));
            BC(end+1,:) = BC(end-1,:) + (BC(end,:)-BC(end-1,:))*incre;
            BP(end+1,:) = BP(end-1,:) + (BP(end,:)-BP(end-1,:))*incre;
            UBP(end+1,:) = UBP(end-1,:) + (UBP(end,:)-UBP(end-1,:))*incre;
            MF(end+1,:) = MF(end-1,:) + (MF(end,:)-MF(end-1,:))*incre;
        elseif abs(T(jj) - TP(jj)) > 0.1*SR && jj > 1
            % A resampling is needed
            i1 = jj+1:length(TP)+1; i0 = jj:length(TP);
            TP(i1) = TP(i0); BC(i1,:) = BC(i0,:); BP(i1,:) = BP(i0,:); UBP(i1,:) = UBP(i0,:); MF(i1,:) = MF(i0,:);
            TP(jj) = TP(jj-1)+SR;
            BC(jj,:) = BC(jj-1,:) + (BC(jj+1,:)-BC(jj-1,:))/(TP(jj+1)-TP(jj-1))*SR;
            BP(jj,:) = BP(jj-1,:) + (BP(jj+1,:)-BP(jj-1,:))/(TP(jj+1)-TP(jj-1))*SR;
            UBP(jj,:) = UBP(jj-1,:) + (UBP(jj+1,:)-UBP(jj-1,:))/(TP(jj+1)-TP(jj-1))*SR;
            MF(jj,:) = MF(jj-1,:) + (MF(jj+1,:)-MF(jj-1,:))/(TP(jj+1)-TP(jj-1))*SR;
        end
    end
    
    if N > 1
        xr = -diff(Y(jj,1:N));
        vr = -diff(Y(jj,N+1:2*N));
        [bgdg,CCtrac,Fel,Feld,Fels,fitrac,ipos] = fbgdg(bgdg,CCtrac,Fel,Feld,Fels,...
            fitrac,ipos,lwagcs,N,strack,traccia,vr,y,xr);
        Flong(:,jj) = Fel; Flongd(:,jj) = Feld; Flongs(:,jj) = Fels; vxr(jj,:) = xr;
    end
    if isempty(swcf)
        if abs(T(jj) - MPneu(cp,1)) < 1e-6 && cp < size(MPneu,1)
            pbrake_0 = MPneu(cp,2:N+1);
            cp = cp+1;
        else
            while T(jj) > MPneu(cp,1) && cp < size(MPneu,1)
                cp = cp+1;
            end
            pbrake_0 = MPneu(cp-1,2:end) + (MPneu(cp,2:end)-MPneu(cp-1,2:end))/(MPneu(cp,1)-MPneu(cp-1,1))*(T(jj)-MPneu(cp-1,1));
        end
        [Ffren,frico,ilow,iup,posfc,train] = brakeforce(Ffren,frico,ilow,iup,indexes,N,pbrake_0,posfc,train,y,vel_0);
    else
        t = T(jj);
        while t > Tbrake(ptb+1,1)
            ptb = ptb+1;
        end;
        tb = t - Tbrake(ptb,1);
        if not(isempty(Ppres))
            pbrake_0 = Ppres(2+(ptb-1)*4,:)*tb^3+Ppres(3+(ptb-1)*4,:)*tb^2+...
                Ppres(4+(ptb-1)*4,:)*tb+Ppres(5+(ptb-1)*4,:);
            [Ffren,frico,ilow,iup,posfc,train] = brakeforce(Ffren,frico,ilow,iup,indexes,N,pbrake_0,posfc,train,y,vel_0);
        else
            Ffren = zeros(1,N);
        end;
    end
    if any(eldyn)
        % Electrodynamic braking is active
        [Feldyn,loco] = brake_eldyn(eldyn,Feldyn,loco,N,ploco,T(jj),jmloc,y);
    end
    if any(trac)
        % Traction is active
        [Ftrac,loco] = trac_eldyn(trac,Ftrac,loco,N,ploco,T(jj),jmloc,y);
    end
    
    Fbrake(jj,:) = Ffren; 
%     mfricoef(jj,1:2:end) = frico(1,:); mfricoef(jj,2:2:end) = frico(2,:);
    frico_appo = frico(1,:)+frico(2,:);
    mfricoef(jj,:) = frico_appo;
    Fbrake(jj,ploco) = Fbrake(jj,ploco)+Feldyn;
    matTrac(jj,ploco) = Ftrac-Feldyn;
    
end;
% mfricoef = mfricoef';
velo_all = Y(:,N+1:end); % Speed of all vehicles
if N > 1
    xm = 10; Flong10 = compFlongxm(Flong,space,T,velo,xm);
    xm = 2; Flong2 = compFlongxm(Flong,space,T,velo,xm);
else
    Flong10 = [];
    Flong2 = [];
end
[Eb,~] = integral_comp(Fbrake,Y(:,1:N));

stopdist = max(max(Y));
save(nomesolu,'T','Y','Flong','Flong2','Flong10','Mvt','stopdist','Fbrake','train','loco','bgdg',...
    'velo','velo_all','space','TP','BC','BP','UBP','MF','Yr','Ypr','matTrac','Acc','mfricoef','lwagcs','Eb');
if shw == 1 && N > 1
    figure(3); clf;
    subplot(3,1,1); plot(T,Flong,'linewidth',2); grid on; title('Flong complessive');
    flylim = get(gca,'YLim');
    subplot(3,1,2); plot(T,Flongd,'linewidth',2); grid on; title('Flong compr. destra');
    set(gca,'YLim',flylim);
    subplot(3,1,3); plot(T,Flongs,'linewidth',2); grid on; title('Flong compr. sinistra');
    set(gca,'YLim',flylim);
end;

end


function [I,IS] = integral_comp(Y,X)
% This function computes the integral according to the Cavalieri Simpson algorithm
% Check if the function applies to matricies or to vectors
sizY = size(Y);
if min(sizY) > 1
    % Input are matricies
    % The dimensions of Y are arranged so that each column refers to a vehicle
    if sizY(1) < sizY(2), Y = Y.'; sizY = size(Y); end
    % Adjusting the matricies dimensions
    sizX = size(X);
    [m,p] = min(sizX);
    if m == 1
        % The vector X becomes a matrix and it is assumed that the number of vehicle
        % is lower than the analysis time steps.
        if p == 1, 
            % X is a row vector
            X = ones(min(sizY),1) * X;
        elseif p == 2
            % X is column vector
            X = X * ones(1,min(sizY));
        end
    end
    % X and Y becomes of the same dimension
    if sizY(1) ~= sizX(1) && sizY(1) == sizX(2), X = X.'; end
else
    % Y is a vector
    sizX = siz(X);
    if min(sizX) > 1, error('It''s not possible to compute the integration'); end
    if sizY(1) ~= sizX(1) && sizY(1) == sizX(2), X = X.'; 
    else
        error('Vector dimensions do not match');
    end
end
% Initialization
I = zeros(size(X)); IS = I;
N = sizY(2); % Number of vehicles
S = sizY(1); % Number of integration position
ind = 1:2:S-2; 
for ii = 1:N
    dx1 = X(ind+1,ii) - X(ind,ii); dx2 = X(ind+2,ii) - X(ind,ii); % Abscissa increments
    % Parabola coefficients
    b = ( (Y(ind+2,ii)-Y(ind,ii)).*dx1.^2 - (Y(ind+1,ii)-Y(ind,ii)).*dx2.^2 ) ./ ...
        (dx2.*dx1.^2-dx1.*dx2.^2);
    a = (( Y(ind+1,ii)-Y(ind,ii) ) - b.*dx1)./(dx1.^2);
    IS(ind+2,ii) = (a.*dx2.^3)/3 + 0.5*b.*dx2.^2 + Y(ind,ii).*dx2; % Integral computation
    IS(:,ii) = cumsum(IS(:,ii)); % Cumulative sum
    IS(ind+1,ii) = 0.5 * (IS(ind,ii) + IS(ind+2,ii)); % Smooting the results
    % Computation via trapezoid rule
    I(:,ii) = cumtrapz(X(:,ii),Y(:,ii));
end

end