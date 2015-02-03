function [bgdg,train] = prel_comp(nveicoli,train)

% Computing the number of points used in order to discretize the equivalent force
% stroke characteristic of two consecutives buffing gears or draw gear.
npxbgdg = 3;
for ii = 1:nveicoli;
    npxbgdg = max([npxbgdg size(train(ii).bgsf,1) size(train(ii).bgsr,1) size(train(ii).dgsf,1) size(train(ii).dgsr,1)]);
end;
npxbgdg = 2*npxbgdg;

% Initializations
zbgdg = zeros(1,npxbgdg);
% bgdg collects informations about force-displacement coupling among buffers (right, first
% row), gears (second row) and left buffers (third row), for each coupling
bgdg = struct('x',zbgdg,'fl',zbgdg,'fu',zbgdg,'pl',zbgdg,'pu',zbgdg,'c',1);

% Piecewise approximation of the force-stroke characteristics of buffing gears and
% draw gears.
train = piece_approx_bg_dg(nveicoli,train);

% Coupling informations
for ii = 1:nveicoli-1
    % Initialization
    for hh = 1:2
        if hh == 1 && train(ii).bgccr == train(ii+1).bgccf
            % Buffing gears
            s1 = train(ii).bgsr; s2 = train(ii+1).bgsf;
            Pr = train(ii).Pbgrl; Pf = train(ii+1).Pbgfl;
            [F,vx] = equiv_bgdg(npxbgdg,Pf,Pr,s1,s2);
            bgdg(hh,ii).xl = vx(3,:); bgdg(hh,ii).fl = F;
            
            Pr = train(ii).Pbgru; Pf = train(ii+1).Pbgfu;
            [F,vx] = equiv_bgdg(npxbgdg,Pf,Pr,s1,s2);
            bgdg(hh,ii).xu = vx(3,:); bgdg(hh,ii).fu = F;
            if size(bgdg(hh,ii).xl,2) ~= size(bgdg(hh,ii).xu,2)
                xuldiff = ['Loading and unloading curve of coupling ' num2str(ii) 'have different points'];
                disp(xuldiff);
            end
            bgdg(hh,ii).CentrC = train(ii+1).bgccf;
        elseif hh == 2 && train(ii).dgccr == train(ii+1).dgccf
            % Draw gears
            s1 = train(ii).dgsr; s2 = train(ii+1).dgsf;
            Pr = train(ii).Pdgrl; Pf = train(ii+1).Pdgfl;
            [F,vx] = equiv_bgdg(npxbgdg,Pf,Pr,s1,s2);
            bgdg(hh,ii).xl = vx(3,:); bgdg(hh,ii).fl = F;
            
            Pr = train(ii).Pdgru; Pf = train(ii+1).Pdgfu;
            [F,vx] = equiv_bgdg(npxbgdg,Pf,Pr,s1,s2);
            bgdg(hh,ii).xu = vx(3,:); bgdg(hh,ii).fu = F;
            if size(bgdg(hh,ii).xl,2) ~= size(bgdg(hh,ii).xu,2)
                xuldiff = ['Loading and unloading curve of coupling ' num2str(ii) 'have different points'];
                disp(xuldiff);
            end
            bgdg(hh,ii).CentrC = train(ii+1).dgccf;
        else
            % It gives an error if a central coupler is coupled with a traditional coupling.
            if hh == 1, type = 'buffers'; elseif hh == 2, type = 'draw gears'; end;
            msg = ['In the ' num2str(ii) 'th coupling, the ' type ' do not match'];
            error(msg);
        end;
        bgdg = viscous_damping(bgdg,hh,ii,train);
    end;
end;

% Piecewise polynomial approximation of COUPLED buffing gears and draw gears
for ii = 1:nveicoli-1
    % It is necessary that the loading and un-loading curve have the same abscissa
    bgdg = sameabscissa(bgdg,ii,1,train);
    
    P = comp_poly_bgdgf(bgdg(1,ii).xl,bgdg(1,ii).fl,'c',train(ii).op);
    bgdg(1,ii).pl = P; bgdg(1,ii).c = 1;
    
    P = comp_poly_bgdgf(bgdg(1,ii).xu,bgdg(1,ii).fu,'c',train(ii).op);
    bgdg(1,ii).pu = P;
    
    
    % It is necessary that the loading and un-loading curve have the same abscissa
    bgdg = sameabscissa(bgdg,ii,2,train);
    
    P = comp_poly_bgdgf(bgdg(2,ii).xl,bgdg(2,ii).fl,'c',train(ii).op);
    bgdg(2,ii).pl = P; bgdg(2,ii).c = 1;
    
    P = comp_poly_bgdgf(bgdg(2,ii).xu,bgdg(2,ii).fu,'c',train(ii).op);
    bgdg(2,ii).pu = P;
    
    bgdg(1,ii).vpu = 0.5*(train(ii).bgvpur+train(ii+1).bgvpuf);
    bgdg(1,ii).vpl = 0.5*(train(ii).bgvplr+train(ii+1).bgvplf);
    bgdg(2,ii).vpu = 0.5*(train(ii).dgvpur+train(ii+1).dgvpuf);
    bgdg(2,ii).vpl = 0.5*(train(ii).dgvplr+train(ii+1).dgvplf);
    % Polynomy used to manage load and unload
    vpl = -bgdg(1,ii).vpl; vpu = bgdg(1,ii).vpu;
    A = [vpl^3 vpl^2 vpl 1;3*vpl^2 2*vpl 1 0;...
        vpu^3 vpu^2 vpu 1;3*vpu^2 2*vpu 1 0];
    b = [0;0;1;0];
    bgdg(1,ii).Plu = A\b;
    vpl = bgdg(2,ii).vpl; vpu = -bgdg(2,ii).vpu;
    A = [vpl^3 vpl^2 vpl 1;3*vpl^2 2*vpl 1 0;...
        vpu^3 vpu^2 vpu 1;3*vpu^2 2*vpu 1 0];
    bgdg(2,ii).Plu = A\b;
    
    if train(ii).gap >= 0
        % If the gap is positive the draw gear do not have any gap and it is assigned
        % to the buffers to compiute correctly their compression
        bgdg(1,ii).gap = train(ii).gap*1e-3;
        bgdg(2,ii).gap = 0; 
    else
        % If the gap is negative there is a preload: the input displacement (deltaG),
        % influences both buffers and draw gear
        deltaG = -train(ii).gap*1e-3;
        % It computes the equivalent XR and XT;
        [XR, XT] = comp_preload(bgdg(1,ii).xl,bgdg(2,ii).xl,bgdg(1,ii).pl,...
            bgdg(1,ii).pu,bgdg(2,ii).pl,bgdg(2,ii).pu,deltaG,...
            bgdg(1,ii).Plu,bgdg(2,ii).Plu);
        
        bgdg(1,ii).gap = -XR; % buffers are compressed
        bgdg(2,ii).gap = XT; % draw gear are pulled
        
    end
    
end;
% Copy of the information of the buffing gear: it will represent the left buffing
% gear behaviour.
bgdg(3,:) = bgdg(1,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [F,vx] = equiv_bgdg(npxbgdg,Pf,Pr,s1,s2)

% This function computes the equivalent force stroke characteristic of buffers and
% draw gears
% From the polynomials and the strokes, the forces are computed
f1 = comp_force(s1,Pr);
f2 = comp_force(s2,Pf);

% Coupling forces
vx = zeros(3,npxbgdg);
fmin = [f1(1) f2(1)];
if npxbgdg > 7
    minf1f2 = min([f1(1) f2(1)]);
    if minf1f2 > 0
        F = logspace(log10(minf1f2),log10(min([f1(end) f2(end)])),npxbgdg);
    else
        %maxf1f2 = max([f1(2) f2(2)]);
        if sum([f1(1) f2(1)]) == 0
            F = logspace(log10(0.1*min([f1(2) f2(2)])),log10(min([f1(end) f2(end)])),npxbgdg);
        else
            F = logspace(log10(500),log10(min([f1(end) f2(end)])),npxbgdg);
        end;
    end;
else
    F = logspace(log10(max([f1(1) f2(1) 1])),log10(min([f1(end) f2(end)])),npxbgdg);
    if F(1) == 1, F(1) = 0; end
end;
pos = [1 1];
for jj = 2:npxbgdg
    x = 0;
    for ij = 1:2
        if ij == 1, P = Pr; f = s1; elseif ij == 2, P = Pf; f = s2; end;
        if ij == 1
            while F(jj) > f1(pos(ij)+1) + 1e-6 % 1e-6 manages rounding errors
                pos(ij) = pos(ij)+1;
            end;
        elseif ij == 2
            while F(jj) > f2(pos(ij)+1)  + 1e-6 % 1e-6 manages rounding errors
                pos(ij) = pos(ij)+1;
            end;
        end;
        if P(1) == 1
            s = roots([P(2),P(3)-F(jj)]);
            sok = findsok(s,ij,vx(ij,jj-1),f,pos);
            % TODO: Check if the following row is necessary
            x = sok + x;
        else
            s = roots([P(2+(pos(ij)-1)*4:pos(ij)*4),P(pos(ij)*4+1)-F(jj)]);
            sok = findsok(s,ij,vx(ij,jj-1),f,pos);
            % TODO: Check if the following row is necessary
            x = sok + x;
        end;
        if F(jj) > fmin(ij)
            vx(ij,jj) = sok;
        else
            vx(ij,jj) = 0;
        end;
    end;
    vx(3,jj) = sum(vx(1:2,jj));
end;

function sok = findsok(s,ij,spoint,f,pos)

% spoint is the starting point for the solution
kk = 1;
while abs(imag(s(kk))) > 1e-9
    kk = kk+1;
end;
sok = s(kk);
for k = kk:length(s)
    if abs(imag(s(k))) < 1e-9 && real(s(k)) > 0 && ...
            (s(k) >= f(pos(ij)) && s(k)-1e-12 <= f(pos(ij)+1)) % 1e-12 menages rounding errors
        if abs(s(k)-spoint) < abs(sok-spoint)
            sok = s(k);
        end;
    end;
end;

function train = end_of_stroke(nveicoli,train)

for ii = 1:nveicoli
    % Buffing gear front
    [s,f] = modif_fs(train(ii).bgsf,train(ii).bgff);
    train(ii).bgsf = s; train(ii).bgff = f;
    
    % Buffing gear rear
    [s,f] = modif_fs(train(ii).bgsr,train(ii).bgfr);
    train(ii).bgsr = s; train(ii).bgfr = f;
    
    % Draw gear front
    [s,f] = modif_fs(train(ii).dgsf,train(ii).dgff);
    train(ii).dgsf = s; train(ii).dgff = f;
    
    % Draw gear rear
    [s,f] = modif_fs(train(ii).dgsr,train(ii).dgfr);
    train(ii).dgsr = s; train(ii).dgfr = f;
end;

function [s,f] = modif_fs(s,f)

ss = s; np = size(f,1);
if size(f,2) == 2 && f(end,2) ~= 0
    ncol = 2;
else
    ncol = 1;
end;
d = zeros(1,ncol);
for icol = 1:ncol
    d(icol) = 2*(f(np,icol) - f(np-1,icol)) / (ss(end) - ss(end-1));
    if icol == 1
        s(np+1) = 2*s(end);
    else
        d(icol) = max([d(icol) 1.1*d(1)]);
    end
    f(np+1,icol) = f(np,icol) + d(icol)*(s(end) - s(end-1));
end;


function train = piece_approx_bg_dg(nveicoli,train)

% Piecewise polynomial approximation of buffing gears and draw gears
for ii = 1:nveicoli
    train(ii).bgff = train(ii).bgff*1e3; train(ii).bgsf = train(ii).bgsf*1e-3;
    train(ii).bgfr = train(ii).bgfr*1e3; train(ii).bgsr = train(ii).bgsr*1e-3;
    train(ii).dgff = train(ii).dgff*1e3; train(ii).dgsf = train(ii).dgsf*1e-3;
    train(ii).dgfr = train(ii).dgfr*1e3; train(ii).dgsr = train(ii).dgsr*1e-3;
    if train(ii).bgdf == 0
        s = train(ii).bgsf; fu = train(ii).bgff(:,1); fl = train(ii).bgff(:,2);
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    else
        s = train(ii).bgsf; fl = train(ii).bgff(:,1); fu = (1-train(ii).bgdf)*fl;
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    end
    train(ii).Pbgfl = Pl;
    train(ii).Pbgfu = Pu;
    
    if train(ii).bgdr == 0
        s = train(ii).bgsr; fu = train(ii).bgfr(:,1); fl = train(ii).bgfr(:,2);
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    else
        s = train(ii).bgsr; fl = train(ii).bgfr(:,1); fu = (1-train(ii).bgdr)*fl;
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    end
    train(ii).Pbgrl = Pl;
    train(ii).Pbgru = Pu;
    
    
    if train(ii).dgdf == 0
        s = train(ii).dgsf; fu = train(ii).dgff(:,1); fl = train(ii).dgff(:,2);
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    else
        s = train(ii).dgsf; fl = train(ii).dgff(:,1); fu = (1-train(ii).dgdf)*fl;
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    end
    train(ii).Pdgfl = Pl;
    train(ii).Pdgfu = Pu;
    
    if train(ii).dgdr == 0
        s = train(ii).dgsr; fu = train(ii).dgfr(:,1); fl = train(ii).dgfr(:,2);
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    else
        s = train(ii).dgsr; fl = train(ii).dgfr(:,1); fu = (1-train(ii).dgdr)*fl;
        Pu = comp_poly_bgdgf(s,fu,'s',1);
        Pl = comp_poly_bgdgf(s,fl,'s',1);
    end
    train(ii).Pdgrl = Pl;
    train(ii).Pdgru = Pu;
    train(ii).dampdg = [train(ii).dgdr train(ii).dgdf];
    train(ii).dampbg = [train(ii).bgdr train(ii).bgdf];
end;

function f = comp_force(s,P)
% This function computes the forces corrisponding to the given strokes using the
% piecewise polynomial approximation
% NOTE THAT THIS FUNCTION ASSUMES THAT THE STROKE s CONTAINS THE POINTS USED TO
% COMPUTE THE VECTOR OF POLYNOMIALS P
%c = 1; % Current polynomial used
f = zeros(size(s));
for ii = 1:length(s)-1
    if P(1) == 1
        PP = [0 0 P(2:3)];
    else
        PP = P(1+4*(ii-1)+1:1+4*ii); % Actual piece of polynomial
    end
    f(ii) = PP(1)*s(ii)^3+PP(2)*s(ii)^2+PP(3)*s(ii)+PP(4);
end
f(end) = PP(1)*s(end)^3+PP(2)*s(end)^2+PP(3)*s(end)+PP(4);


function [newx,newy] = checkxyslope(x,y)

if max(x) > 1, delta = 1; else delta = 1e-3; end;
% delta = min([delta;diff(x)]);
mol = 1.5;
nx = size(x,1); % Number of points of the characteristic
if nx >= 3
    %if size(y,2) == 2, x = [x x]; end;
    if (size(y,2) > 1 && y(end,2) > 0), x = [x x]; end;
    xx = x; yy = y;
    for ii = 1:size(x,2) %size(y,2)
        nx = nnz(xx(2:end,ii)) + 1;
        x = xx(1:nx,ii); y = yy(1:nx,ii);
        slopeok = 0;
        while slopeok == 0
            nx = size(x,1); % Number of points of the characteristic
            jj = 1;
            while jj < nx-1
                dy = y(jj+1)-y(jj); dx = x(jj+1)-x(jj);
                s1 = dy/dx;
                dy = y(jj+2)-y(jj+1); dx = x(jj+2)-x(jj+1);
                s2 = dy/dx;
                % Note 1.01 are added in order to avoid numerical rounding errors
                if abs(s2) > 1.01*abs(s1)*mol && dx > 1.01*delta
                    x = [x(1:jj+1);x(jj+1)+delta;x(jj+2:end)];
                    nx = size(x,1);
                    %ystraight = y(jj+1) + sign(s2)*s1*mol*delta;
                    nstep = floor(dx/delta);
                    ystraight = y(jj+1) + sign(s2)*s1*max([mol (s2/s1)^(1/nstep)])*delta;
                    y = [y(1:jj+1);ystraight;y(jj+2:end)];
                    jj = nx;
                elseif abs(s2)*1.01 < abs(s1)/mol && dx > 1.01*delta && x(jj+1) > 2e-3
                    x = [x(1:jj+1);x(jj+1)+delta;x(jj+2:end)];
                    nx = size(x,1);
                    
                    %ystraight = y(jj+1) + sign(s2)*s1/mol*delta;
                    nstep = floor(dx/delta);
                    ystraight = y(jj+1) + sign(s2)*s1/max([mol (s1/s2)^(1/nstep)])*delta;
                    if ystraight > 0.9*y(jj+2)
                        ystraight = y(jj+1) + sign(s2)*s2*delta;
                    end
                    y = [y(1:jj+1);ystraight;y(jj+2:end)];
                    jj = nx;
                end
                jj = jj+1;
            end
            if jj < nx, slopeok = 1; end;
        end
        xx(1:nx,ii) = x; yy(1:nx,ii) = y;
    end
    if size(yy,2) > 1 && sum(yy(:,2)) > 0
        nzx1 = nnz(xx(:,1)); nzx2 = nnz(xx(:,2));
        if nzx1 < nzx2
            y1 = spline(xx(1:nzx1+1,1),yy(1:nzx1+1,1),xx(:,2));
            x = xx(:,2); y = [y1 yy(:,2)];
        elseif nzx2 < nzx1
            y2 = spline(xx(1:nzx2+1,2),yy(1:nzx2+1,2),xx(:,1));
            x = xx(:,1); y = [yy(:,1) y2];
        elseif sum(xx(:,2)-xx(:,1)) > 1
            % Forces are computed in different points: it is necessary to create an
            % unique x-vector
            x = [xx(:,1);xx(:,2)];
            x = sort(x);
            d = diff(x);
            x(d == 0) = [];
            y1 = spline(xx(:,1),yy(:,1),x); y2 = spline(xx(:,2),yy(:,2),x);
            y = [y1 y2];
        else
            % Everithing is OK
            y = yy;
        end
        % Checking that the loading curve is always above the unloading curve
        yu = min([y';0.99*y(:,2)']);
        y(:,1) = yu';
    elseif size(yy,2) == 2
        % Damping is specified
        y = [y zeros(size(y))];
    end;
    
    %newx = x;
end

newx = x; newy = y;

function bgdg = sameabscissa(bgdg,ii,ind,train)


% It is necessary that the loading and un-loading curve have the same abscissa
% Moreover it is necessary that the loading curve is above the un-loading curve.
% Lastly, it is necessary to extend the force-stroke curve in order to avoid
% problems during the integration.
% Amplitude of transition zone
% TODO: It can be put in input as advanced parameter.
tz = 2e-3;
pacco = bgdg(ind,ii).xl; stiff = bgdg(ind,ii).fl;
P = comp_poly_bgdgf(pacco,stiff,'s',1);
pos = 1; F = interpbgdgf(P,pos,pacco,tz);
itz = pacco > tz;
pacco = [0 tz pacco(itz)]; stiff = [0 F stiff(itz)];
while min(diff(pacco)) < 1e-3
    [a,b] = min(diff(pacco));
    pacco(b+1) = []; stiff(b+1) = [];
end
[xl,fl] = checkxyslope(pacco',stiff');
% Extension to manage end of stroke issues
[xl,fl] = modif_fs(xl,fl);
pacco = xl; stiff = fl;
pacco = pacco'; stiff = stiff';
bgdg(ind,ii).xl = pacco; bgdg(ind,ii).fl = stiff;

% if bgdg(ind,ii).damp == 0
%     bgdg(ind,ii).fu = stiff;
%     bgdg(ind,ii).xu = pacco;
% else
pacco = bgdg(ind,ii).xu; stiff = bgdg(ind,ii).fu;
P = comp_poly_bgdgf(pacco,stiff,'s',1);
pos = 1; F = interpbgdgf(P,pos,pacco,tz);
itz = pacco > tz;
pacco = [0 tz pacco(itz)]; stiff = [0 F stiff(itz)];
while min(diff(pacco)) < 1e-3
    [a,b] = min(diff(pacco));
    pacco(b+1) = []; stiff(b+1) = [];
end
[xu,fu] = checkxyslope(pacco',stiff');
% Extension to manage end of stroke issues
[xu,fu] = modif_fs(xu,fu);
% Loading curve must be above un-loading curve.
chkslope = 0;
while chkslope == 0
    [xu2,fu2] = checkload_unload(xl,fl,xu,fu);
    if max(abs(fu-fu2)) < 1e-6
        chkslope = 1;
    else
        [xu,fu] = checkxyslope(xu2,fu2);
    end
end
pacco = xu; stiff = fu;
pacco = pacco'; stiff = stiff';
bgdg(ind,ii).xu = pacco; bgdg(ind,ii).fu = stiff;

% it is necessary to create an unique x-vector
x = [bgdg(ind,ii).xu bgdg(ind,ii).xl];
x = sort(x);
d = diff(x);
x(abs(d) < 0.5e-3) = [];
P = comp_poly_bgdgf(xu,fu,'c',train(ii).op);
fu = zeros(size(x)); pos = 1;
for jj = 1:length(x)
    [fu(jj),pos] = interpbgdgf(P,pos,xu,x(jj));
end
P = comp_poly_bgdgf(xl,fl,'c',train(ii).op);
fl = zeros(size(x)); pos = 1;
for jj = 1:length(x)
    [fl(jj),pos] = interpbgdgf(P,pos,xl,x(jj));
end
bgdg(ind,ii).fu = fu; bgdg(ind,ii).fl = fl;
bgdg(ind,ii).xu = x; bgdg(ind,ii).xl = x;
% end

function [xu,fu] = checkload_unload(xl,fl,xu,fu)

% This function changes, if it is necessary, the un-loading curve so that it is below
% the loading curve
posl = 2; nxl = length(xl);
for ii = 2:length(xu)
    while xu(ii) > xl(posl) && posl < nxl
        posl = posl+1;
    end
    dx = xl(posl)-xl(posl-1);
    dy = fl(posl)-fl(posl-1);
    slope = dy/dx;
    fstraight = fl(posl-1) + slope*(xu(ii)-xl(posl-1));
    if fu(ii) > 0.9*fstraight
        fu(ii) = 0.9*fstraight;
    end;
end

function [XR, XT] = comp_preload(Vxresp,Vxtir,polyrespL,polyrespUL,polytirL,...
    polytirUL,deltaG,PolyRaccResp,PolyRaccTir)

% This function is used in order to compute the preload effect XR and XT in
% buffer and draw gear in case of preload displacement deltaG

XT = fzero(@preload,deltaG,[],deltaG,polyrespL,polyrespUL,polytirL,...
    polytirUL,PolyRaccResp,PolyRaccTir,Vxresp,Vxtir);
XR = deltaG - XT;

function z = preload(xtir,deltaG,polyrespL,polyrespUL,polytirL,...
    polytirUL,PolyRaccResp,PolyRaccTir,Vxresp,Vxtir)
posR = 1;
posT = 1;
if xtir <=deltaG
    % DRAW GEARS:
    % At the beginning computes the load and un-load Fr
    Frl = interpbgdgf(polyrespL,posR,Vxresp,(deltaG-xtir));
    Frul = interpbgdgf(polyrespUL,posR,Vxresp,(deltaG-xtir));
    % now it uses the join curve in order to compute the equivalent Fr
    coef = PolyRaccResp(4); % The relative speed is set to zero
    Fr = coef*Frul + (1-coef)*Frl;
    % BUFFER GEARS:
    % At the beginning computes the load and un-load Ft
    Ftl = interpbgdgf(polytirL,posT,Vxtir,xtir);
    Ftul = interpbgdgf(polytirUL,posT,Vxtir,xtir);
    % now it uses the join curve in order to compute the equivalent Ft
    coef = PolyRaccTir(4); % The relative speed is set to zero
    Ft = coef*Ftul + (1-coef)*Ftl;
    
    % Now by imposing the equation of equilibrium it is determined the handle function
    % used in FZERO:
    z = 2*Fr-Ft;
else
    z = -1e9;
end

function bgdg = viscous_damping(bgdg,hh,ii,train)
if hh == 1,
    vdf = train(ii+1).bgvdf; vdr = train(ii).bgvdr;
elseif hh == 2,
    vdf = train(ii+1).dgvdf; vdr = train(ii).dgvdr;
end
if vdf == 0 && vdr ~=0, bgdg(hh,ii).vdeq = 1e3*vdr; elseif vdf ~= 0 && vdr == 0, bgdg(hh,ii).vdeq = 1e3*vdf; ...
elseif vdf ~= 0 && vdr ~= 0, bgdg(hh,ii).vdeq = 1e3*vdf*vdr/(vdr+vdf); else bgdg(hh,ii).vdeq = 0; end
