function strack = ftrack(traccia,scartamento)

%# scalar CCtI deltafi fi0 fi1 fi2 fiI incrTk  k maxAsc N ntratti psi0 STE teta0
%# scalar tetaI tipopar Wn x0 y0 z0

% Numero di tratti che compongono il tracciato
ntratti = size(traccia,1);
% t: type of track: 1...straight; 2...full curve; 3...parabolic
% l: length of the track
% c: parameters of "horizontal" curvature.
% cv: parameters of "vertical" curvature.
% fi: parameters of slope.
% teta: parameters of cant.
% psi: parameters of tangent.
strack = struct('t',0,'l',0,'c',0,'cv',0,'fi',0,'teta',0,'psi',0.5*pi,'alfa',0,'s',1,'x0',0,'y0',0,...
    'C',[0 0],'B',0,'sx',+1,'sy',+1);
Nt = 100;
vx = zeros(1,Nt*size(traccia,1)); vy = vx; vs = vx; fi = vx;
% Initalization
x0 = 0;
y0 = 0;
z0 = 0;
psi0 = pi*0.5;

tetaI = traccia(1,4);
fiI = atan(traccia(1,3));

% Track slope in rad
fi0 = atan(traccia(1,3));

% Track roll in rad
teta0 = traccia(1,4);

s0 = 0; % Initalization of linear abscissa
for k = 1:ntratti
    if traccia(k,6) == 1  % Straight stretch
        L = traccia(k,1);
        strack(k).t = 1;                        %Type
        strack(k).l = sum(traccia(1:k-1,1));	%Length
        strack(k).c = 0;                        %Curvature (horizontal)
        strack(k).psi = psi0;                   %Tangent respect to XZ plane
        strack(k).fi = fi0;                     %Slope
        strack(k).teta = teta0;                 %Superelevation
        strack(k).x0 = x0; strack(k).y0 = y0;

        sloc = linspace(0,L,Nt);
        vx((k-1)*Nt +1:k*Nt) = x0 + cos(psi0)*sloc;
        vy((k-1)*Nt +1:k*Nt) = y0 + sin(psi0)*sloc;
        vs((k-1)*Nt +1:k*Nt) = s0 + sloc;
        fi((k-1)*Nt +1:k*Nt) = strack(k).fi*ones(1,Nt);
        x0 = x0 + cos(psi0)*L;                  %New x starting coordinate of the next piece of track
        y0 = y0 + sin(psi0)*L;                  %The same as above except that for y.
        s0 = vs(k*Nt);
    elseif traccia(k,6) == 2    % Full curve stretch
        L = traccia(k,1);
        strack(k).t = 2;
        strack(k).l = sum(traccia(1:k-1,1));
        R = traccia(k,2);                       % Curvature radius + right, - left
        strack(k).c = 1/R;
        strack(k).psi = psi0;                   %Tangent respect to XZ plane
        strack(k).fi = fi0;                     %Slope
        strack(k).teta = teta0;                 %Superelevation
        strack(k).x0 = x0; strack(k).y0 = y0;

        n0 = [-sin(psi0),cos(psi0)];
        % Matrix of rotation
        ROT = [cos(psi0-0.5*pi) -sin(psi0-0.5*pi);sin(psi0-0.5*pi) cos(psi0-0.5*pi)];
        tetaloc = pi - L/abs(R);
        if R < 0
            n0 = -n0;
            tetaloc = pi - tetaloc;
        end;
        C = [x0 - n0(1)*abs(R),y0 - n0(2)*abs(R)];% Center of the circle
        strack(k).C = C;

        x0loc = abs(R)*cos(tetaloc); y0loc = abs(R)*sin(tetaloc);
        appo = ROT*[x0loc;y0loc];
        x0 = appo(1) + C(1); y0 = appo(2) + C(2);
        psi0 = psi0 - L/R;

        sloc = linspace(0,traccia(k,1),Nt);
        vtetaloc = pi - sloc/abs(R);
        if R < 0
            vtetaloc = pi - vtetaloc;
        end;
        vx0loc = abs(R)*cos(vtetaloc); vy0loc = abs(R)*sin(vtetaloc);
        appo = ROT*[vx0loc;vy0loc];
        vx((k-1)*Nt +1:k*Nt) = C(1) + appo(1,:);
        vy((k-1)*Nt +1:k*Nt) = C(2) + appo(2,:);
        vs((k-1)*Nt +1:k*Nt) = s0 + sloc;
        fi((k-1)*Nt +1:k*Nt) = strack(k).fi*ones(1,Nt);
        s0 = vs(k*Nt);
    elseif traccia(k,6) == 3 && traccia(k,5) == 0    % Parabolic stretch (Clothoid, Cornu spiral)
        % Computation in the local reference frame
        L = traccia(k,1);
        tfc = 0; % Indicator to check if the clothoid is between two full curves
        if traccia(k-1,2) == 0
            % Previous track is a straight line
            R = traccia(k+1,2);
        elseif traccia(k+1,2) == 0
            % Following track is a straight line
            R = traccia(k-1,2);
        elseif traccia(k-1,2) ~= 0 && traccia(k+1,2) ~= 0
            % The transition is between to full curves of different radious
            tfc = 1;
        else
            error('Track not handled');
        end;
        strack(k).t = 3;
        strack(k).l = sum(traccia(1:k-1,1));
        %strack(k).c = 1/R;
        strack(k).psi = psi0;                   %Tangent respect to XZ plane
        strack(k).fi = fi0;                     %Slope
        strack(k).teta = teta0;                 %Superelevation
        strack(k).x0 = x0; strack(k).y0 = y0;
        
        fi((k-1)*Nt +1:k*Nt) = strack(k).fi + (traccia(k+1,3) - strack(k).fi)/L*linspace(0,L,Nt);
        
        fi0 = traccia(k+1,3);
        teta0 = traccia(k+1,4);
        vs((k-1)*Nt +1:k*Nt) = s0+linspace(0,L,Nt);
        s0 = vs(k*Nt);

        if tfc == 0
            % The clothoid is between a straight line and a full curve
            B = realsqrt(L*abs(R)*pi);
            strack(k).B = B;
            tfin = L/B;
            if traccia(k-1,2) == 0 % Previos track is a straight line
                segnoy = +1;
            elseif traccia(k+1,2) == 0 % Following track is a straight line
                segnoy = -1;
            end;
            if R > 0, segnox = +1; else segnox = -1; end;
            strack(k).sx = segnox; strack(k).sy = segnoy;
            [FresnelC,FresnelS] = fresnelCS(tfin);
            xfin = segnox*B*FresnelS; yfin = segnoy*B*FresnelC;
            taufin = 0.5*pi*tfin^2;

            sloc = linspace(0,L,Nt);
            tfin = sloc/B;
            [FresnelC,FresnelS] = fresnelCS(tfin);
            vxfin = segnox*B*FresnelS; vyfin = segnoy*B*FresnelC;

            if traccia(k-1,2) == 0 % Previos track is a straight line
                vx((k-1)*Nt +1:k*Nt) = x0 + cos(psi0-0.5*pi)*(vxfin) - sin(psi0-0.5*pi)*(vyfin);
                vy((k-1)*Nt +1:k*Nt) = y0 + sin(psi0-0.5*pi)*(vxfin) + cos(psi0-0.5*pi)*(vyfin);
                x0 = x0 + cos(psi0-0.5*pi)*xfin - sin(psi0-0.5*pi)*yfin;
                y0 = y0 + sin(psi0-0.5*pi)*xfin + cos(psi0-0.5*pi)*yfin;
                psi0 = psi0 - segnox*taufin;
            elseif traccia(k+1,2) == 0 % Following track is a straight line
                psi0 = psi0 - segnox*taufin;
                vx((k-1)*Nt +1:k*Nt) = x0 + cos(psi0-0.5*pi)*(vxfin(end:-1:1)-vxfin(end)) - sin(psi0-0.5*pi)*(vyfin(end:-1:1)-vyfin(end));
                vy((k-1)*Nt +1:k*Nt) = y0 + sin(psi0-0.5*pi)*(vxfin(end:-1:1)-vxfin(end)) + cos(psi0-0.5*pi)*(vyfin(end:-1:1)-vyfin(end));
                x0 = x0 + cos(psi0-0.5*pi)*(segnoy*xfin) - sin(psi0-0.5*pi)*(segnoy*yfin);
                y0 = y0 + sin(psi0-0.5*pi)*(segnoy*xfin) + cos(psi0-0.5*pi)*(segnoy*yfin);
            end;
        else
            % The clothoid is between two full curves
            C1 = 1/traccia(k-1,2); C2 = 1/traccia(k+1,2);
            B2 = pi*L/abs(C2-C1);
            s10 = B2*C1/pi; s20 = B2*C2/pi;
            s = linspace(s10,s20,Nt);
            t = s/realsqrt(B2);

            [FresnelC,FresnelS] = fresnelCS(t);
            segnox = 1; segnoy = 1;
            tau10 = pi*0.5*t(1)^2;
            tau20 = pi*0.5*t(end)^2;
            strack(k).s = 1;
            if C1 > 0
                if C2 > 0 && C2 < C1
                    segnoy = -1;
                    % TODO: Try to change in tau10x
                    tau10x = tau10 - pi;
                    taux = tau10x - pi*0.5;
                    alfa = psi0 - taux;
                    psi0 = psi0 - (tau10 - tau20);
                    strack(k).s = -1;
                elseif C2 > 0 && C2 > C1
                    taux =  (+ pi*0.5 - tau10);
                    alfa = psi0 - taux;
                    psi0 = psi0 - (tau20 - tau10);
                elseif C2 < 0
                    segnoy = -1;
                    tau10x = tau10 - pi;
                    taux = tau10x - pi*0.5;
                    alfa = psi0 - taux;
                    psi0 = psi0 - (tau10 - tau20);
                    strack(k).s = -1;
                end;
            else
                if C2 < 0 && C2 > C1
                    tau10x = pi -tau10;
                    taux = tau10x - pi*0.5;
                    alfa = psi0 - taux;
                    psi0 = psi0 - (tau20 - tau10);
                    strack(k).s = -1;
                elseif C2 < 0 && C2 < C1
                    segnoy = -1;
                    taux = tau10 + pi*0.5;
                    alfa = psi0 - taux;
                    psi0 = psi0 - (tau10 - tau20);
                else
                    segnox = -1; segnoy = -1;
                    tau10x = pi -tau10;
                    taux = tau10x + pi*0.5;
                    alfa = psi0 - taux;
                    psi0 = psi0 - (tau20 - tau10);
                    strack(k).s = -1;
                end;
            end;
            x = segnox*realsqrt(B2)*FresnelS; y = segnoy*realsqrt(B2)*FresnelC;

            vx((k-1)*Nt +1:k*Nt) = x0 + cos(alfa)*(x-x(1)) - sin(alfa)*(y-y(1));
            vy((k-1)*Nt +1:k*Nt) = y0 + sin(alfa)*(x-x(1)) + cos(alfa)*(y-y(1));
            x0 = vx(k*Nt); y0 = vy(k*Nt);
            strack(k).B = realsqrt(B2);
            strack(k).sx = segnox; strack(k).sy = segnoy;
            strack(k).alfa = alfa;
        end;
    elseif traccia(k,5) ~= 0
        % There is a change in the slope
        L = traccia(k,1);
        strack(k).t = 3;                        %Type
        strack(k).l = sum(traccia(1:k-1,1));	%Length
        strack(k).c = 0;                        %Curvature (horizontal)
        strack(k).psi = psi0;                   %Tangent respect to XZ plane
        strack(k).fi = fi0;                     %Slope
        strack(k).teta = teta0;                 %Superelevation
        strack(k).x0 = x0; strack(k).y0 = y0;


        Rv = traccia(k,5);                      % Curvature radius + uphill, - downhill
        strack(k).cv = 1/Rv;                    %Curvature (vertical)

%         n0 = [-sin(psi0),cos(psi0)];
%         % Matrix of rotation
%         ROT = [cos(psi0-0.5*pi) -sin(psi0-0.5*pi);sin(psi0-0.5*pi) cos(psi0-0.5*pi)];
%         tetaloc = pi - L/abs(R);
%         if R < 0
%             n0 = -n0;
%             tetaloc = pi - tetaloc;
%         end;
%         C = [x0 - n0(1)*abs(R),y0 - n0(2)*abs(R)];% Center of the circle
%         strack(k).C = C;
% 
%         x0loc = abs(R)*cos(tetaloc); y0loc = abs(R)*sin(tetaloc);
%         appo = ROT*[x0loc;y0loc];
%         x0 = appo(1) + C(1); y0 = appo(2) + C(2);
%         psi0 = psi0 - L/R;
% 
%         sloc = linspace(0,traccia(k,1),Nt);
%         vtetaloc = pi - sloc/abs(R);
%         if R < 0
%             vtetaloc = pi - vtetaloc;
%         end;
%         vx0loc = abs(R)*cos(vtetaloc); vy0loc = abs(R)*sin(vtetaloc);
%         appo = ROT*[vx0loc;vy0loc];
%         vx((k-1)*Nt +1:k*Nt) = C(1) + appo(1,:);
%         vy((k-1)*Nt +1:k*Nt) = C(2) + appo(2,:);
%         vs((k-1)*Nt +1:k*Nt) = s0 + sloc;
%         fi((k-1)*Nt +1:k*Nt) = strack(k).fi*ones(1,Nt);
%         s0 = vs(k*Nt);
% 
%         
        sloc = linspace(0,L,Nt);
%         vx((k-1)*Nt +1:k*Nt) = x0 + cos(psi0)*sloc;
%         vy((k-1)*Nt +1:k*Nt) = y0 + sin(psi0)*sloc;
        vs((k-1)*Nt +1:k*Nt) = s0 + sloc;
        fi((k-1)*Nt +1:k*Nt) = fi0 + sloc/Rv;
%         x0 = x0 + cos(psi0)*L;                  %New x starting coordinate of the next piece of track
%         y0 = y0 + sin(psi0)*L;                  %The same as above except that for y.
        s0 = vs(k*Nt); fi0 = fi(k*Nt);
        
    end;
end;
% figure(101);
% subplot(2,1,1); plot(vx,vy);
% axis('equal'); grid on; zoom on;
% subplot(2,1,2); plot(vs,fi); grid on; zoom on;


