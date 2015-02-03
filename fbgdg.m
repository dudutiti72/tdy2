function [bgdg,CCtrac,Fel,Feld,Fels,fitrac,ipos] = fbgdg(bgdg,CCtrac,Fel,Feld,Fels,...
    fitrac,ipos,lwagcs,nveicoli,strack,traccia,vr,y,xr)


[Cp,fi,ipos,~] = track_car(1,ipos,lwagcs,strack,traccia,y);
fitrac(1) = fi; CCtrac(1) = Cp;
for ii = 1:nveicoli-1
    % Track condition of the following vehicle of coupling ii
    [Cd,fi,ipos,psid] = track_car(ii+1,ipos,lwagcs,strack,traccia,y);
    fitrac(ii+1) = fi; CCtrac(ii+1) = Cd;
    % Updating
    psip = psid; Cp = Cd;
    
    Felt = 0; Felbgd = 0; Felbgs = 0; % Initializations
    % DRAW GEAR
    if xr(ii) + bgdg(2,ii).gap > 0
        xr(ii) = xr(ii) + bgdg(2,ii).gap;
        % Draw gear is active
        if vr(ii) < -bgdg(2,ii).vpu
            [Felbgdg,bgdg(2,ii).c] = interpbgdgf(bgdg(2,ii).pu,bgdg(2,ii).c,bgdg(2,ii).xu,xr(ii));
        elseif vr(ii) > bgdg(2,ii).vpl
            [Felbgdg,bgdg(2,ii).c] = interpbgdgf(bgdg(2,ii).pl,bgdg(2,ii).c,bgdg(2,ii).xl,xr(ii));
        else
            Pcoef = bgdg(2,ii).Plu;
            coef = Pcoef(1)*(vr(ii))^3+Pcoef(2)*vr(ii)^2+Pcoef(3)*(-vr(ii))+Pcoef(4);
            [Felbgdgu,bgdg(2,ii).c] = interpbgdgf(bgdg(2,ii).pu,bgdg(2,ii).c,bgdg(2,ii).xu,xr(ii));
            [Felbgdgl,bgdg(2,ii).c] = interpbgdgf(bgdg(2,ii).pl,bgdg(2,ii).c,bgdg(2,ii).xl,xr(ii));
            Felbgdg = coef*Felbgdgu + (1-coef)*Felbgdgl;
        end;
        % There is also the effect of the damping proportional to the relative
        % speed
        Felt = Felbgdg*cos(psip-psid) - bgdg(2,ii).vdeq*vr(ii);
    end;
    % BUFFING GEAR
    if bgdg(1,ii).CentrC == 0
        % There is no Central Coupler
        % The different compression of the two side buffers are computed
        [psirel,vrd,vrs,xrd,xrs] = compdesin(Cd,Cp,psid,psip,vr(ii),xr(ii));
        
        if xrd + bgdg(1,ii).gap < 0
            % Right buffing gear is active
            xrd = xrd + bgdg(1,ii).gap;
            if vrd > bgdg(1,ii).vpu
                [Felbgdg,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pu,bgdg(1,ii).c,bgdg(1,ii).xu,-xrd);
            elseif vrd <  -bgdg(1,ii).vpl
                [Felbgdg,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pl,bgdg(1,ii).c,bgdg(1,ii).xl,-xrd);
            else
                Pcoef = bgdg(1,ii).Plu;
                coef = Pcoef(1)*vr(ii)^3+Pcoef(2)*vr(ii)^2+Pcoef(3)*vr(ii)+Pcoef(4);
                [Felbgdgu,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pu,bgdg(1,ii).c,bgdg(1,ii).xu,-xrd);
                [Felbgdgl,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pl,bgdg(1,ii).c,bgdg(1,ii).xl,-xrd);
                Felbgdg = coef*Felbgdgu + (1-coef)*Felbgdgl;
            end;
            % There is also the effect of the damping proportional to the relative
            % speed
            Felbgd = -Felbgdg*cos(psirel)  + bgdg(1,ii).vdeq*vrd;
        end;
        
        if xrs + bgdg(3,ii).gap < 0
            % Left buffing gear is active
            xrs = xrs + bgdg(3,ii).gap;
            if vrs > bgdg(3,ii).vpu
                [Felbgdg,bgdg(3,ii).c] = interpbgdgf(bgdg(3,ii).pu,bgdg(3,ii).c,bgdg(3,ii).xu,-xrs);
            elseif vrs < -bgdg(3,ii).vpl
                [Felbgdg,bgdg(3,ii).c] = interpbgdgf(bgdg(3,ii).pl,bgdg(3,ii).c,bgdg(3,ii).xl,-xrs);
            else
                Pcoef = bgdg(3,ii).Plu;
                coef = Pcoef(1)*vr(ii)^3+Pcoef(2)*vr(ii)^2+Pcoef(3)*vr(ii)+Pcoef(4);
                [Felbgdgu,bgdg(3,ii).c] = interpbgdgf(bgdg(3,ii).pu,bgdg(3,ii).c,bgdg(3,ii).xu,-xrs);
                [Felbgdgl,bgdg(3,ii).c] = interpbgdgf(bgdg(3,ii).pl,bgdg(3,ii).c,bgdg(3,ii).xl,-xrs);
                Felbgdg = coef*Felbgdgu + (1-coef)*Felbgdgl;
            end;
            % There is also the effect of the damping proportional to the relative
            % speed
            Felbgs = -Felbgdg*cos(psirel)  + bgdg(3,ii).vdeq*vrs;
        end;
        % Computation of the resulting force of the coupling
        Fel(ii) = Felt + Felbgd + Felbgs;
        Feld(ii) = Felbgd; Fels(ii) = Felbgs; % Storing also the right ann left forces (not supported by the GUI)
    else
        % There is a central buffer
        Felbg = Felbgd;
        if xr(ii) + bgdg(1,ii).gap < 0
            % Central Buffer is active
            xr(ii) = xr(ii) + bgdg(1,ii).gap;
            if vr(ii) > bgdg(1,ii).vpu
                [Felbgdg,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pu,bgdg(1,ii).c,bgdg(1,ii).xu,-xr(ii));
            elseif vr(ii) <  -bgdg(1,ii).vpl
                [Felbgdg,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pl,bgdg(1,ii).c,bgdg(1,ii).xl,-xr(ii));
            else
                Pcoef = bgdg(1,ii).Plu;
                coef = Pcoef(1)*vr(ii)^3+Pcoef(2)*vr(ii)^2+Pcoef(3)*vr(ii)+Pcoef(4);
                [Felbgdgu,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pu,bgdg(1,ii).c,bgdg(1,ii).xu,-xr(ii));
                [Felbgdgl,bgdg(1,ii).c] = interpbgdgf(bgdg(1,ii).pl,bgdg(1,ii).c,bgdg(1,ii).xl,-xr(ii));
                Felbgdg = coef*Felbgdgu + (1-coef)*Felbgdgl;
            end;
            % There is also the effect of the damping proportional to the relative
            % speed
            Felbg = -Felbgdg*cos(psip-psid) + bgdg(1,ii).vdeq*vr(ii);
        end;
        % Computation of the resulting force of the coupling
        Fel(ii) = Felt + Felbg;
        Feld(ii) = Felbg; Fels(ii) = Felbg; % Storing also the right ann left forces (not supported by the GUI)
    end        
end;


function [psirel,vrd,vrs,xrd,xrs] = compdesin(Cd,Cp,psid,psip,vr,xr)

% FIXME: Put in input as a costructive parameter
DltR = 0.875; % Distance of the side buffers
psirel = psip-psid; % Relative angle
xrd = xr + psirel*DltR; xrs = xr - psirel*DltR; % Right and left compression
psiprel = vr*(Cp - Cd); % Relative angular speed
vrd = vr + psiprel*DltR; vrs = vr - psiprel*DltR; % Right and left compression speed

