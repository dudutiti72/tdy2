function [Feldyn,loco] = brake_eldyn(eldyn,Feldyn,loco,nveicoli,ploco,t,jmloc,y)

for ii = eldyn
    if t > loco(ii).ts
        % The force vs velocity characteristic of the loco is defined for positive
        % speed only.
        [Fv,loco(ii).cvb] = interpbgdg(loco(ii).pfvb,loco(ii).cvb,loco(ii).vb,abs(y(nveicoli+ploco(ii))));
        ti = 1; % Initialization: a manoeuvre cannot start with a traction removal.
        if jmloc(ii) > 1
            ti = (loco(ii).man(jmloc(ii),10) > loco(ii).man(jmloc(ii)-1,10)); % Braking insertion
            tr = (loco(ii).man(jmloc(ii),10) < loco(ii).man(jmloc(ii)-1,10)); % Braking removal
        end
        if (loco(ii).tig == 0 && ti) || (loco(ii).trg == 0 && tr)
            % It is used the provided Force,Time characteristics
            dt = max([t - loco(ii).ts - loco(ii).elbts,0]); % Time local to the manoeuvre
            [Ft,loco(ii).ctb] = interpbgdg(loco(ii).pftb,loco(ii).ctb,loco(ii).tb,dt);
            Feldyn(ii) = min([Ft loco(ii).man(jmloc(ii),10)*Fv]);
        elseif ti
            Ft = loco(ii).Feldyn + loco(ii).tig * (t - loco(ii).tg);
            Feldyn(ii) = min([Ft loco(ii).man(jmloc(ii),10)*Fv]);
        elseif tr
            Ft = max([0 loco(ii).Feldyn - loco(ii).trg * (t - loco(ii).tg)]);
            Feldyn(ii) = max([Ft loco(ii).man(jmloc(ii),10)*Fv]);
        else
            Feldyn(ii) = loco(ii).man(jmloc(ii),10)*Fv;
        end
    end
end

