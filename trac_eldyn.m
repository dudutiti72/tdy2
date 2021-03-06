function [Feldyn,loco] = trac_eldyn(trac,Feldyn,loco,nveicoli,ploco,t,jmloc,y)
for ii = trac
    %if t > loco(ii).eltts + loco(ii).ts % The field eltts is "useless" since it can be substituted by setting a manoeuvre where the loco does nothing
    if t > loco(ii).ts % The field eltts is "useless" since it can be substituted by setting a manoeuvre where the loco does nothing
        % The force vs velocity characteristic of the loco is defined for positive
        % speed only.
        [Fv,loco(ii).cv] = interpbgdg(loco(ii).pfv,loco(ii).cv,loco(ii).v,abs(y(nveicoli+ploco(ii))));
        ti = 1; % Initialization: a manoeuvre cannot start with a traction removal.
        tr = 0;
        if jmloc(ii) > 1
            ti = (loco(ii).man(jmloc(ii),10) > loco(ii).man(jmloc(ii)-1,10)); % Traction insertion
            tr = (loco(ii).man(jmloc(ii),10) < loco(ii).man(jmloc(ii)-1,10)); % Traction removal
        end
        if (loco(ii).tig == 0 && ti) || (loco(ii).trg == 0 && tr)
            % It is used the provided Force,Time characteristics
            dt = max([t - loco(ii).ts - loco(ii).eltts,0]); % Time local to the manoeuvre
            [Ft,loco(ii).ct] = interpbgdg(loco(ii).pft,loco(ii).ct,loco(ii).t,dt);
            Feldyn(ii) = min([Ft loco(ii).man(jmloc(ii),10)*Fv]);
        elseif ti
            Ft = loco(ii).Ftrac + loco(ii).tig * (t - loco(ii).tg);
            Feldyn(ii) = min([Ft loco(ii).man(jmloc(ii),10)*Fv]);
        elseif tr
            Ft = loco(ii).Ftrac - loco(ii).trg * (t - loco(ii).tg);
            Feldyn(ii) = max([Ft loco(ii).man(jmloc(ii),10)*Fv]);
        else
            Feldyn(ii) = loco(ii).man(jmloc(ii),10)*Fv;
        end
    end
end
