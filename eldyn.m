function [Feldyn,loco] = eldyn(eldyn,Feldyn,loco,nveicoli,t,jmloc,y)
% ii = eldyn > 0;
for ii = find(eldyn)
    if t > loco(ii).eltbs + loco(ii).ts
        dt = t - (loco(ii).eltbs + loco(ii).ts);
        [Ft,loco(ii).ct] = interpbgdg(loco(ii).pft,loco(ii).ct,loco(ii).t,dt);
        % The force vs velocity characteristic of the loco is defined for positive
        % speed only.
        [Fv,loco(ii).cv] = interpbgdg(loco(ii).pfv,loco(ii).cv,loco(ii).v,abs(y(nveicoli+1)));
        Feldyn(ii) = loco(ii).man(jmloc(ii),7)*min([Ft Fv]);
    end
end
