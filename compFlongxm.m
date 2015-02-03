function Flongxm = compFlongxm(Flong,space,T,vel_0,xm)
space = sign(vel_0(1))*space;
jmin = find(space > xm,1); j1 = 1;
Flongxm = zeros(size(Flong));
for jj = jmin:length(T)
    while (space(jj) - space(j1)) > xm
        j1 = j1+1;
    end;
%     dummy = Flong(:,j1:jj);
%     [mabs,p] = min(abs(dummy),[],2);
%     s = sign(dummy(sub2ind(size(dummy),(1:size(Flong,1))',p)));
%     Flongxm(:,jj) = s.*mabs;
    % When a sign change occurs in x meters, the corresponding Flongxm equals zero.
    m = -min(-Flong(:,j1:jj),[],2);
    M = max(Flong(:,j1:jj),[],2);
    s = sign(m.*M);
    Flongxm(s<0,jj) = 0; Flongxm(s>0,jj) = m(s>0);
    %Flongxm(:,jj) = -min(-Flong(:,j1:jj),[],2);
end
end
