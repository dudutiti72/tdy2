function S = TPspline(S,tempo,tend,SR)

t = 0:SR:tend;
ndata = size(S,2);
newdata = zeros(size(t,2),ndata);
for ii = 1:ndata
    newdata(:,ii) = spline(tempo,S(:,ii),t)';
end;
S = newdata;
