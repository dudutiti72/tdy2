function repl_DataSA(col,nfE,nfT)

% nfE = '1#001.txt';
% nfT = 'DataSA.txt';
% col = [1 2 7 11 32];

% This function replaces the columns col of dataSA using the experimental counterpart
T = load(nfT); E = load(nfE);
% dtSA = T(2,1)-T(1,1); dtE = E(2,1)-E(1,1);

X = E(:,1); XX = T(:,1);
for ii = 1:length(col)
   ic = col(ii)+3;
   % Re-sampling experimental data
   Y = E(:,ii+1);
   YY = spline(X,Y,XX);
   T(:,ic) = YY;
   
end
generatxt([nfT 'r'],T);

end

