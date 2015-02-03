function Bauto = compB(auto,wmass)

% COMPB   This function computes the actual braked weight in vehicles 
%         equipped with an autocontinous system, taking into account their
%         total mass
%
% INPUTS  auto:  Array with two columns. The first column contains weights
%                and the second the corresponding braked weight percentage
%         wmass: Total vehicle mass (Tare + load)
%
% OUTPUTS Bauto: Automatically adjusted braked weight

% Index of the last mass below wmass in brake definition
r = find(auto(:,1) <= wmass,1,'last');

if isempty(r), error('Mass too low'); end;

if r == size(auto,1)
    if wmass < auto(end,1)*1.001
        r = r - 1;
    else
        error('Mass too high'); 
    end
end
d = (auto(r+1,1)-auto(r,1));
s = (auto(r+1,2)-auto(r,2))/d;
p = auto(r,2) + s*(wmass-auto(r,1)); % Percentage of brake weight

if p > 3, p = p*1e-2; end

Bauto = p*wmass;

end
