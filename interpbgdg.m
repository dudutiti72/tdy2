function [F,pos] = interpbgdg(P,pos,X,x)

% INTERPBGDG This function calculates the force on the coupling gears given
%            a certain displacement by evaluating the derived piecewise
%            polynomial. It also updates the index pos for faster search
%            during the simulation. Furthermore, it is used in general to
%            evaluate interpolation polynomials (i.e friction laws)
%
% INPUTS     P:   Vector with polynomial coefficients
%            pos: Previous index to start the search from
%            X:   Stroke values of the coupled gear between which the 
%                 polynomial approximations have been computed
%            x:   Given relative displacement
%
% OUTPUTS    F:   Calculated force
%            pos: Updated index

front = 0;

while pos < P(1) && x > X(pos+1)
    pos   = pos+1; 
    front = 1;
end
while pos > 1 && x < X(pos) && front == 0
    pos   = pos-1;
end
% The x position is scaled so that at the beginning of each section its value is zero
x = x-X(pos); % [s!] This is the only line different from interpbgdgf, maybe combine two functions adding another argument to differentiate them
F = P(2+(pos-1)*4)*x^3+P(3+(pos-1)*4)*x^2+P(4+(pos-1)*4)*x+P(5+(pos-1)*4);
