function [F,pos] = interpbgdgf(P,pos,X,x)

% INTERPBGDGF This function calculates the force on the coupling gears given
%             a certain displacement by evaluating the derived piecewise
%             polynomial. It also updates the index pos for faster search
%             during the simulation
%
% INPUTS      P:   Row vector with polynomial coefficients stuck together. 
%                  First element is always the number of polynomials
%             pos: The previous state of the gear is stored in the bgdg
%                  structure to make the search of the polynomial faster
%             X:   Stroke values of the coupled gear between which the 
%                  polynomial approximations have been computed
%             x:   Given relative displacement (stroke)
%
% OUTPUTS     F:   Calculated force
%             pos: Updated polynomial index 

front = 0;

while pos < P(1) && x > X(pos+1)
    pos   = pos+1; 
    front = 1;
end
while pos > 1 && x < X(pos) && front == 0
    pos   = pos-1;
end
F = P(2+(pos-1)*4)*x^3+P(3+(pos-1)*4)*x^2+P(4+(pos-1)*4)*x+P(5+(pos-1)*4);
