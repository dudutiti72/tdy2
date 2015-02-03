function [P,newx,newy] = poly_trac(delta,x,y)

% POLY_TRAC Computes a piecewise 3rd order polynomial approximations based 
%           on the given points (x,y)
%
% INPUTS    delta: Minimum allowed distance between the points of the 
%                  characteristic polynomial
%           x:     X values (Speed or time) 
%           y:     Y values (Traction or brake force)
%
% OUTPUTS   P:     Polynomial coefficients stuck together. The first 
%                  element is  always the number of polynomials
%           newx:  Modified x values according to delta
%           newy:  Modified y values according to delta

mol = 1.5;

% Checking the input
if delta >= min(diff(x)), delta = min(diff(x))*0.01; end;

if size(x,1) ~= size(y,1) && size(x,2)~= size(y,2)
    error('Check the coherence of the input');
elseif size(x,1) < size(x,2)
    % In the following it is assumed to deal with column vectors
    x = x'; y = y';
end
nx = size(x,1); % Number of points of the characteristic
if nx >= 3
    P  = zeros(1,3*4+1); % Initialization of the vectors that stores the polynomial coefficients
    nx = size(x,1);      % Number of points
    jj = 1;
    while jj < nx-1
        dy1 = y(jj+1)-y(jj); 
        dx1 = x(jj+1)-x(jj);
        s1  = dy1/dx1;
        dy2 = y(jj+2)-y(jj+1); 
        dx2 = x(jj+2)-x(jj+1);
        s2  = dy2/dx2;
        % Note 1.01 are added in order to avoid numerical rounding errors
        if (abs(s2) > 1.01*abs(s1)*mol || abs(s2)*1.01 < abs(s1)/mol || sign(s1) ~= sign(s2)) ...
                && dx1+dx2 > 2.01*delta
            x1 = x(jj+1)-delta; x2 = x(jj+1)+delta;
            y1 = s1*x1-s1*x(jj)+y(jj); y2 = s2*x2-s2*x(jj+1)+y(jj+1);
            x  = [x(1:jj);x1;x2;x(jj+2:end)];
            y  = [y(1:jj);y1;y2;y(jj+2:end)];
            P3 = poly3(dx1-delta,s1,s1,y(jj:jj+1));
            %P3 = poly3(dx1-delta,s1,s2*0.5,y(jj:jj+1));
            P(2+(jj-1)*4:1+jj*4) = P3;
            jj = jj+1;
            P3 = poly3(2*delta,s1,s2,y(jj:jj+1)); % Chamfer
            %P3 = poly3(2*delta,s2*0.5,s2,y(jj:jj+1)); % Chamfer
            P(2+(jj-1)*4:1+jj*4) = P3;
            jj = jj+1; nx = nx+1; T1 = 0;
        else
            % Any chamfer is necassary
            if jj == 1 || T1 == 0, T1 = s1; end
            T2 = 0.5*(s1+s2);
            P3 = poly3(dx1,T1,T2,y(jj:jj+1));
            P(2+(jj-1)*4:1+jj*4) = P3;
            T1 = T2;
            jj = jj+1;
        end

    end
    % The last part is approximated by a straight line
    m = diff(y(nx-1:nx))/diff(x(nx-1:nx)); % Slope
    %q = y(nx-1)-m*x(nx-1); % Intersection with zero
    %It is assumed that the first abscissa is always zero
    q = y(nx-1); % Intersection with zero
    P(2+(nx-2)*4:1+(nx-1)*4) = [0 0 m q];

    P(1) = (nx-1);
else
    % The characteristic can be approximated by a straight line
    P(1) = 1;
    P(4) = diff(y)/diff(x); % Slope
    P(5) = y(1)-P(4)*x(1); % Intersection with zero
end

newx = x; newy = y;

end

function P3 = poly3(dx,T1,T2,y)
% Third order polynomial. It is assumed that the first abscissa is always zero
X  = [0 0 dx dx];
b  = [y(1);T1;y(2);T2];
A  = [X(1)^3 X(1)^2 X(1) 1;3*X(2)^2 2*X(2) 1 0;X(3)^3 X(3)^2 X(3) 1;3*X(4)^2 2*X(4) 1 0];
P3 = A\b;
end