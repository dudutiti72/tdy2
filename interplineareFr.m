function v = interplineareFr(ascissa,ordinata,valori)

% Questa funzione è stata riadattata dalla funzione MATLAB interp1

% Caratteristiche delle variabili d'ingresso:
% ascissa e ordinata sono vettori colonna con la stessa dimensione
% valori deve essere un vettore riga

%# scalar j ii m n q

u = valori; 
y = ordinata; 


[m,n] = size(y);
x = ascissa;

h = diff(x);

siz = size(u);
u = u(:);
p = [];

v = zeros(size(u,1),n*size(u,2));
q = size(u,1);
if any(diff(u) < 0)
	[u,p] = sort(u);
else
	p = 1:q;
end

% Find indices of subintervals, x(k) <= u < x(k+1), 
% or u < x(1) or u >= x(m-1).

if isempty(u)
    k = u;
else
    k = zeros(siz(2),1);
    %# fastindex
    for ii = 1:siz(2)
        appo = find(x <= u(ii));
        if ~isempty(appo)
            %%# fastindex
            k(ii,1) = appo(size(appo,1));
        end;
    end;
    k(u < x(1) | ~isfinite(u)) = 1;
    k(u >= x(m)) = m-1;
end

	
s = u - x(k);
for j = 1:n
	del = diff(y(:,j))./h;
	v(p,j) = y(k,j) + s.*del(k);
end
	
if min(size(v)) == 1 & prod(siz) > 1
   v = reshape(v,siz);
end

v(find(v < 0)) = 0;
v(find(v > 3.87)) = 3.87;

%INTERP1 1-D interpolation (table lookup).
%   YI = INTERP1(X,Y,XI) interpolates to find YI, the values of
%   the underlying function Y at the points in the vector XI.
%   The vector X specifies the points at which the data Y is
%   given. If Y is a matrix, then the interpolation is performed
%   for each column of Y and YI will be length(XI)-by-size(Y,2).
%
%   YI = INTERP1(Y,XI) assumes X = 1:N, where N is the length(Y)
%   for vector Y or SIZE(Y,1) for matrix Y.
%
%   Interpolation is the same operation as "table lookup".  Described in
%   "table lookup" terms, the "table" is [X,Y] and INTERP1 "looks-up"
%   the elements of XI in X, and, based upon their location, returns
%   values YI interpolated within the elements of Y.
%
%   YI = INTERP1(X,Y,XI,'method') specifies alternate methods.
%   The default is linear interpolation.  Available methods are:
%
%     'nearest'  - nearest neighbor interpolation
%     'linear'   - linear interpolation
%     'spline'   - piecewise cubic spline interpolation (SPLINE)
%     'pchip'    - piecewise cubic Hermite interpolation (PCHIP)
%     'cubic'    - same as 'pchip'
%     'v5cubic'  - the cubic interpolation from MATLAB 5, which does not
%                  extrapolate and uses 'spline' if X is not equally spaced.
%
%   YI = INTERP1(X,Y,XI,'method','extrap') uses the specified method for
%   extrapolation for any elements of XI outside the interval spanned by X.
%   Alternatively, YI = INTERP1(X,Y,XI,'method',EXTRAPVAL) replaces
%   these values with EXTRAPVAL.  NaN and 0 are often used for EXTRAPVAL.
%   The default extrapolation behavior with four input arguments is 'extrap'
%   for 'spline' and 'pchip' and EXTRAPVAL = NaN for the other methods.
%
%   For example, generate a coarse sine curve and interpolate over a
%   finer abscissa:
%       x = 0:10; y = sin(x); xi = 0:.25:10;
%       yi = interp1(x,y,xi); plot(x,y,'o',xi,yi)
%
%   See also INTERP1Q, INTERPFT, SPLINE, INTERP2, INTERP3, INTERPN.

%   Copyright 1984-2001 The MathWorks, Inc.
%   $Revision: 5.38 $  $Date: 2001/04/15 11:59:10 $
