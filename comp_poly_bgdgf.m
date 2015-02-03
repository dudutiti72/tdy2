function P = comp_poly_bgdgf(pacco,stiff,varargin)
% Computes a piecewise polynomial approximation of the digitalized function; the
% piecewise polynomy is continous and derivable one time.
% Number of useful points
npoints = nnz(stiff(2:end)) + 1;
vnpoi = 1:npoints; % In order to short the lines
mol = 21;
if npoints >= 3
    P = zeros(1,4*(npoints-1)+1);
    if nargin > 2
        if strcmp(varargin{1},'s')
            % It assumes that the first point of the stroke is zero.
            T1 = (stiff(2)-stiff(1))/pacco(2);
        else
            T1 = 0;
        end;
    else
        T1 = 0;
    end;
    if nargin == 4, ord = varargin{2}; else ord = 3; end;
    if ord == 3
        for jj = 1:npoints-1
            if jj == npoints - 1
                T2 = (stiff(npoints)-stiff(npoints-1))/(pacco(npoints)-pacco(npoints-1));
            elseif jj == 1% && nargin == 2
                %T2 = (stiff(jj+1)-stiff(jj))/(pacco(jj+1)-pacco(jj));
                T2 = (stiff(jj+2)-stiff(jj+1))/(pacco(jj+2)-pacco(jj+1));
            else
                T2 = 0.5*((stiff(jj+1)-stiff(jj))/(pacco(jj+1)-pacco(jj)) + (stiff(jj+2)-stiff(jj+1))/(pacco(jj+2)-pacco(jj+1)));
            end;
            %if abs(T2) > mol*abs(T1) && T1 > 0%&& jj > 1
            if abs(T2) > mol*abs(T1) && T1 > 0 && jj < npoints-1
                T2 = mol*T1*sign(T2);
            end;
            P = interp3(jj,P,pacco,stiff,T1,T2);
            T1 = T2;
        end;
    elseif ord == 1
        if strcmp(varargin{1},'c')
            jj = 1;
            T2 = (stiff(jj+2)-stiff(jj+1))/(pacco(jj+2)-pacco(jj+1));
            P = interp3(jj,P,pacco,stiff,T1,T2);
        else
            P = interp1(1,P,pacco,stiff);
        end;
        for jj = 2:npoints-1
            P = interp1(jj,P,pacco,stiff);
        end;
    end;
    P(1) = npoints-1;
else
    P(1) = 1;
    P(2:3) = polyfit(pacco(vnpoi),stiff(vnpoi),1);
end;

function P = interp3(jj,P,pacco,stiff,T1,T2)

% The coefficient of the polynomial are found imposing:
% the continuity in the first point (first row of the following matrix A)
% the continuity of the slope in the first point (2nd row of A)
% the continuity in the second point (3rd row of A)
% the continuity of the slope in the second point (4th row of A)
X = [pacco(jj) pacco(jj) pacco(jj+1) pacco(jj+1)];
%X = [0 0 pacco(jj+1)-pacco(jj) pacco(jj+1)-pacco(jj)];
b = [stiff(jj);T1;stiff(jj+1);T2];
A = [X(1)^3 X(1)^2 X(1) 1;3*X(2)^2 2*X(2) 1 0;X(3)^3 X(3)^2 X(3) 1;3*X(4)^2 2*X(4) 1 0];
appo = A\b;
% T1 = T2;
P(2+(jj-1)*4:1+jj*4) = appo;

function P = interp1(jj,P,pacco,stiff)

% The coefficient of the polynomial are found imposing:
% the continuity in the first point (first row of the following matrix A)
X = [pacco(jj) pacco(jj+1)];
b = [stiff(jj);stiff(jj+1)];
A = [X(1) 1;X(2) 1];
appo = [zeros(2,1);A\b];
P(2+(jj-1)*4:1+jj*4) = appo;

