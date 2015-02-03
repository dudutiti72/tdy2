function P = comp_poly_fc(pacco,stiff)

% COMP_POLY_FC This function computes a piecewise polynomial approx based
%              on the discrete points (pacco,stiff). The result is
%              continuous and differentiable in time
%
% INPUTS       pacco: X values 
%              stiff: Y values
%
% OUTPUT       P: Coefficients of computed polynomial

% Number of useful points
npoints = nnz(stiff(2:end)) + 1;
vnpoi   = 1:npoints; % In order to short the lines
if npoints >= 3
    for jj = 1:npoints-1
        % T1 is the tangent at the beginning of the section, T2 is the same at the
        % end
        if jj == 1
            T1 = (stiff(2)-stiff(1))/(pacco(2)-pacco(1));
        else
            T1 = 0.5*((stiff(jj)-stiff(jj-1))/(pacco(jj)-pacco(jj-1)) + (stiff(jj+1)-stiff(jj))/(pacco(jj+1)-pacco(jj)));
        end
        if jj == npoints - 1
            T2 = (stiff(npoints)-stiff(npoints-1))/(pacco(npoints)-pacco(npoints-1));
        else
            T2 = 0.5*((stiff(jj+1)-stiff(jj))/(pacco(jj+1)-pacco(jj)) + (stiff(jj+2)-stiff(jj+1))/(pacco(jj+2)-pacco(jj+1)));
        end
        X = [0 0 pacco(jj+1)-pacco(jj) pacco(jj+1)-pacco(jj)];
        b = [stiff(jj);T1;stiff(jj+1);T2];
        A = [X(1)^3 X(1)^2 X(1) 1;3*X(2)^2 2*X(2) 1 0;X(3)^3 X(3)^2 X(3) 1;3*X(4)^2 2*X(4) 1 0];
        % This polynomial has the tangent T1 and T2 and it passes from the first and
        % last point of the section
        P(2+(jj-1)*4:1+jj*4) = A\b;
    end
    P(1) = npoints-1; % Number of section managed by the polynomial
else
    % In case there are only two points
    P(1)   = 1;
    P(4:5) = polyfit(pacco(vnpoi),stiff(vnpoi),1);
end
