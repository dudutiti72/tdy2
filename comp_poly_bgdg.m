function P = comp_poly_bgdg(pacco,stiff)
% Computes a piecewise polynomial approximation of the digitalized function; the
% piecewise polynomy is continous and derivable one time.
% Number of useful points
npoints = nnz(stiff(2:end)) + 1;
vnpoi = 1:npoints; % In order to short the lines
mol = 2.1;
if npoints >= 3
    T1 = (stiff(2)-stiff(1))/(pacco(2)-pacco(1));
    for jj = 1:npoints-1
        if jj == npoints - 1
            T2 = (stiff(npoints)-stiff(npoints-1))/(pacco(npoints)-pacco(npoints-1));
        else
            T2 = 0.5*((stiff(jj+1)-stiff(jj))/(pacco(jj+1)-pacco(jj)) + (stiff(jj+2)-stiff(jj+1))/(pacco(jj+2)-pacco(jj+1)));
        end;
        if abs(T2) > mol*abs(T1) && abs(T1) > 1e-9
            T2 = mol*T1*sign(T2);
        end;
        %X = [pacco(jj) pacco(jj) pacco(jj+1) pacco(jj+1)];
        X = [0 0 pacco(jj+1)-pacco(jj) pacco(jj+1)-pacco(jj)];
        b = [stiff(jj);T1;stiff(jj+1);T2];
        A = [X(1)^3 X(1)^2 X(1) 1;3*X(2)^2 2*X(2) 1 0;X(3)^3 X(3)^2 X(3) 1;3*X(4)^2 2*X(4) 1 0];
        P(2+(jj-1)*4:1+jj*4) = A\b;
        % Updating
        T1 = T2;
    end;
    P(1) = npoints-1;
else
    P(1) = 1;
    P(2:3) = polyfit(pacco(vnpoi),stiff(vnpoi),1);
end;
