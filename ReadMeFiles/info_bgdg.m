
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
%                           BGDG struct array                             %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % [SBB] % % % % %

% Size:    3 x (nveicoli-1)
% 1st row: Right buffer
% 2nd row: Draw  gear
% 3rd row: Left  buffer

% Note: 1st col includes the coupling of first vehicle with the second and 
%       so on and so forth

% x:      [s!] maybe this field, initialized in prel_comp is outdated
% fl:     Vector with force values of coupled info (load characteristic)
% fu:     Vector with force values of coupled info (unload char.)
% pl:     Piecewise 3rd order polynomial approximation of load data based
%         on the discrete points (xl,fl). First element indicates the
%         number of polynomials and the rest are the polynomial terms stuck 
%         together
% pu:     Piecewise 3rd order polynomial approximation of unload data based
%         on the discrete points (xu,fu)
% c:      Constantly updated index to store which polynomial was used last
% xl:     Vector with stroke values of coupled info (load characteristic)
% xu      Vector with stroke values of coupled info (unload char.)
% CentrC: Central coupling ( Note that it must hold ccr(ii) = ccf(ii+1) )
% vdeq:   Combined viscious damping coefficient calculated in subfunction
%         viscous_damping in function prel_comp
% vpu:    Unload limiting velocity
% vpl:    Load limiting velocity
% Plu:    Polynomial used for determining the coupling force when the 
%         approaching speed is not greater than vpl or the distancing speed
%         is not greater than vpu. It is written as c(v) in user guide p.72
% gap:    Coupled gap info. If gap < 0 both buffers and draw gears are
%         influenced. Otherwise only the buffers (1st and 3rd row)


