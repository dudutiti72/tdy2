function [traccia] = defTrack(L,N,R,Rv,scartamento,teta_0,ttratto)

% DEFTRACK Function that manipulates track info to build the array traccia
%          used in the computations
%
% INPUTS   L:           Length of each track section
%          N:           Slope  of each track section  [‰]
%          R:           Curvature radius of each track section
%          Rv:          Vertical  radius of each track section. Used when
%                       two stretches have different slopes
%          scartamento: Gauge [m]
%          teta_0       Difference between inner and outer rail [cm]
%          ttratto:     Vector with information about track ++
%
% OUTPUT   traccia      Array with track info where each row reffers to a 
%                       different section

N = N/1000; % [‰] (ALT+0137)

% teta is the rail roll angle [rad]
teta = asin(teta_0'/(scartamento*100))'; 

% ttratto provides information about the type of stretch: 
% 1 straight
% 2 full curve
% 3 variable curve radius (parabolic)

% Copies of the above variables
L2  = L;
R2  = zeros(size(L));
Rv2 = Rv;
N2  = N;

teta2    = teta;
ttratto2 = ttratto;

% Automatic insertion of parabolic stretchs
jj = 0;
for k = 2:length(L)
    appo = 0;
    if ttratto(k) == 1 && ttratto(k-1) == 1 && N(k) ~= N(k-1) 
        % There are two consecutive stretches and they have different slope
        appo    = 1;
        tipopar = 2;                                                        
    elseif ((ttratto(k) == 1 && ttratto(k-1) == 2) || ...
            (ttratto(k) == 2 && ttratto(k-1) == 1) || ...
            (ttratto(k) == 2 && ttratto(k-1) == 2)) && ((teta_0(k) ~= teta_0(k-1)) || N(k) ~= N(k-1))
        if (teta_0(k) ~= teta_0(k-1))
            % There are two consecutive stretches, where one is straight and
            % the other is full curve or both are full curves but with
            % different superelevation
            appo    = 2;
            tipopar = 1;
        elseif N(k) ~= N(k-1)
            % There are two consecutive stretches, where one is straight and
            % the other is full curve or both are full curves but with
            % different slope
            appo = 1;
        end
    end
    if appo ~= 0
        if appo == 1
            % Parabolic stretch among two straight lines
            fi0 = N(k-1);
            fi1 = N(k);
            % Slope difference to be filled by the parabolic stretch
            deltafi = abs(fi1-fi0);
            % By imposing a curvature radius of 3000 m, it is possible to
            % determine the length of this new parabolic stretch
			dRv = 3000; % [m]
			s12 = deltafi*dRv;
            % The minimum length of a stretch is fixed to   1 m
			if s12 < 1, s12 = 1; end;
            % We would like to avoid a too much long parabolic stretch
			if s12 > (L(k-1)+L(k))*0.1, s12 = (L(k-1)+L(k))*0.1; end;

			% Updating existing stretche
			L2(jj+k-1)    = L(k-1)-s12*(L(k-1)/(L(k-1)+L(k)));
			L2(jj+k+1)    = L(k)-s12*(L(k)/(L(k-1)+L(k)));
            R2(jj+k-1)    = R(k-1);
            R2(jj+k+1)    = R(k);
            Rv2(jj+k-1)   = Rv2(k-1);
            Rv2(jj+k+1)   = Rv2(k);
            teta2(jj+k-1) = teta(k-1);
            teta2(jj+k+1) = teta(k);
            N2(jj+k-1)    = N(k-1);
            N2(jj+k+1)    = N(k);
            
            ttratto2(jj+k-1) = ttratto(k-1);
            ttratto2(jj+k+1) = ttratto(k);
            
            L(k-1) = L2(jj+k-1);
            L(k)   = L2(jj+k+1);
            
            % Adding the new stretch
            L2(jj+k)    = s12;
            R2(jj+k)    = 0;
            Rv2(jj+k)   = s12/(fi1-fi0);
            teta2(jj+k) = 0;
            N2(jj+k)    = 0;
            
            ttratto2(jj+k) = 3;
            
            jj = jj + 1; 
            
        elseif appo == 2
            
            % The parabolic stretch is among a straight and a full curve
            % stretch
            % It is assumed a rising of the outer rail of 2 mm per 1 m of
            % length
			incrSpv = 2; %[mm]
            
            % Difference in cm among the outer and inner rail
            Spv1 = teta_0(k-1);           
            Spv2 = teta_0(k);
            
            deltaSpv = (abs(Spv1-Spv2))*10; % [mm]      
            s12      = deltaSpv/incrSpv;    % [m]
            
            if (teta_0(k) == teta_0(k-1)) && (N(k) ~= N(k-1))
                fi0 = N(k-1); fi1 = N(k);
                % The stretches are strainght
                deltafi = (fi1-fi0);
                % By imposing a curvature radius of 3000 m, it is possible to
                % determine the length of this new parabolic stretch
                dRv = 3000*sign(deltafi); % [m]
                s12 = deltafi*dRv;
                % The minimum length of a stretch is fixed to   1 m
                if s12 < 1, s12 = 1; end;
            else dRv = 0;
            end
            % The two stretches have a too high difference of
            % superelevation
            if s12 > L(k-1)+L(k)
                error('ERROR It was not possible to insert a parabolic stretch!!!');
            end
            
            % Updating of existing stretches
            L2(jj+k-1)    = L(k-1)-s12*(L(k-1)/(L(k-1)+L(k)));
            L2(jj+k+1)    = L(k)-s12*(L(k)/(L(k-1)+L(k)));
            R2(jj+k-1)    = R(k-1);
            R2(jj+k+1)    = R(k);
            Rv2(jj+k-1)   = Rv2(k-1);
            Rv2(jj+k+1)   = Rv2(k);
            teta2(jj+k-1) = teta(k-1);
            teta2(jj+k+1) = teta(k);
            N2(jj+k-1)    = N(k-1);
            N2(jj+k+1)    = N(k);
            
            ttratto2(jj+k-1) = ttratto(k-1);
            ttratto2(jj+k+1) = ttratto(k);
            
            L(k-1) = L2(jj+k-1);
            L(k)   = L2(jj+k+1);
            
            % Adding the new parabolic stretch
            L2(jj+k)    = s12;
            R2(jj+k)    = 0;
            Rv2(jj+k)   = dRv;
            teta2(jj+k) = 0;
            N2(jj+k)    = 0;
            
            ttratto2(jj+k) = 3;
            
            jj = jj + 1;
            
        end
    else
        L2(jj+k)    = L(k);
        R2(jj+k)    = R(k);
        Rv2(jj+k)   = R2(k);
        teta2(jj+k) = teta(k);        
        N2(jj+k)    = N(k);
        
        ttratto2(jj+k) = ttratto(k);
    end
end
            
traccia(:,1) = [L2;L2(end)];
traccia(:,2) = [R2;R2(end)];
traccia(:,3) = [N2;N2(end)];
traccia(:,4) = [teta2;teta2(end)];  
traccia(:,5) = [Rv2;Rv2(end)];
traccia(:,6) = [ttratto2;ttratto2(end)];
