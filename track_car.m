function [C,fi,ipos,psi] = track_car(ii,ipos,lwagcs,strack,traccia,y)

% TRACK_CAR This function computes the geometric input for the vehicle ii 
%                that is on the 'ipos(ii)' part of the track
%
% INPUTS    ii:      vehicle in question
%           ipos:    Track section id where each vehicle is   
%           lwagcs:  Cumulative distance between COGs
%           strack:  Struct array with track info
%           traccia: Array with track info 
%           y:       First half of this vector includes the position of 
%                    each vehicle and second half the velocities
%
% OUTPTUS    C:      Curvature at track section where the vehicle is
%            fi:     Slope at current track section
%            ipos:   Updated location on track for vehicle ii
%            psi:    Parameters of tangent at current track section

% Position on the track: at the beginning the abscissa of rear vehicles is negative
s    = y(ii) - lwagcs(ii); 
sloc = s-strack(ipos(ii)).l; % Local abcsissa, refferred to the section of track

if s > strack(ipos(ii)+1).l
    ipos(ii) = ipos(ii)+1; % Updating of the local piece of track
    sloc     = s-strack(ipos(ii)).l;
end
c = ipos(ii); % To shorten the notation
switch strack(c).t
    case 1
        % Straight track: everything is fixed
        psi = strack(c).psi;
        fi  = strack(c).fi;
        C   = 0;
    case 2
        % Full curve: the psi angle changes with the linear abscissa
        psi = strack(c).psi - sloc*strack(c).c;
        fi  = strack(c).fi;
        C   = strack(c).c;
    case 3
        % Transition curve: there are several sub cases
        % The slope is changed linearly
        fi = strack(c).fi + (strack(c+1).fi - strack(c).fi)/(strack(c+1).l-strack(c).l)*sloc;
        if strack(c).cv == 0
            % The psi angle is changed differently by using the clothoid formula and
            % considering different sub cases
            if strack(c-1).c == 0 % Previous track is a straight line
                tloc = sloc/strack(c).B;
                psi  = strack(c).psi - strack(c).sx*0.5*pi*tloc^2;
                C    = pi*tloc/strack(c).B*sign(strack(c+1).c);
            elseif strack(c+1).c == 0 % Following track is a straight line
                tloc = (traccia(c,1)-sloc)/strack(c).B;
                tfin = traccia(c,1)/strack(c).B;
                psi  = strack(c).psi - strack(c).sx*0.5*pi*(tfin^2-tloc^2);
            elseif strack(c-1).c ~= 0 && strack(c+1).c ~= 0 % The clothoid is between two full curves
                C1   = strack(c-1).c;
                B    = strack(c).B;
                sini = B^2*C1/pi;
                tloc = (sini+strack(c).s*sloc)/B;
                C    = pi*tloc/strack(c).B*sign(strack(c-1).c);
                tin  = sini/B;
                psi  = strack(c).psi - strack(c).s*0.5*pi*(tloc^2-tin^2);
            end
        else
            C   = 0; 
            psi = strack(c).psi;
        end
end



