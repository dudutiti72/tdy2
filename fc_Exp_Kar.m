%% Comparison among experimental friction coefficient and Karwatzki law
% The comparison is made on a range of speed from 0 to 130 km/h and considering the
% subsequent vector of block forces: [0 10.64 27.75] [kN]
%% DATA
vFdynkN = [0 10.64 27.75]; %[kN] 
vv = linspace(0,140)/3.6; % [m/s];
sB_Bg = 25600; %[mm^2] (Section Bg shoe)
sB_Bgu = 40000; %[mm^2] (Section Bg shoe)
g = 9.81;
% Friction coefficient law
[fc,Pfc] = fc_inp;

%% Calculation of friction coefficient according Karwatzki law
mkarw = zeros(size(vFdynkN,2),size(vv,2));
for jj = 1:size(vFdynkN,2)
    FdynkN = vFdynkN(jj);
    for ii = 1:size(vv,2)
        veloA = vv(ii)*3.6;
        kKarw = 0.6*((((16/g)*FdynkN)+100)/(((80/g)*FdynkN)+100));
        % Friction coefficient according to Karwatzki
        mkarw(jj,ii) =kKarw*((veloA+100)/(5*veloA+100));
    end;
end;

%% Bg Shoe and material "phosphoric iron cast" ("ghisa fosforosa")
vPsp = 10 * 1000*vFdynkN/sB_Bg; % [kg/cm^2]
fcmol = 1.2; % Multiplication factor
y = zeros(2,1); mcoefat = zeros(size(vFdynkN,2),size(vv,2)); 
for jj = 1:size(vFdynkN,2)
    FdynkN = vFdynkN(jj); Psp = vPsp(jj); 
    ilow = 2; iup = 1; posfc = 1; 
    for ii = 1:size(vv,2)
        y(2) = vv(ii);
        [coefat,ilow,iup,posfc] = fricoef_p_v(fc,1,ilow,iup,1,Pfc,posfc,Psp,y);
        % Coefficient of friction according to experimental results.
        mcoefat(jj,ii) = fcmol * coefat;
    end;
end;
figure(1); clf; subplot(2,1,1); hold on; 
h = plot(vv*3.6,mcoefat,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Friction coefficient'); title('Bg Shoe and phosphoric iron cast','fontsize',11);
plot(vv*3.6,mkarw,'--'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');
subplot(2,1,2);
erel = (mkarw-mcoefat)./mcoefat*100;
h = plot(vv*3.6,erel,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Relative Error [%]'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');


%% Bg Shoe and material "iron cast" ("ghisa fosforosa")
mcoefat = mcoefat/1.2;
figure(2); clf; subplot(2,1,1); hold on; 
h = plot(vv*3.6,mcoefat,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Friction coefficient'); title('Bg Shoe and iron cast','fontsize',11);
plot(vv*3.6,mkarw,'--'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');
subplot(2,1,2);
erel = (mkarw-mcoefat)./mcoefat*100;
h = plot(vv*3.6,erel,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Relative Error [%]'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');

%% Bgu Shoe and material "phosphoric iron cast" ("ghisa fosforosa")
vPsp = 10 * 1000*vFdynkN/sB_Bgu; % [kg/cm^2]
fcmol = 1.2; % Multiplication factor
y = zeros(2,1); mcoefat = zeros(size(vFdynkN,2),size(vv,2)); 
for jj = 1:size(vFdynkN,2)
    FdynkN = vFdynkN(jj); Psp = vPsp(jj); 
    ilow = 2; iup = 1; posfc = 1; 
    for ii = 1:size(vv,2)
        y(2) = vv(ii);
        [coefat,ilow,iup,posfc] = fricoef_p_v(fc,1,ilow,iup,1,Pfc,posfc,Psp,y);
        % Coefficient of friction according to experimental results.
        mcoefat(jj,ii) = fcmol * coefat;
    end;
end;
figure(3); clf; subplot(2,1,1); hold on; 
h = plot(vv*3.6,mcoefat,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Friction coefficient'); title('Bg Shoe and phosphoric iron cast','fontsize',11);
plot(vv*3.6,mkarw,'--'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');
subplot(2,1,2);
erel = (mkarw-mcoefat)./mcoefat*100;
h = plot(vv*3.6,erel,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Relative Error [%]'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');

%% Bgu Shoe and material "iron cast" ("ghisa fosforosa")
mcoefat = mcoefat/1.2;
figure(4); clf; subplot(2,1,1); hold on; 
h = plot(vv*3.6,mcoefat,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Friction coefficient'); title('Bg Shoe and iron cast','fontsize',11);
plot(vv*3.6,mkarw,'--'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');
subplot(2,1,2);
erel = (mkarw-mcoefat)./mcoefat*100;
h = plot(vv*3.6,erel,'linewidth',2); xlabel('Speed [km/h]'); ylabel('Relative Error [%]'); grid on;
legend(h,'F_{block} = 0','F_{block} = 10.64','F_{block} = 27.75');
