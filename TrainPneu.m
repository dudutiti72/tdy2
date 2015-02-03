%% TRAIN BRAKE PNEUMATICS "TrainPneu development release"
% This code solves the basic fluyd dynamics equation by applying them to a pipe with
% variable transversal section. It solves the pneumatic problem of a train main brake
% pipe considering braking and release. It simulates distributors (accelerating
% chambers), main and auxiliary reservoirs and so on.

%% VARIABLE DECLARATION AND INPUT DATA

%# scalar c CVactv dt dcc dimens Dpy dt EOT FOT ii iv k Ltubo n nCV nSR nsim nveicoli P1 pm R SR Tsim
%# scalar u1 fid siz

% Reading of input data from txt files
  [CA,CVactv,D1,DBVdata,Dcoll,dLC,Dpy,dTF,dx,epsilon,fsl,gmis,gmisCF,iDCVa,kcoll,Lcoll,...
          Lvago,nomefilese,nomepath,nveicoli,P1,pCF,SA,SR,t0,Tsim,tvis,Vbc,VhcM,...
          VhcMa,vManovra] = inp_data2;
    
%% COMPUTED DATA
% From this input data other inputs are computed in order to perform computations.
[CA,dQdto,d2Qdt2o,drodto,d2rodt2o,DT,dt,dudto,d2udt2o,D,EOT,FOT,iCV,iV,j0,j1,jj,K,...
    L,Lmedia,n,nCV,nDBV,nsim,pmis,rugrel,SA,segnoCA,segnoSA,SR,typeDBV,vmis,vmisCF] = comp_data(CA,...
    CVactv,D1,DBVdata,Dcoll,dx,epsilon,gmis,gmisCF,kcoll,Lcoll,Lvago,nveicoli,SA,SR,Tsim);

% K(37:38) = K(37:38)*30;
% COMPUTATION PERFORMING
[PbcM,PbcT,PM,PMT,PsaM,roM,tempo,tempoT,TM,vdtmed,uM] = prescg2(CA,CVactv,D,...
    DBVdata,dLC,Dpy,DT,dt,dTF,dQdto,d2Qdt2o,drodto,d2rodt2o,dudto,d2udt2o,dx,EOT,FOT,...
    iCV,iDCVa,iV,j0,j1,jj,K,L,Lmedia,n,nCV,nDBV,nsim,P1,pCF,rugrel,segnoCA,segnoSA,...
    SA,SR,typeDBV,tvis,Vbc,VhcMa,vManovra,vmis,vmisCF);

tf = etime(clock,t0);

disp(sprintf('Computational time %g, Simulated time %g; realtime ratio %g',...
	tf,Tsim,tf/Tsim));

if Dpy
    figure(13); clf;
    plot(tempo,uM); grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('Velocity [m/s]');
    grid on
    zoom on
    figure(14); clf;
    plot(tempo,roM); grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('Density [kg/m^3]');
    grid on
    zoom on
    figure(15); clf;
    plot(tempo,TM); grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('Temperature [K]');
    grid on
    zoom on

    if CVactv
        figure(16); clf;
        plot(tempoT,(PM-1e5)/1e5); grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('BP/BC pressure [bar]');
        grid on
        zoom on
        hold on
        plot(tempoT,PbcM); %grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('Brake cylinder pressure [bar]');
        grid on
        zoom on
        figure(17); clf;
        plot(tempo,PsaM); grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('Auxiliary reservoir pressure [bar]');
        grid on
        zoom on
    else
        figure(16); clf;
        plot(tempo,(PM-1e5)/1e5); grid on; zoom on; set(gca,'xlim',[0 Tsim]); xlabel('Time [s]'); ylabel('Brake Pipe pressure [bar]');
        grid on
        zoom on
    end;
end;

%======================================================================
% DATA PRINTING
% if CVactv
Szvmis = size(vmis,2);
%#RK-modified rfileout = [nomepath,'DataT\',nomefilese];
% rfileout = [nomepath,'DataT/',nomefilese];
rfileout = [nomepath,'DataT',fsl,nomefilese];
% Tempo visualizzato nel file di output_testo
a = clock;
%# fastindex
dimens = 20;
%# fastindex
dcc = floor(Szvmis/dimens);
disp(' ');
disp(sprintf('Log file writing...'));
stampaBP(a,dcc,dimens,nomefilese,Szvmis,((PM-1e5)/1e5),rfileout,tempoT',VhcM); %pressure in brake pipe
stampaVel(a,dcc,dimens,nomefilese,Szvmis,uM,rfileout,tempo',VhcM); %velocity along brake pipe
stampaRo(a,dcc,dimens,nomefilese,Szvmis,roM,rfileout,tempo',VhcM); %density along brake pipe
stampaTmp(a,dcc,dimens,nomefilese,Szvmis,TM,rfileout,tempo',VhcM); %temperature along brake pipe
if CVactv
    stampaBC(a,dcc,dimens,nomefilese,Szvmis,PbcM,rfileout,tempoT',VhcM); % pressure in brake cylinder
    stampaSA(fsl,nomepath,PbcT,tempoT);
end
disp('   ...done');
disp('');
% end;

