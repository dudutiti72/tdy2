function tRTUf2011(igui)
% ____________________________________________________________________________
% GUI Interface
if not(exist('igui','var'))
    % Since it is not possible to pass a variable via Java
    igui = 1; 
end
if nargin < 2
    % appf is a string that is appended to characterize the test it is not supported
    % by the GUI
    appf = '';
    % The file ExecutionTime is used to communicate with the GUI and to understand
    % when the simulation is finished.
    if exist('ExecutionTime.inf','file')
        delete('ExecutionTime.inf'); 
    end;
end; 
timestart = clock; % Starting time of the simulation
% ____________________________________________________________________________

% ____________________________________________________________________________
% Reading appropriate input source file
[fsl,namedirfile,nomesolu,ntrack,nzug,petr,prTD,shw,vel_0] = input_for_computation(appf,igui);
% ____________________________________________________________________________

% Reading input from file
% ____________________________________________________________________________
% igui..0 Traditional; igui..1 GUI input
% In future, igui = 0 will not supported
if igui == 0
    % Reading and reformulation of the imputs
    [indexes,~,Mvt,nveicoli,~,traccia,train] = translate_input(fsl,ntrack,nzug,petr);
else
    [Dcoll,epsilon,indexes,kcoll,Lcoll,~,Mvt,nveicoli,P1,pathGUI,ploco,~,sep,~,SR,...
        traccia,train] = GUI_input(appf,fsl,ntrack,nzug,petr,prTD);
end
% ____________________________________________________________________________

% ____________________________________________________________________________
% Struct variable for the locos
if igui == 0
    loco = car_loco(namedirfile,train);
else 
    loco = car_locoGUI(fsl,pathGUI,ploco,sep,train);
end
% ____________________________________________________________________________

% ____________________________________________________________________________
% Track definition
scartamento = 1.435; 
L = traccia(:,1); R = traccia(:,2); N = traccia(:,3); teta_0 = traccia(:,4); Rv = traccia(:,5); ttratto = traccia(:,6);
[traccia] = defTrack(L,N,R,Rv,scartamento,teta_0,ttratto);
strack = ftrack(traccia,scartamento);
% ____________________________________________________________________________

% ____________________________________________________________________________
% Buffing gears/draw gears coupled characteristic curves
[bgdg,train] = prel_comp(nveicoli,train);
% ____________________________________________________________________________

% ____________________________________________________________________________
% Reading of the pneumatic input
if igui == 1
    [CA,CVactv,D1,DBVdata,dLC,dTF,dx,gmis,gmisCF,...
        Lcoll,Lvago,nveicoli,P1,pCF,SA,train,Tsim,Vbc] = PneumDevices(fsl,...
        Lcoll,P1,pathGUI,sep,train);
else
    % The same function is used also in TrainPneu
    [CA,CVactv,D1,DBVdata,Dcoll,dLC,~,dTF,dx,epsilon,~,gmis,gmisCF,~,kcoll,Lcoll,...
        Lvago,~,~,P1,pCF,~,SA,SR,~,~,train,Tsim,~,~,Vbc,~,~,~] = inp_data2(namedirfile,1,fsl,nveicoli,train);
end
% Reading of the pneumatic input
% From this input data other inputs are computed in order to perform computations.
% This is done to keep the compatibility with TrainPneu
[CA,dQdto,d2Qdt2o,drodto,d2rodt2o,DT,dt,dudto,d2udt2o,D,EOT,FOT,iCV,iV,j0,j1,jj,K,...
    ~,Lmedia,n,nCV,nDBV,~,~,rugrel,SA,segnoCA,segnoSA,SR,typeDBV,~,vmisCF,vUnC] = comp_data(CA,...
    CVactv,D1,DBVdata,Dcoll,dx,epsilon,gmis,gmisCF,kcoll,Lcoll,Lvago,nveicoli,SA,SR,train,Tsim);
Pbrake = []; swcf = []; Tbrake = []; Ppres = []; % Old variable for backword compatibility
% ____________________________________________________________________________

% ____________________________________________________________________________
% Computation
solver = 'ode15s';
integraode(appf,bgdg,indexes,Mvt,Pbrake,Ppres,shw,strack,swcf,Tbrake,traccia,train,vel_0,...
    CA,CVactv,D,DBVdata,dLC,DT,dt,dTF,dQdto,d2Qdt2o,drodto,d2rodt2o,dudto,d2udt2o,dx,EOT,FOT,...
    iCV,iV,j0,j1,jj,K,Lmedia,loco,n,nCV,nDBV,P1,pCF,rugrel,segnoCA,segnoSA,...
    SA,solver,SR,typeDBV,Vbc,vmisCF,vUnC);
% ____________________________________________________________________________

% ____________________________________________________________________________
% From the solution the data do be handled by the GUI are computed
visres(nomesolu,appf);
% ____________________________________________________________________________

% ____________________________________________________________________________
% Writing the file that stores the execution time: communication with the GUI
if nargin < 2,
    nf = fopen('ExecutionTime.inf','w');
    fprintf(nf,'The execution time is %g',etime(clock,timestart));
    fclose(nf);
end
% ____________________________________________________________________________

end

