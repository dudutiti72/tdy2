function automatic_version_verification(i1,i2,inc,vd)
% Lunch as automatic_version_verification(1,34,1,0)
% This function creates two mat files, one with igui = 0 and igui = 1
% igui = 0 launches the simulation with the old input of MATLAB
% igui = 1 launches the simulation with the new input of the GUI
% The matlab files launched with igui = 0 are in the folder like this
% E:\Data\Luciano\Dati_Applicazioni\1103\TrainDy Development\Validation Version 1.1.6\DB validation tests\Freight_1200\TrainDy_017_FILES\Freight1200m-EB-P(017)_V1.5
% The corresponding file created with igui = 1 is here
% E:\Data\Luciano\Dati_Applicazioni\1103\TrainDy Development\Validation Version 1.1.6\Projects\Freight_1200_Freight1200m-EB-P(017)_V1.5
% i1 is the first index
% i2 is the last
% inc is the index increment
% vd is a flag to display the results: vd = 2, do not make computation; vd = 0 makes
% computation and plot the results; vd = 1 is not supported

nfa = fopen('elenco_TDET.txt','r');
path_folder = fgetl(nfa);
conta = 0;
for indiceT=i1:inc:i2
    while conta < indiceT
        conta = conta+1;
        fgetl(nfa);
        riga1= fgetl(nfa);
        riga2= fgetl(nfa);
        riga3= fgetl(nfa);
        riga4= fgetl(nfa);
        riga5= fgetl(nfa);
        riga6= fgetl(nfa);
    end
    path1=[path_folder num2str(riga1)];
    path2=riga2;
    path4=riga4;
    if vd < 2
        nfT=fopen('TDET.inf','w');
        fprintf(nfT,'%s\r\n', 'WINDOWS');
        %fprintf(nfT,'%s\r\n', path1);
        fprintf(nfT,'%s', path1);
        fprintf(nfT,'%s\r\n', path2);
        fprintf(nfT,'%s\r\n', riga3);
        fprintf(nfT,'%s\r\n', path4);
        fprintf(nfT,'%s\r\n', num2str(riga5));
        fprintf(nfT,'%s\r\n', num2str(riga6));
        fprintf(nfT,'Batch\r\n');
        fprintf(nfT,'%s\r\n', '          ');
        fclose(nfT);
        
        
        TrainDyToGUI;
    end
    
    %[fsl,~,~,petr,prTD,~,~] = inp_TDET;
    fsl = '\';
    petr = [path1,path2]; prTD = [petr,fsl,riga3];
    appo = strfind(petr,[fsl,'TrainDy_Files_']);
    if isempty(appo)
        appo = strfind(petr,fsl);
        Projname = petr(appo(end)+1:end);
    else
        Projname = petr(appo(end)+15:end);
    end
    
    appo = strfind(prTD,fsl);
    i1 = strfind(prTD(appo(end):end),[fsl 'TrainDy_']);
    i2 = strfind(prTD(appo(end):end),'_FILES');
    TestName = prTD(appo(end)+8+i1:appo(end)+i2-2);
    if isempty(TestName)
        TestName = prTD(appo(end)+1:end);
    end
    
    %path1=[path_folder,'Projects\',Proj,'\','Test\',test];
    path1 = [path_folder 'Projects' fsl Projname '_' TestName fsl,'Test',fsl,TestName];
    %appf = ['_',num2str(indiceT)];
    appf = '';
    if vd < 2
        % It means that the projects have to be launched again
        nfT=fopen(['GUI',appf,'.inf'],'w');
        fprintf(nfT,'%s\r\n', [path1 '.txt']);
        fprintf(nfT,'%s\r\n', '          ');
        fclose(nfT);
        tRTUf2011(1);
        tRTUf2011(0);
    end
    % Mat files that store the output results
    fil1 = [path_folder 'Projects' fsl Projname '_' TestName fsl,TestName,'1.mat'];
    fil0 = [prTD,fsl,TestName,'.mat'];
    %fil0 = [path_folder,path2(1:strfind(path2,'\')),fsl,TestName,'0.mat'];
    if vd == 1
        visdiff(fil1,fil0);
    else
        load(fil1,'T','Flong');
        T1 = T; Flong1 = Flong;
        load(fil0,'T','Flong');
        T0 = T; Flong0 = Flong;
        figure(1); clf; plot(T1,Flong1);
        figure(10); clf; plot(T0,Flong0); 
%         pause
    end
end
end