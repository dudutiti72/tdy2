function analisi_risultati_GUI(i1,i2,inc)
% Lunch as analisi_risultati_GUI(1,34,1)
% This function writes xls files using the mat file computed by the traindy version
% with igui = 1. The file risultati.xls is in the old matlab input folder. The file risultati.xls
% has to be compared against copia di risultati.xls.
% Example of the old matlab input folder:
% E:\Data\Luciano\Dati_Applicazioni\1103\TrainDy Development\Validation Version 1.1.6\DB validation tests\Freight_1200\TrainDy_017_FILES\Freight1200m-EB-P(017)_V1.5
% i1 is the first index
% i2 is the last
% inc is the index increment

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
    
    path1 = [path_folder 'Projects' fsl Projname '_' TestName fsl,'Test',fsl,TestName];
    appf = ['_',num2str(indiceT)];
    
    % Mat files that store the output results
    fil1 = [path_folder 'Projects' fsl Projname '_' TestName fsl,TestName,'1.mat'];
%     fil0 = [prTD,fsl,TestName,'.mat'];
    load(fil1);
    
    if size(T,2) > size(T,1), T=T'; end
    Flong=Flong*1e-3;
    if size(Flong,1) ~= size(T,1), Flong = Flong'; end
    Fbrake=Fbrake*1e-3;
    veloapp=velo;
    velo = velo*3.6;
    if size(velo,1) ~= size(T,1), velo = velo'; end
    namexls=[prTD,fsl,'risultati.xls'];
    xlswrite(namexls, velo, 'velo', 'B1');
    xlswrite(namexls, T, 'velo', 'A1');
    Time_name{1}='Time';
    Wagon_name{1}='TrainDy1';
    if not(isempty(Flong)),
        xlswrite(namexls, Flong, 'Flong', 'B2');
        xlswrite(namexls, T, 'Flong', 'A2');
        xlswrite(namexls, Time_name(1), 'Flong', 'A1');
        xlswrite(namexls, Wagon_name(1), 'Flong', 'B1');
    end
    xlswrite(namexls, Fbrake, 'Fbrake', 'B2');
    xlswrite(namexls, T, 'Fbrake', 'A2');
    xlswrite(namexls, Time_name(1), 'Fbrake', 'A1');
    xlswrite(namexls, Wagon_name(1), 'Fbrake', 'B1');
    
    scelta=2;
    if scelta == 1
        deltaT=T(2)-T(1);
        spostamenti=veloapp.*deltaT;
        acsissa_curvilinea=0;
        for i=1:size(T)
            acsissa_curvilinea = acsissa_curvilinea + spostamenti(i);
            vettore_posizione(i)=acsissa_curvilinea;
        end
        vettore_posizione=vettore_posizione';
        Run_name{1}='distance';
        xlswrite(namexls, Flong, 'FlongVSrun', 'B2');
        xlswrite(namexls, vettore_posizione, 'FlongVSrun', 'A2');
        xlswrite(namexls, Run_name(1), 'FlongVSrun', 'A1');
        xlswrite(namexls, Wagon_name(1), 'FlongVSrun', 'B1');
        xlswrite(namexls, Fbrake, 'FbrakeVSrun', 'B2');
        xlswrite(namexls, vettore_posizione, 'FbrakeVSrun', 'A2');
        xlswrite(namexls, Run_name(1), 'FbrakeVSrun', 'A1');
        xlswrite(namexls, Wagon_name(1), 'FbrakeVSrun', 'B1');
        
    end
end

