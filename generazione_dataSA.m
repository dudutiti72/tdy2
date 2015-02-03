function generazione_dataSA
nf = fopen('directory_dataSA.txt','r');
dirtrainpneu='D:\FAIVELEY\TrainPneuP\versioni succesive alla validazione\TrainPneu';
dirtrainday='D:\FAIVELEY\TrainDynamic\NTrainDy';
number_dir_char = fgetl(nf);
number_dir=str2num(number_dir_char);
for indiceSA=1:number_dir
    directory_dataSA = fgetl(nf);
    cd (dirtrainpneu);
    nfpneu=fopen('scambio.inf','w');
    fprintf(nfpneu,'%s\r\n', num2str(directory_dataSA));
    fprintf(nfpneu,'%s\r\n', '          ');
    TrainPneu
    cd(dirtrainday);
end
end