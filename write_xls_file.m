function write_xls_file

[fsl,ntrack,nzug,petr,prTD,shw,vel_0,batch_flag] = inp_TDET;
name_start = findstr(prTD,fsl);
appo = [prTD '\'];
nomesolu = [appo prTD(name_start(end)+1:end) '.mat'];
analisi_risultati(nomesolu);
end