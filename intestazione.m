function intestazione(a,fid,nomefile,strfl,testo,vnumeric,vstring)

fprintf(fid,'%%# TrainPneu development release \r\n');
fprintf(fid,'%%# Input file name: %s',nomefile);
fprintf(fid,'\r\n');
fprintf(fid,'%%# Data of simulation:');
fprintf(fid,' %02.0f/%02.0f/%4.0f ',a(3),a(2),a(1));
fprintf(fid,' hour of simulation:');
fprintf(fid,' %02.0f:%02.0f:%02.0f\r\n',a(4),a(5),a(6));
fprintf(fid,'%%# Vehicles %s \r\n',vstring);
fprintf(fid,'%%# Data printed :\r\n');
fprintf(fid,testo);
fprintf(fid,'\r\n');
fprintf(fid,'%%');
appo = ['\t ' strfl '%2.0f'];
fprintf(fid,appo,vnumeric);
fprintf(fid,'\r\n');
