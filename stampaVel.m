function stampaVel(a,dcc,dimens,nomefile,nveicoli,Velc,rfileout,temp,VhcM)

%# scalar dcc dimens
%# scalar c fid fidc ii i1 i2 siz

c = 0; % Counter of file writed file txt
%# fastindex
VhG = num2str(VhcM);
testo = ('%%# Time| Velocity in BP |'); strfl = 'V';
for ii = 1:dcc
	c = c+1;
	fileout = [rfileout,'_VL',num2str((ii-1)*dimens+1,'%02i'),'-',num2str(ii*dimens,'%02i'),'.txt'];
	fileoutc = [rfileout,'_VL',num2str((ii-1)*dimens+1,'%02i'),'-',num2str(ii*dimens,'%02i'),'c.txt'];
	fid = fopen(fileout,'w');
	fidc = fopen(fileoutc,'w');
	
	
    intestazione(a,fid,nomefile,strfl,testo,VhcM,VhG);
    intestazione(a,fidc,nomefile,strfl,testo,VhcM,VhG);
	
	fdata = '%3.3f';
	i1 = (ii-1)*dimens+1;
	i2 = ii*dimens;
	siz = i2-i1+1;
	scrividati(Velc,fdata,fid,i1,i2,siz,temp);
    scrividatic(Velc,fdata,fidc,i1,i2,siz,temp);

end;

if nveicoli-dimens*dcc > 0
    % Further data to printing
    fileout=[rfileout,'_VL',num2str(c*dimens+1,'%02i'),'-',num2str(nveicoli,'%02i'),'.txt'];
    fileoutc=[rfileout,'_VL',num2str(c*dimens+1,'%02i'),'-',num2str(nveicoli,'%02i'),'c.txt'];
	fid = fopen(fileout,'w');
	fidc = fopen(fileoutc,'w');
	
    intestazione(a,fid,nomefile,strfl,testo,VhcM,VhG);
    intestazione(a,fidc,nomefile,strfl,testo,VhcM,VhG);
	
	fdata = '%3.3f';
	i1 = c*dimens+1;
	i2 = nveicoli;
	siz = i2-i1+1;
	scrividati(Velc,fdata,fid,i1,i2,siz,temp);
    scrividatic(Velc,fdata,fidc,i1,i2,siz,temp);
end;
