function trainGUI = mat_train(ii,lsep,nf,position,trainGUI,sep,strtofind,trainfield,col)

cline = trova_info(strtofind,nf);
ind1 = numel(strtofind)+1; ind2 = [findstr(cline,sep) numel(cline)+1];
for k = 1:numel(ind2)
    info = str2double(cline(ind1:ind2(k)-1));
    trainGUI = setfield(trainGUI,{position(ii),1},trainfield,{k,col},info );
    ind1 = ind2(k)+lsep;
end

end