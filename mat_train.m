function trainGUI = mat_train(ii,lsep,nf,position,trainGUI,sep,strtofind,trainfield,col)

% MAT_TRAIN This function is used to extract information from .txt input 
%           files, stored in matrix form. I.e the electrodynamic braking
%           characteristic of a locomotive
%           
% INPUTS    ii:         Number of locomotive or wagon (i.e the 2nd loco has
%                       ii = 2 and is situated in position(ii))
%           lsep:       Number of elements of sep
%           nf:         File identifier of loco or wagon file
%           position:   Positions of locomotives or wagons in the train
%           trainGUI:   Struct array with info about each vehicle
%           sep:        GUI seperator data
%           strtofind:  String to find in .txt file, in order to locate
%                       the requested information
%           trainfield: Name of field where the info will be stored
%           col:        In which column of the trainfield shall the info
%                       be stored
%
% OUTPUT    trainGUI:   Modified struct array


cline = trova_info(strtofind,nf);
ind1  = numel(strtofind) + 1;
ind2  = [findstr(cline,sep) numel(cline) + 1];

for k = 1:numel(ind2)
    info     = str2double(cline(ind1:ind2(k)-1));
    trainGUI = setfield(trainGUI,{position(ii),1},trainfield,{k,col},info);
    ind1     = ind2(k)+lsep;
end

end