function numline = gotonumeric(nf)

while 1
    cline = fgetl(nf);
    if ~ischar(cline)
        break; 
    end
    numline = str2num(cline);
    if not(isempty(numline)), break; end;
end

end