function cline = trova_info(strdatrovare,nf)

nrew = 0; trovato = 0;
while nrew < 2
    while trovato == 0
        cline = fgetl(nf);
        if ~ischar(cline)
            trovato = 1;
        end;
        if not(isempty(strfind(cline,strdatrovare))) && cline(1) == strdatrovare(1)
            trovato = 1; nrew = 3;
        end;
    end
    if ~ischar(cline)
        frewind(nf);
        nrew = nrew + 1; trovato = 0;
    end
end
if nrew == 2
    cline = [];
    %error('Sorry, unable to find Your string :-(');
end