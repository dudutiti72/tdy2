function [F,pos] = interpbgdgf(P,pos,X,x)

front = 0;
while pos < P(1) && x > X(pos+1)
    pos = pos+1; front = 1;
end;
while pos > 1 && x < X(pos) && front == 0
    pos = pos-1;
end;
F = P(2+(pos-1)*4)*x^3+P(3+(pos-1)*4)*x^2+P(4+(pos-1)*4)*x+P(5+(pos-1)*4);
