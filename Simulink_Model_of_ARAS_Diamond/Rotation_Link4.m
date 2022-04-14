function [R] = untitled3(a,b,c,d)


C=cross(c,d);
D=cross(C,c);

R=[D/norm(D),C/norm(C),c];

end

