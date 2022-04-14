function [R] = untitled3(a,b,c,d)


C=cross(a,c);
D=cross(C,a);

R=[D/norm(D),C/norm(C),a];

end

