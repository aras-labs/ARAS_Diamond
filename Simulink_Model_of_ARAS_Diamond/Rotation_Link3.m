function [R] = untitled3(a,b,c,d)


C=cross(b,d);
D=cross(C,b);

R=[D/norm(D),C/norm(C),b];

end

