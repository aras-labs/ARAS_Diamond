function [J1dot] = untitled(alpha,beta,gamma,A,a,b,c,d,gamma_dot)

G=gamma;
zero=[0;0;0];
K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
R=(((-cos(alpha) * cos(G) ^ 2 + cos(alpha)) * cos(A) - sin(G) * sin(alpha) * cos(G)) * K - sin(A) * cos(A) * sin(alpha)) / sin(G) ^ 2 / sin(alpha) / sin(A) ^ 2;

J1dot=[zero,-R*gamma_dot*a];



end

