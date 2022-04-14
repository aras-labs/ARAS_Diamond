function [J1] = untitled2(alpha,beta,gamma,A,a,b,c,d)


K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
J1=[a,-K*a];

end

