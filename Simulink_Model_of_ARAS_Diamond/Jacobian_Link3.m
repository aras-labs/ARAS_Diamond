function [J3] = untitled2(alpha,beta,gamma,A,B,a,b,c,d)



K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
Q=((cos(gamma)*sin(A)+sin(gamma)*K*cos(A)))/((cos(B)*sin(beta)));


J3=[a,-K*a-Q*b];

end

