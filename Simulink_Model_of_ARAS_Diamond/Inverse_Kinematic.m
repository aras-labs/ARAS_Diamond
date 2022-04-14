function [theta,Passive] = untitled2(gamma,phi,alpha,beta)

theta__1=phi+acos((cos(beta)-cos(gamma)*cos(alpha))/(sin(gamma)*sin(alpha)));
theta__2=phi-acos((cos(beta)-cos(gamma)*cos(alpha))/(sin(gamma)*sin(alpha)));

theta=[theta__1;theta__2];


A=0.5*(theta__1-theta__2);
B=acos((cos(gamma)-cos(beta)*cos(alpha))/(sin(beta)*sin(alpha)));
D=((cos(alpha)-cos(gamma)*cos(beta))/(sin(gamma)*sin(beta)));

Passive=[A;B;D];

end

