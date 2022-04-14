function [J] = Jacobian_Final(X)


phi=X(1);
gamma=X(2);

%% Geometry Define

alpha=pi/4;
beta=pi/4;

%% Inverse Kinematic
[theta_New,Passive]=Inverse_Kinematic(gamma,phi,alpha,beta);
theta__1=theta_New(1);
theta__2=theta_New(2);   

A=Passive(1);
B=Passive(2);
D=Passive(3);


K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
J=[1,K;1,-K];

end

