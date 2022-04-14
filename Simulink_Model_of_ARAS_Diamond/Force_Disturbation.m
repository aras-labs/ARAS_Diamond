function [Tau] = untitled7(X,F)


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

   
    %% Jacobian Analysis of End_Effector:
    J= Jacobian(alpha,beta,gamma,A); 

    %% Force Distubation

Tau=J'\F;


end

