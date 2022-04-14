function [C] = untitled8(X,X_dot,Ibar1,Ibar2,Ibar3,Ibar4)


    %% Attend to this consideration!

    phi=X(1);
    gamma=X(2);

    phi_dot=X_dot(1);
    gamma_dot=X_dot(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Geometry Define

alpha=pi/4;
beta=pi/4;


% %% Inertial Parameters
%     m_1 = mass(1);
%     m_2 = mass(2);
%     m_3 = mass(3);
%     m_4 = mass(4); 



    %% Parallel Axis Theorem
    IA1=Moment_Inertia_Creator(Ibar1);
    IA2=Moment_Inertia_Creator(Ibar2);
    IA3=Moment_Inertia_Creator(Ibar3);
    IA4=Moment_Inertia_Creator(Ibar4);
    
    %% Inverse Kinematic
    [theta_New,Passive]=Inverse_Kinematic(gamma,phi,alpha,beta);
    theta__1=theta_New(1);  theta__2=theta_New(2);    
    A=Passive(1);B=Passive(2);D=Passive(3);
    
    %% Calculating Unit Vectors
    Unit= Unit_Vectors(theta__1,theta__2,alpha,beta,gamma,phi);
    a=Unit(1:3);b=Unit(4:6);c=Unit(7:9);d=Unit(10:12);
    
        %% Rotation Matrix 
    R1=Rotation_Link1(a,b,c,d); R2=Rotation_Link2(a,b,c,d); R3=Rotation_Link3(a,b,c,d); R4=Rotation_Link4(a,b,c,d);

    %% Jacobian Analysis of End_Effector:
    J= Jacobian(alpha,beta,gamma,A);             %Scalar approach 
    q_dot=J*X_dot;
    theta__1_dot=q_dot(1);  theta__2_dot=q_dot(2);   

    %% Jacobain of Links
    J1=Jacobian_Link1(alpha,beta,gamma,A,a,b,c,d);
    J2=Jacobian_Link2(alpha,beta,gamma,A,a,b,c,d);
    J3=Jacobian_Link3(alpha,beta,gamma,A,B,a,b,c,d);
    J4=Jacobian_Link4(alpha,beta,gamma,A,B,a,b,c,d);
      
      %% Acceleration_Analysis
    
    J1dot=Jacobian_dot_Link1(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
    J2dot=Jacobian_dot_Link2(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
    J3dot=Jacobian_dot_Link3(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J1dot,theta__2_dot);
    J4dot=Jacobian_dot_Link4(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J2dot,theta__1_dot);
    
    %% Moment Inertia Mapping
    I1=R1*IA1*R1';
    I2=R2*IA2*R2';
    I3=R3*IA3*R3';
    I4=R4*IA4*R4';   

    % 	%% Conver Local rho to Base rho
%     Rho1=R1*Skew(rho1)*R1';
%     Rho2=R2*Skew(rho2)*R2';
%     Rho3=R3*Skew(rho3)*R3';
%     Rho4=R4*Skew(rho4)*R4';      
%     %% Mass Matrix
%     M1=J1'*I1*J1;
%     M2=J2'*I2*J2;
%     M3=J3'*I3*J3;
%     M4=J4'*I4*J4;  
%     M = M1 + M2 + M3 + M4 ;
    %% C Matrix
    C1= J1'*I1*J1dot+J1'*Skew(J1*X_dot)*I1*J1;    
    C2= J2'*I2*J2dot+J2'*Skew(J2*X_dot)*I2*J2;
    C3= J3'*I3*J3dot+J3'*Skew(J3*X_dot)*I3*J3;
    C4= J4'*I4*J4dot+J4'*Skew(J4*X_dot)*I4*J4;
    C = C1 + C2 + C3 + C4 ; 
%     %% Gravity Vectors ( Rho1>> Base Measured ) 
%     G1=-m_1*J1'*Rho1*g;  
%     G2=-m_2*J2'*Rho2*g;  
%     G3=-m_3*J3'*Rho3*g;  
%     G4=-m_4*J4'*Rho4*g;  
%     G = G1 + G2 + G3 + G4; 
% 
%     %% Dynamic Equation
% 
%     
%     F_Dynamic   =  (M)*X_dot_dot + C*X_dot +G;
%     Tau_Dynamic = J'\F_Dynamic;
%     
 
end

