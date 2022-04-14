%% In the Name of God
clear;
clc;
load('Gauss_Jordan.mat')


%% Time of Simulation Define

T_final=20;
Step=0.005;
time=0:Step:T_final;
i=1;


%% Trajectory cofficients Define (Trajectory Define in Sperical Coordinate)

% phiJ=Trajectory(1*(pi/180),179*(pi/180),0,0,0,T_final);
% gammaJ=Trajectory(60*(pi/180),45*(pi/180),0,0,0,T_final);

for t=0:Step:T_final
    
        %% Random Trajectory Define
    X0=[90;50]*(pi/180);
    X1=30*sin(t);X2=20*sin(t);
    X=[X1;X2]*(pi/180)+X0;

    dX1=30*cos(t);dX2=20*cos(t);
    X_dot=[dX1;dX2]*(pi/180);

    ddX1=-30*sin(t);ddX2=-20*sin(t);
    X_dot_dot=[ddX1;ddX2]*(pi/180);

%     %% Trajectory Define
%     gamma=gammaJ(1)+gammaJ(2)*t+gammaJ(3)*t^2+gammaJ(4)*t^3;
%     phi=phiJ(1)+phiJ(2)*t+phiJ(3)*t^2+phiJ(4)*t^3;
%     X=[phi;gamma];
%     
%     gamma_dot=gammaJ(2)+2*gammaJ(3)*t+3*gammaJ(4)*t^2;
%     phi_dot=phiJ(2)+2*phiJ(3)*t+3*phiJ(4)*t^2;
%     X_dot=[phi_dot;gamma_dot];  
% 
%     gamma_dot_dot=2*gammaJ(3)+6*gammaJ(4)*t;
%     phi_dot_dot=2*phiJ(3)+6*phiJ(4)*t;
%     X_dot_dot=[phi_dot_dot;gamma_dot_dot];  

    %% Explicit Dynamic
    M=Mass_Matrix(X);
    C=C_Matrix(X,X_dot);
    G=G_Matrix(X);
    
    F=M*X_dot_dot+C*X_dot+G;
    J=Jacobian_Final(X);
    Tau=J'\F;
    
    %% Verification
    X_Plot(i,:)=X;   
    Tau_Plot(i,:)=Tau;

    
    i=i+1;
end

subplot(211)
plot(time', X_Plot(:,2)*(180/pi),'linewidth',2)
grid on

subplot(212)
plot(time',Tau_Plot(:,2),'linewidth',2)
grid on
