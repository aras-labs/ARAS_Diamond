clear;
clc;


%% CAD Data
%Theta=Inertial_Parameters_Full;

%% Identification Data
load('LS_Bounded_Result_Data.mat') 
Theta=Theta_Initial;

%% Initial Condition for Simulation
X0=[90;50]*(pi/180); 
dX0= [30; 20]*(pi/180);

