%% Initialize system parameters
close all;clear all;clc;

%% Values
% Comes from Autodesk Vision 360
lp = 0.331; %m
lw = 0.369; %m
mp = 0.533; %kg
mw = 0.107; %kg
Ip = 8.049E-3; %kg.m^2 MoIxx about COG
Iw = 8.06E-5;  %kg.m^2 MoIyy wheel
g=9.81; %m.s^-2
Km=3.7E-2; %N.m/A (motor torque constant)
Tmax=1.7e-1; %N.m
Imax=4.4; %A
%tf
s=tf('s');
%% Define the state-space model [A,B,C,D]
A=[0 1 0;(mp*lp+mw*lw)*g/(mp*lp^2+mw*lw^2+Ip) 0 0;(mp*lp+mw*lw)*g/(-(mp*lp^2+mw*lw^2+Ip)) 0 0]; %state matrix
B=[0; -Km/(mp*lp^2+mw*lw^2+Ip); Km*(mp*lp^2+mw*lw^2+Ip+Iw)/(Iw*(mp*lp^2+mw*lw^2+Ip))]; %input matrix
C=eye(3); %output matrix (identity 3x3)
D=zeros(3,1,'uint32'); %feedthrough matrix equal 0
x0=[pi/30;0;0]; %initial condition
y0=[-pi/30;0;0]; %initial condition

%% Create state-space model for MATLAB (Matrix A,B,C,D are equal to above)
disp('State-space model')
sys=ss(A,B,C,D,'InputName','u', 'OutputName',{'thetap', 'thetapdot', 'thetawdot'},'StateName',{'thetap', 'thetapdot', 'thetawdot'})

%% Calculation about A matrix
disp('T : matrix of the eigenvectors of A')
disp('Ahat : matrix of the eigenvalues of A')
% Returns diagonal matrix Ahat of eigenvalues and matrix T whose columns are the corresponding right eigenvectors
[T,Ahat]=eig(A); %T defines the eigenvector & Ahat the eigenvalues
syms x
Charpoly=charpoly(A,x); %characteristic polynomial of matrix A
solve(Charpoly == 0,x); %det(xI-A)=0

%% State-space model to transfer function 
disp('State-space model to transfer function')
H=tf(sys) %Transfer function sys
H_zpk=zpk(sys) %conversion to Zero-Pole-Gain form
size(H) %define the number of inputs and outputs
pzplot(H(1,1),'m',H(2,1),'b',H(3,1),'r')
h=findobj(gca,'type','line');
set(h,'markersize',10,'linewidth',1)
grid on
axis equal

%% Verify the controllability and observability
%Compute controllability matrix
Co=ctrb(A,B);
%Compute observability matrix
Ob=obsv(A,C);
%Controllability (Kalman's criterion)
disp('Rank of Co')
Rco=rank(Co)
if rank(Co) == size(A,1)
    disp('It is controllable')
else
   disp('It is not controllable')
end
%Observability (Kalman's criterion)
disp('Rank of Ob')
Rob=rank(Ob)
if rank(Ob) == size(A,1)
     disp('It is observable')
else
    disp('It is not observable')
end

