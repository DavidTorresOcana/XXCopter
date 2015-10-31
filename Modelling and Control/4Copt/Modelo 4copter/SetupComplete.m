%Setup of all the parameters of the 4copter
% clear all
close all
clc

addpath(genpath('ESC+Motor Model'));
%% General parameters
k_quat=100;  % High gain Quaternion Normalization
r_sens = [0,0,0.03]' ; % Position of sensors in BA
%% Mass properties, momentos de inercia y longitudes
m=3.51; % Mass   % To be changed/measured/tuned
CG= [0,0,0.02]; % 2cm belob BAC  % To be changed/measured/tuned
M_eq=0.001; % Equivalent mass of the proppeller ~1/5*mass_propeller  % To be changed/measured/tuned

Ix=0.0552; % To be changed/measured/tuned
Iy=0.0552; % To be changed/measured/tuned
Iz=0.11;   % To be changed/measured/tuned

I_BAC= diag( [Ix,Iy,Iz]);

%% Props positions  
%           1
%           |
%       4---o---2
%           |
%           3
Rotation_sign = [ 1,-1,1,-1]; % [Clockwise,Anticlockwise,Clockwise,Anticlockwise]  
l=0.345; % Distance of propeler from origin. Arm lenght % To be changed/measured/tuned
h= 0.08; % vertical distance of the propellers form origin  % To be changed/measured/tuned
   
%% Propeller/motor parameters
% Go to ..\data\14x4.7 prop for finding the propeller data  and model fitting
% Go to ..\data\60A motor prop for finding the propeller data  and model fitting
J_m=4.2E-6; % inertia of motors
Tau_motors = 0.06; % Time  constant of the ESC+Motors  % To be changed/measured/tuned

R=0.254; % Porps radius
J_T=J_m+1/2*M_eq*R^2; % inertia of prop +motor % To be changed/measured/tuned

b=2; % num of blades of the props
c=3/100; % meters mean chord of props
sigma=b*c/(pi*R);
a=10.26;  % Mean C_l slope of sections of props <~2*pi % To be changed/measured/tuned
theta_0=0.38397;   % Propeller pith % To be changed/measured/tuned
theta_1=-0.2967;     % Usualy 0
C_d0=0.001193;    % Drag coefficient of propellers % To be changed/measured/tuned
%% Aerodynamic coefficients.
% They try to represents the aerodynamic influence of the vehicle moving
% across the air.  They can be perfectly 0 in first aprox.
C_x=1.2;
C_y=C_x;
A_x=0.015;
A_y=A_x;
C_z=2.2;
A_z=0.16;

%% General parameters of the quadcopter
rho=1.225; % Density at this position % To be changed/measured/tuned
g=9.81;
v_i0=sqrt( 1/4*m*g/(2*rho*pi*R^2)  ); % Induced velocity in hover

%% Load the AB model of induced velocities. Propeller model

load('ABModelDimless.mat')

% % % [X,Y]=meshgrid(Vx_adim,Vz_adim);
% % % surf(X,Y,v_i_adim');
% % % hold on;
% % % scatter3(0,0,1.16,'filled','LineWidth',20);
% % % hold off;
% % % legend('Dimless induced velocoty','Hover position')
% % % xlabel('V_x Dimless')
% % % ylabel('V_z Dimless')
% % % zlabel('V_i Dimless')

%% Load the Torque-Throttle-Omega model of ESC+motor: 0.46Glows 60A motor+60A ESC
% Go to ..\data\60A motor for finding the propeller+motor+ESC data  and model fitting
Ref_volt=16;% Reference voltage

load('TTO_map.mat')% To be changed/measured/tuned
figure
surf(Throtle_TTO,Torque_TTO,Omega_TTO)
Torque_TTO_vec=Torque_TTO(:,1);
Throtle_TTO_vec=Throtle_TTO(1,:);


%% Load the Throttle-Omega model of ESC+motor 
load('TO_curve.mat')

