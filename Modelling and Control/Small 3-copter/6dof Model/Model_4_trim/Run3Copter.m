%Run the model of the 3copter
clear all
close all
clc
global throtle_1 throtle_23 eta_23
addpath(genpath('Used Functions'))
T_sim=0.01;
%% Load data
SetupComplete;

close all
% return
%% Find trim point
% Cost evaluation: 1 time
throtle_1= 0.745; % Those are suposed to be the trimed state ones
eta_23 = deg2rad(-1.3);
throtle_23 = throtle_1/cos(eta_23);
Trim_Cost = TrimCostFunction(100*[throtle_1,throtle_23,eta_23])

% return
%% Trim by  Optimization
% % options=optimset('Display','iter','LargeScale','off','TolFun',10e-9,'TolX',10e-11,'MaxFunEvals',6000,'MaxIter',6000);
% % 
% % [X,fval]= fminunc( @(X) TrimCostFunction(X), 100*[throtle_1,throtle_23,eta_23] )
% % 
% % throtle_1=X(1)/100;
% % throtle_23=X(2)/100;
% % eta_23=X(3)/100;
% % 
% % Cost = TrimCostFunction(100*[throtle_1,throtle_23,eta_23])

%% Retrieve real model trim values: Hover

% Hover point for trim
Omega_hover(1)=fitresult_TO(Ref_volt*throtle_1);
Omega_hover(2)=fitresult_TO(Ref_volt*throtle_23);
Omega_hover(3)=fitresult_TO(Ref_volt*throtle_23);

% Find Torques hover
load('TTO_raw_data.mat')

fitt_TTO=TTO_data_fitV2(throtle(throtle<=18), Mom(throtle<=18), Omega(throtle<=18));

X=fminunc( @(X) (fitt_TTO(Ref_volt*throtle_1,X) -Omega_hover(1))^2,0.1505);
Torque_hover(1)=X;
X=fminunc( @(X) (fitt_TTO(Ref_volt*throtle_23,X) -Omega_hover(2))^2,0.1505);
Torque_hover(2)=X;
X=fminunc( @(X) (fitt_TTO(Ref_volt*throtle_23,X) -Omega_hover(3))^2,0.1505);
Torque_hover(3)=X;



