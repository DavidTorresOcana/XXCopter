%%
close all
clear all
clc
% Params
R=0.1016;
theta_0=0.179049;
c=0.02;
sigma=2*c/(pi*R);
%TBT
theta_1=-0.341459;
a=13.13;
c_d_0=0.02;


load('Raw_data.mat')


Cost_aero(C_TModel,C_QModel,sigma,theta_0, theta_1,a,c_d_0)


[X,cost]= fminunc(@(X) 1000000*Cost_aero(C_TModel,C_QModel,sigma,theta_0, X(1),X(2),X(3)), [theta_1,a,c] )

theta_1=X(1);
a=X(2);
c=X(3);
%% PLot

subplot(2,1,1)
plot(C_TModel(:,1),C_TModel(:,2),'*');
hold on
plot(C_TModel(:,1),sigma*a/4*(theta_0*2/3+theta_1/2+C_TModel(:,1)),'*r')

subplot(2,1,2)
plot(C_QModel(:,1),C_QModel(:,2),'*');
hold on
plot(C_QModel(:,1),sigma*a/4*(theta_0*2/3+theta_1/2+C_QModel(:,1)).*C_QModel(:,1) + sigma*c_d_0/8,'*r')


