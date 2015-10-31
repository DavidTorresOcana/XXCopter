%Run the model of the 3copter
clear all
close all
clc
global throtle_0
addpath(genpath('Used Functions'),genpath('MCopter Model'))
T_sim=0.01;
%% Load data, dimensions, etc
SetupComplete;

close all
% return

%% Find trim point
Alt_0=20;% =input(' Give the initial altitude (m): ');
% Cost evaluation: 1 time
throtle_0= 0.4; % Those are suposed to be the trimed state ones

% return


%% Retrieve real model trim values: Hover
load('TO_curve_raw.mat')
p0=rand;p1=rand;p2=rand;p3=rand;
Cost_TO(throtle,Omega,[p0,p1,p2,p3]);
p_good = minimize(  @(p) Cost_TO(throtle,Omega,p), [p0,p1,p2,p3] );
fitresult_TO = @(x) p_good(1)+p_good(2)*x+p_good(3)*x.^2+p_good(4)*x.^3;

plot(throtle,Omega,'r*')
hold on
x= 2:0.1:20;
plot(x,fitresult_TO(x))
xlabel('Throtle');ylabel('omega')
legend(' Data','Fitted')

% Hover point for trim
Omega_hover=fitresult_TO(Ref_volt*throtle_0);

Initial_Omegas =Omega_hover*ones(4,1);

close all
clear x y z
% Find Torques hover
load('TTO_raw_data.mat')

p0=530;p1=-450;p2=0.6;
Cost_TTO(throtle,Mom,Omega,[p0,p1,p2]);
options = optimset('TolFun', 1e-8, 'TolX', 1e-8,'MaxFunEvals',900000);
[p_good_TTO,fcost] = minimize(  @(p) Cost_TTO(throtle(throtle<=18),Mom(throtle<=18),Omega(throtle<=18),p), [p0,p1,p2],[],[],[],[],[],[],[],options );
fitresult_TTO = @(x,y) max(  (p_good_TTO(1).*log((x).*atan2(x,y))+p_good_TTO(2)).*(-y.^2+p_good_TTO(3))  ,  0) +eps;

plot3(throtle,Mom,Omega,'b.','Markersize',20)
hold on
[x,y]= meshgrid(2:0.1:20,0.001:0.05:1);

for i=1:size(x,1)
    for j=1:size(x,2)
        z(i,j)=fitresult_TTO(x(i,j),y(i,j));
    end
end
surf(x,y,z)
grid

Torque_hover=minimize( @(X) (fitresult_TTO(Ref_volt*throtle_0,X) -Omega_hover)^2,0.154,[],[],[],[],0,1,[]);


plot3(Ref_volt*throtle_0,Torque_hover,Omega_hover,'r.','Markersize',20)
xlabel('Throtle');ylabel('Mom');ylabel('omega')
legend(' Data','Fitted','Hover')
pause
Initial_Torques=Torque_hover*ones(4,1);


close all