function [fitresult, gof] = TO_curve_fit(throtle, Omega)
%CREATEFIT(THROTLE,OMEGA)
%  Create a fit.
%
%  Data for 'untitled fit 1' fit:
%      X Input : throtle
%      Y Output: Omega
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 20-Mar-2015 23:27:18


%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( throtle, Omega );

% Set up fittype and options.
ft = fittype( 'gauss1' );
opts = fitoptions( ft );
opts.Display = 'Off';
opts.Lower = [-Inf -Inf 0];
opts.StartPoint = [598.747575867714 31.5707142857143 10.9405095033398];
opts.Upper = [Inf Inf Inf];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, xData, yData );
legend( h, 'Omega vs. throtle', 'untitled fit 1', 'Location', 'NorthEast' );
% Label axes
xlabel( 'throtle' );
ylabel( 'Omega' );
grid on


