function [fitresult, gof] = TO_hover_map(throtle, Omega)
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

%  Auto-generated by MATLAB on 27-Dec-2014 11:48:44


%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( throtle, Omega );

% Set up fittype and options.
ft = fittype( 'poly3' );
opts = fitoptions( ft );
opts.Lower = [-Inf -Inf -Inf -Inf];
opts.Upper = [Inf Inf Inf Inf];

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


