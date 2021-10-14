% fitAeroForceFunction2d
%
% Fits a function to an aerodynamic force that is a function of
% two variables. The first variable must be velocity, the function
% will be constrained to produce zero force at zero velocity.
%
% INPUTS:
%   v: Measured velocity
%   x: Second variable
%   force_meas: Measured force
% OUTPUTS:
%   force_fit: Fitted function which can be evaluated as
%              F = force_fit(v, x)
%   p_fit: Polynomail parameters
function [force_fit, p_fit] = fitAeroForceFunction2d(v, x, force_meas)
    % Form of the function to fit
    F_func = @(p, x) ...
        (p(1) * x(1, :).^2 + p(2) * x(1, :).^3 + p(3) * x(1, :).^4) .* ...
        (p(4) + p(5) * x(2, :) + p(6) * x(2, :).^2);

    % Lower and upper bounds for the surface fit polynomials that will constrain the
    % downforce to be zero at zero velocity
    lb = -inf(1, 6);
    lb(1) = 0;
    ub = inf(1, 6);

    % Initial parameters to begin fit at
    p0 = [1, 0, 0, 1, 0, 0];

    % Optimization options
    opts = optimoptions('lsqcurvefit');
    opts.Algorithm = 'trust-region-reflective';
    opts.Display = 'off';
    opts.MaxIterations = 1000;
    opts.MaxFunctionEvaluations = 6000;
    opts.OptimalityTolerance = 1e-6;
    opts.StepTolerance = 1e-6;

    % Perform the fit, then create a new function handle that uses the
    % fitted parameters. We have to re write the full polynomial here with the
    % fitted parameter values.
    p_fit = lsqcurvefit(F_func, p0, [v; x], force_meas, lb, ub, opts);
    force_fit = @(v, x) (p_fit(1) * v.^2 + p_fit(2) * v.^3 + p_fit(3) * v.^4) .* ...
        (p_fit(4) + p_fit(5) * x + p_fit(6) * x.^2);
end
