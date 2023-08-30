% fitTireRadiusFunctionWPFz
%
% Fits a function to estimate effective tire radius as a function of angular velocity,
% pressure, and normal load.


% Fits a function to an aerodynamic force that is a function of
% three variables. The first variable must be velocity, the function
% will be constrained to produce zero force at zero velocity.
%
% INPUTS:
%   v: Linear X velocity of the wheel [m/s]
%   P: Tire pressure
%   w: Wheel angular velocity [rad/s]
%   Fz: Normal load
% OUTPUTS:
%   eff_rad_fit: Fitted function which can be evaluated as
%              r = eff_rad_fit(w, P, Fz)
%   p_fit: Polynomial parameters
function [eff_rad_fit, p_fit] = fitTireRadiusFunctionWPFz(v, P, w, Fz)
    % Compute the target effective tire radius based on measured linear velocity and
    % rotational rate
    r_eff_meas = v ./ w;

    % Form of the function to fit (x matrix is [P, W, Fz])
    %   -Linear variation with pressure only
    %   -Quadratic variation with velocity and pressure
    %   -Quadratic variation with normal load and pressure
    r_func = @(p, x) ...
        p(1) + p(2) * x(1, :) + ...
        p(3) * x(1, :) .* x(2, :) + p(4) * x(1, :).^2 .* x(2, :) + ...
        p(5) * x(1, :) .* x(2, :).^2 + p(6) * x(1, :).^2 .* x(2, :).^2
        p(7) * x(1, :) .* x(3, :) + p(8) * x(1, :).^2 .* x(3, :) + ...
        p(9) * x(1, :) .* x(3, :).^2 + p(10) * x(1, :).^2 .* x(3, :).^2

    % Lower and upper bounds for the surface fit polynomials that will constrain the
    % nominal radius to be >0 and to increase as pressure increases
    lb = -inf(1, 10);
    lb(1) = 0;
    lb(2) = 0;
    ub = inf(1, 10);

    % Initial parameters to begin fit at
    r_init  = mean(r_eff_meas);
    p0 = [r_init, 0, 0, 0, 0, 0, 0, 0, 0, 0];

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
    p_fit = lsqcurvefit(r_func, p0, [P; w; Fz], r_eff_meas, lb, ub, opts);
    eff_rad_fit = @(P, w, Fz) ...
        p(1) + p(2) * P + ...
        p(3) * P .* w + p(4) * P.^2 .* w + ...
        p(5) * P .* w.^2 + p(6) * P.^2 .* w.^2
        p(7) * P .* Fz + p(8) * P.^2 .* Fz + ...
        p(9) * P .* Fz.^2 + p(10) * P.^2 .* Fz.^2
end
