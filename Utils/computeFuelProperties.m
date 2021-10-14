% computeFuelProperties
%
% Estimates the fuel density and fuel tank CoG longitudinal position.
%
% INPUTS:
%   setup: Setup pre pit stop
%   xf_pre: Front damper position pre pit stop [mm]
%   xr_pre: Rear damper position pre pit stop [mm]
%   xf_post: Front damper position post pit stop [mm]
%   xr_post: Rear damper position post pit stop [mm]
%   V_fuel: Fuel level post pit stop
% OUTPUTS:
%   setup: Input setup with the fields {p_fuel, lf1, lf2} populated
function setup = computeFuelProperties(setup, xf_pre, xr_pre, xf_post, xr_post, V_fuel)
    % First estimate the fuel density by comparing damper positions pre and post
    % pit stop along with fuel level and doing a force balance
    % 2 * (xf2 - xf1) * kf + 2 * (xr2 - xr1) * kr = dV * p * g
    F_fuel_front = 2 * 1e-3 * (xf_post - xf_pre) * setup.kspring_wheel_f;
    F_fuel_rear = 2 * 1e-3 * (xr_post - xr_pre) * setup.kspring_wheel_r;
    dV_fuel = V_fuel - setup.V_fuel;

    setup.p_fuel = (F_fuel_front + F_fuel_rear) / (dV_fuel * setup.g);

    % Determine the fuel tank position from a moment balance about the front and
    % rear wheels
    % 2 * (xf2 - xf1) * kf * L = dV * p * g * lf2
    % 2 * (xr2 - xr1) * kr * L = dV * p * g * lf1
    setup.lf1 = (F_fuel_rear * setup.L) / (setup.p_fuel * dV_fuel * setup.g);
    setup.lf2 = (F_fuel_front * setup.L) / (setup.p_fuel * dV_fuel * setup.g);
end
