% computeDamperZero
%
% Estimates the damper positions that correspond to zero spring force. This uses
% two setups that different springs rates (but are the same otherwise).
%
% INPUTS:
%   setup: Setup that has had fuel properties estimated
%   setup_diff_springs: Copy of setup but with different spring rates
%   xf: Front damper position with initial setup [mm]
%   xr: Rear damper position with initial setup [mm]
%   xf_diff: Front damper position with setup with different springs [mm]
%   xr_diff: Rear damper position with setup with different springs [mm]
% OUTPUTS:
%   setup: Input setup with the fields {xf0, xr0} populated
function setup = computeDamperZero(setup, setup_diff_springs, xf, xr, xf_diff, xr_diff)
    % Solve for the front and rear neutral damper positions by equating the
    % the front/rear spring forces between setups 1 and 2 (accounting for fuel
    % level)
    % F_spring_front_1 = F_mass_front + F_fuel_front_1
    % F_spring_front_2 = F_mass_front + F_fuel_front_2
    % --> F_spring_front_2 - F_spring_front_1 = F_fuel_front_2 - F_fuel_front_1
    %
    % F_spring_front_1 = 2(xf1 - xf0) * kspring_wheel_f1;
    % 2(xf2 - xf0) * kf2 - 2(xf1 - xf0) * kf1 = (lf2 / L) * Vf2 * p_fuel * g - (lf2 / L) * Vf1 * p_fuel * g
    % 2(xr2 - xr0) * kr2 - 2(xr1 - xr0) * kr1 = (lf1 / L) * Vf2 * p_fuel * g - (lf1 / L) * Vf1 * p_fuel * g
    %
    F_fuel_front_1 = (setup.lf2 / setup.L) * setup.V_fuel * setup.p_fuel * setup.g;
    F_fuel_front_2 = (setup.lf2 / setup.L) * setup_diff_springs.V_fuel * setup.p_fuel * setup.g;

    F_fuel_rear_1 = (setup.lf1 / setup.L) * setup.V_fuel * setup.p_fuel * setup.g;
    F_fuel_rear_2 = (setup.lf1 / setup.L) * setup_diff_springs.V_fuel * setup.p_fuel * setup.g;

    setup.xf0 = (F_fuel_front_2 - F_fuel_front_1 - 2 ...
        * setup_diff_springs.kspring_wheel_f * 1e-3 * xf_diff ...
        + 2 * setup.kspring_wheel_f * 1e-3 * xf) ...
        / (setup.kspring_wheel_f - setup_diff_springs.kspring_wheel_f);

    setup.xr0 = (F_fuel_rear_2 - F_fuel_rear_1 - 2 ...
        * setup_diff_springs.kspring_wheel_r * 1e-3 * xr_diff ...
        + 2 * setup.kspring_wheel_r * 1e-3 * xr) ...
        / (setup.kspring_wheel_r - setup_diff_springs.kspring_wheel_r);
end
