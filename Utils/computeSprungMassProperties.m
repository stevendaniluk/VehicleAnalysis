% computeSprungMassProperties
%
% Estimates the sprung mass and longitudinal position of the sprung mass CoG
%
% INPUTS:
%   setup: Setup that has had damper zero positions and fuel properties
%          estimated
%   xf: Front damper position [mm]
%   xr: Rear damper position [mm]
% OUTPUTS:
%   setup: Input setup with the fields {m_sprung, lm1, lm2} populated
function setup = computeSprungMassProperties(setup, xf, xr)
    % Solve for the sprung mass by doing a force balance on a single setup
    % F_spring_front + F_spring_rear = F_sprung_mass + F_fuel
    % 2(xf - xf0) * kf + 2(xr - xr0) * kr = m_sprung * g + Vf * p_fuel * g
    setup.m_sprung = (2 * 1e-3 * (xf- setup.xf0) * setup.kspring_wheel_f ...
        + 2 * 1e-3 * (xr - setup.xr0) * setup.kspring_wheel_r ...
        - setup.V_fuel * setup.p_fuel * setup.g) / setup.g;

    setup.m_unsprung = setup.m_dry - setup.m_sprung;

    % Solve for the sprung mass CoG position by doing a moment balance about the
    % front and rear axles
    % 2(xf - xf0) * L = ms * g * lm2 + V_fuel * p_fuel * g * lf2
    % 2(xr - xr0) * L = ms * g * lm1 + V_fuel * p_fuel * g * lf1
    setup.lm1 = (2 * 1e-3 * (xr - setup.xr0) * setup.kspring_wheel_r * setup.L ...
        - setup.V_fuel * setup.p_fuel * setup.g * setup.lf1) / (setup.m_sprung * setup.g);
    setup.lm2 = (2 * 1e-3 * (xf - setup.xf0) * setup.kspring_wheel_f * setup.L ...
        - setup.V_fuel * setup.p_fuel * setup.g * setup.lf2) / (setup.m_sprung * setup.g);
end
