% fuelLevelDuringRun
%
% Estimates the fuel level during a run by interpolating between initial and
% final fuel levels based on distance travelled.
%
% INPUTS:
%   V_i: Initial measured fuel level
%   V_f: Final measured fuel level
%   t: Timestamps
%   v: Groudn speed
function V_fuel = fuelLevelDuringRun(V_i, V_f, t, v)
    % Estimate distance travelled via Euler integration from first point onwards
    dt = diff(t);
    ds = v(1:end - 1) .* dt;
    d = cumsum(ds);
    d_total = d(end);
    
    % Interpolate fuel levels based on distance travelled
    V_fuel = zeros(size(t));
    V_fuel(1) = V_i;
    V_fuel(2:end) = V_i - (d / d_total) * (V_i - V_f);
end

