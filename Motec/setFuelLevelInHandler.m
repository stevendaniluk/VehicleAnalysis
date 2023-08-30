% setFuelLevelInHandler
%
% Populates a data channel in a MotecHandler with 'FUEL_LEVEL'
%
% INPUTS:
%   handler: MotecHandler to populate data in
%   V_i: Initial fuel level [l]
%   V_f: Final fuel level [l]
% OUTPUTS:
%   handler: Updated MotecHandler
function handler = setFuelLevelInHandler(handler, V_i, V_f)
    t = handler.getTimestamps();
    v_ground = handler.getChannel('GROUND_SPEED');
    V_fuel_full = fuelLevelDuringRun(V_i, V_f, t, v_ground);
    handler = handler.addChannelData('FUEL_LEVEL', V_fuel_full, 'L');
end
