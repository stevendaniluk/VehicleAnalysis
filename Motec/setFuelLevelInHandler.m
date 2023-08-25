function handler = setFuelLevelInHandler(handler, V_f, V_i)
    t = handler.getTimestamps();
    v_ground = handler.getChannel('GROUND_SPEED');
    V_fuel_full = fuelLevelDuringRun(V_i, V_f, t, v_ground);
    handler = handler.addChannelData('FUEL_LEVEL', V_fuel_full, 'L');
end
