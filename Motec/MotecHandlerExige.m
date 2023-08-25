% MotecHandlerExige
%
% Version of MotecHandler for extracting MoTeC data produced by the Exige
classdef MotecHandlerExige < MotecHandler
    methods
        function this = preProcessLogData(this)
            % Convert ground speed from km/h to m/s
            this.log.Ground_Speed.Value = this.log.Ground_Speed.Value / 3.6;
            this.log.Ground_Speed.Units = 'm/s';

            % Convert steering wheel angle from degrees to radians
            this.log.Steering_Wheel_Angle.Value = (pi / 180) * this.log.Steering_Wheel_Angle.Value;
            this.log.Steering_Wheel_Angle.Units = 'rad';

            % Convert damper positions to meters
            this.log.Damper_Pos_FL.Value = 1e-3 * this.log.Damper_Pos_FL.Value;
            this.log.Damper_Pos_FL.Units = 'm';

            this.log.Damper_Pos_FR.Value = 1e-3 * this.log.Damper_Pos_FR.Value;
            this.log.Damper_Pos_FR.Units = 'm';

            this.log.Damper_Pos_RL.Value = 1e-3 * this.log.Damper_Pos_RL.Value;
            this.log.Damper_Pos_RL.Units = 'm';

            this.log.Damper_Pos_RR.Value = 1e-3 * this.log.Damper_Pos_RR.Value;
            this.log.Damper_Pos_RR.Units = 'm';
        end

        function this = defineChannelMap(this)
            this = this.addChannelMappingEntry('LAP_DISTANCE', 'Lap_Distance');
            this = this.addChannelMappingEntry('GROUND_SPEED', 'Ground_Speed');
            this = this.addChannelMappingEntry('AIR_DYN_PRESSURE', 'Dynamic_Air_Pressure');
            this = this.addChannelMappingEntry('AX', 'Acceleration_X');
            this = this.addChannelMappingEntry('AY', 'Acceleration_Y');
            this = this.addChannelMappingEntry('AZ', 'Acceleration_Z');
            this = this.addChannelMappingEntry('WX', 'Gyro_Roll_Velocity');
            this = this.addChannelMappingEntry('WY', 'Gyro_Pitch_Velocity');
            this = this.addChannelMappingEntry('WZ', 'Gyro_Yaw_Velocity');
            this = this.addChannelMappingEntry('ELEVATION', 'GPS_Altitude');
            this = this.addChannelMappingEntry('ENGINE_RPM', 'Engine_RPM');
            this = this.addChannelMappingEntry('FUEL_LEVEL', 'FUEL_LEVEL');
            this = this.addChannelMappingEntry('THROTTLE', 'Throttle_Pedal');
            this = this.addChannelMappingEntry('BRAKE', 'Brake_Pedal');
            this = this.addChannelMappingEntry('CLUTCH', 'Clutch_Pedal');
            this = this.addChannelMappingEntry('STEERING_WHEEL', 'Steering_Wheel_Angle');
            this = this.addChannelMappingEntry('DAMPER_FL', 'Damper_Pos_FL');
            this = this.addChannelMappingEntry('DAMPER_FR', 'Damper_Pos_FR');
            this = this.addChannelMappingEntry('DAMPER_RL', 'Damper_Pos_RL');
            this = this.addChannelMappingEntry('DAMPER_RR', 'Damper_Pos_RR');
            this = this.addChannelMappingEntry('WS_ROT_FL', 'Wheel_Rot_Speed_FL');
            this = this.addChannelMappingEntry('WS_ROT_FR', 'Wheel_Rot_Speed_FR');
            this = this.addChannelMappingEntry('WS_ROT_RL', 'Wheel_Rot_Speed_RL');
            this = this.addChannelMappingEntry('WS_ROT_RR', 'Wheel_Rot_Speed_RR');
            this = this.addChannelMappingEntry('TIRE_P_FL', 'Tire_Pressure_FL');
            this = this.addChannelMappingEntry('TIRE_P_FR', 'Tire_Pressure_FR');
            this = this.addChannelMappingEntry('TIRE_P_RL', 'Tire_Pressure_RL');
            this = this.addChannelMappingEntry('TIRE_P_RR', 'Tire_Pressure_RR');
            this = this.addChannelMappingEntry('TIRE_T_FL', 'Tire_Temp_Center FL');
            this = this.addChannelMappingEntry('TIRE_T_FR', 'Tire_Temp_Center_FR');
            this = this.addChannelMappingEntry('TIRE_T_RL', 'Tire Temp_Center RL');
            this = this.addChannelMappingEntry('TIRE_T_RR', 'Tire_Temp Center_RR');
        end
    end
end
