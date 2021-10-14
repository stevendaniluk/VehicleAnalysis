% MotecHandlerAcc
%
% Version of MotecHandler for extracting MoTeC data produced by Assetto Corsa Competizione.
classdef MotecHandlerAcc < MotecHandler
    methods
        function [v, t] = getSpeed(this, varargin)
            [v, t] = this.getChannel('SPEED', varargin{:});
            v = v / 3.6;
        end

        function [a, t] = getLongitudinalAccel(this, varargin)
            [a, t] = this.getChannel('G_LON', varargin{:});
            a = a * 9.8067;
        end

        function [a, t] = getLateralAccel(this, varargin)
            [a, t] = this.getChannel('G_LAT', varargin{:});
            a = -a * 9.8067;
        end

        function [w, t] = getYawRate(this, varargin)
            [w, t] = this.getChannel('ROTY', varargin{:});
            w = -w * pi / 180;
        end

        function [rpm, t] = getRPM(this, varargin)
            [rpm, t] = this.getChannel('RPMS', varargin{:});
        end

        function [c, t] = getThrottle(this, varargin)
            [c, t] = this.getChannel('THROTTLE', varargin{:});
        end

        function [c, t] = getBrake(this, varargin)
            [c, t] = this.getChannel('BRAKE', varargin{:});
        end

        function [c, t] = getClutch(this, varargin)
            [c, t] = this.getChannel('CLUTCH', varargin{:});
        end

        function [theta, t] = getSteeringWheelAngle(this, varargin)
            [theta, t] = this.getChannel('STEERANGLE', varargin{:});
            theta = -theta * pi / 180;
        end

        function [x, t] = getDamperPosFL(this, varargin)
            [x, t] = this.getChannel('SUS_TRAVEL_LF', varargin{:});
        end

        function [x, t] = getDamperPosFR(this, varargin)
            [x, t] = this.getChannel('SUS_TRAVEL_RF', varargin{:});
        end

        function [x, t] = getDamperPosRL(this, varargin)
            [x, t] = this.getChannel('SUS_TRAVEL_LR', varargin{:});
        end

        function [x, t] = getDamperPosRR(this, varargin)
            [x, t] = this.getChannel('SUS_TRAVEL_RR', varargin{:});
        end

        function [x, t] = getDamperPosAvgF(this, varargin)
            [xl, t] = this.getDamperPosFL(varargin{:});
            [xr, ~] = this.getDamperPosFR(varargin{:});
            x = (xl + xr) / 2;
        end

        function [x, t] = getDamperPosAvgR(this, varargin)
            [xl, t] = this.getDamperPosRL(varargin{:});
            [xr, ~] = this.getDamperPosRR(varargin{:});
            x = (xl + xr) / 2;
        end

        function [x, t] = getWheelSpeedFL(this, varargin)
            [x, t] = this.getChannel('WHEEL_SPEED_LF', varargin{:});
        end

        function [x, t] = getWheelSpeedFR(this, varargin)
            [x, t] = this.getChannel('WHEEL_SPEED_RF', varargin{:});
        end

        function [x, t] = getWheelSpeedRL(this, varargin)
            [x, t] = this.getChannel('WHEEL_SPEED_LR', varargin{:});
        end

        function [x, t] = getWheelSpeedRR(this, varargin)
            [x, t] = this.getChannel('WHEEL_SPEED_RR', varargin{:});
        end

        function [x, t] = getTirePressureFL(this, varargin)
            [x, t] = this.getChannel('TYRE_PRESS_LF', varargin{:});
        end

        function [x, t] = getTirePressureFR(this, varargin)
            [x, t] = this.getChannel('TYRE_PRESS_RF', varargin{:});
        end

        function [x, t] = getTirePressureRL(this, varargin)
            [x, t] = this.getChannel('TYRE_PRESS_LR', varargin{:});
        end

        function [x, t] = getTirePressureRR(this, varargin)
            [x, t] = this.getChannel('TYRE_PRESS_RR', varargin{:});
        end
    end
end
