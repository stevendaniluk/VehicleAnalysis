% MotecHandler
%
% Utility class for extracting data from a MoTeC log which has been exported to
% a Matlab format.
%
% This class defines the interface for querying common channels. Individual applications need to
% extract the appropriate channels and perform all the data conversions.
classdef MotecHandler
    properties
        log;
    end

    methods
        % loadFromFile
        %
        % INPUTS:
        %   filename: Name of matlab file (exported from MoTeC) to load
        function this = loadFromFile(this, filename)
            [~, ~, ext] = fileparts(filename);
            if ~isequal(ext, '.mat')
                filename = strcat(filename, '.mat');
            end

            this = this.loadFromMatlabData(load(filename));
        end

        % loadFromMatlabData
        %
        % INPUTS:
        %   log: Matlab object to initialize from (exported from MoTeC)
        function this = loadFromMatlabData(this, log)
            this.log = log;
        end

        % trimTimeRange
        %
        % Trims the log data to be within a time window.
        %
        % INPUTS:
        %   t_start: Start time window (inclusive)
        %   t_end: End time of time window (inclusive)
        function this = trimTimeRange(this, t_start, t_end)
            fields = fieldnames(this.log);
            for i=1:length(fields)
                indices = ...
                    and(this.log.(fields{i}).Time >= t_start, this.log.(fields{i}).Time <= t_end);
                this.log.(fields{i}).Time = this.log.(fields{i}).Time(indices);
                this.log.(fields{i}).Value = this.log.(fields{i}).Value(indices);
            end

        end

        % getChannel
        %
        % Access for any channel by name.
        %
        % INPUTS:
        %   channel: Name of channel to extract
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   x: Channel values
        %   t: Timestamps
        function [x, t] = getChannel(this, channel, varargin)
            if ~isempty(varargin) > 0 && ~isempty(varargin{1})
                t_start = varargin{1};
            else
                t_start = 0;
            end
            if length(varargin) > 1 && ~isempty(varargin{2})
                t_end = varargin{2};
            else
                t_end = inf;
            end

            indices = and(this.log.(channel).Time >= t_start, this.log.(channel).Time <= t_end);
            t = this.log.(channel).Time(indices);
            x = this.log.(channel).Value(indices);
        end

        % detectCoastPeriod
        %
        % Looks for a period in the MoTeC log that meets the following criteria:
        %   -Clutch fully pressed
        %   -Engine running
        %   -Speed is within some range
        %   -All of the above are true for 5s
        %
        % INPUTS:
        %   min_speed: Minimum speed for coast period
        %   max_speed: Maximum speed for coast period
        %   trim: Buffer to trim the start and end times by to shorten the period (optional) [s]
        %   t_offset: Time offset to begin searching at (optional) [s]
        % OUTPUTS:
        %   t_start: Beginning of stationary period (-inf when not found) [s]
        %   t_end: End of stationary period (inf when not found) [s]
        function [t_start, t_end] = detectCoastPeriod(this, min_speed, max_speed, trim, t_offset)
            if nargin < 4
                trim = 0;
            end
            if nargin < 5
                t_offset = 0;
            end

            [v, t] = this.getSpeed();
            [clutch, ~] = this.getClutch();
            [rpm, ~] = this.getRPM();

            % Channel thresholds to be met during a stationary period
            clutch_thresh = 0.95;
            rpm_thresh = 500;
            duration_thresh = 5.0;

            % Search through the data looking for all conditions to be met
            t_start = -inf;
            t_end = inf;
            period_found = false;

            start_index = find(t >= t_offset, 1);
            for i=start_index:length(t)
                clutch_ok = clutch(i) >= clutch_thresh;
                speed_ok = v(i) <= max_speed && v(i) >= min_speed;
                rpm_ok = rpm(i) >= rpm_thresh;
                if clutch_ok && speed_ok && rpm_ok
                    % Conditions are satisfied
                    if isinf(t_start)
                        % Mark the start time first time we see this
                        t_start = t(i) + trim;
                    end

                    % Always update the end time
                    t_end = t(i) - trim;

                    % Record once we've exceeded the target duration so we can stop
                    % searching once this period ends
                    if (t_end - t_start) > duration_thresh
                        period_found = true;
                    end
                else
                    % Stationary conditions not met, determine if we can free stop
                    % or if we need to reset and keep searching
                    if period_found
                        break
                    else
                        % Reset the start time
                        t_start = -inf;
                    end
                end
            end

            if ~period_found
                % Reset the times to their default values so the user knows a stationary
                % period wasn't found
                t_start = -inf;
                t_end = inf;
            end
        end

        % getSpeed
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   v: Velocity [m/s]
        %   t: Timestamps [s]
        function [v, t] = getSpeed(this, varargin)
            v = [];
            t = [];
        end

        % getLongitudinalAccel
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   a: Longitudinal acceleration (+ve forward) [m/s^2]
        %   t: Timestamps [s]
        function [a, t] = getLongitudinalAccel(this, varargin)
            a = [];
            t = [];
        end

        % getLateralAccel
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   a: Lateral acceleration (+ve right) [m/s^2]
        %   t: Timestamps [s]
        function [a, t] = getLateralAccel(this, varargin)
            a = [];
            t = [];
        end

        % getYawRate
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   w: Angular velocity about the Z axis (+ve turning right) [rad/s]
        %   t: Timestamps [s]
        function [w, t] = getYawRate(this, varargin)
            w = [];
            t = [];
        end

        % getRPM
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   rpm: Engine RPM [rev/minute]
        %   t: Timestamps [s]
        function [rpm, t] = getRPM(this, varargin)
            rpm = [];
            t = [];
        end

        % getThrottle
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   throttle: Throttle position (0=unpressed, 1=pressed)
        %   t: Timestamps [s]
        function [throttle, t] = getThrottle(this, varargin)
            throttle = [];
            t = [];
        end

        % getBrake
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   brake: Brake position (0=unpressed, 1=pressed)
        %   t: Timestamps [s]
        function [brake, t] = getBrake(this, varargin)
            brake = [];
            t = [];
        end

        % getClutch
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   clutch: Clutch position (0=unpressed, 1=pressed)
        %   t: Timestamps [s]
        function [clutch, t] = getClutch(this, varargin)
            clutch = [];
            t = [];
        end

        % getSteeringWheelAngle
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   theta: Steering wheel angle (+ve turns right) [rad]
        %   t: Timestamps [s]
        function [theta, t] = getSteeringWheelAngle(this, varargin)
            theta = [];
            t = [];
        end

        % getDamperPosXX
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   x: Damper position [mm]
        %   t: Timestamps [s]
        function [x, t] = getDamperPosFL(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getDamperPosFR(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getDamperPosRL(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getDamperPosRR(this, varargin)
            x = [];
            t = [];
        end

        % getDamperPosAvgF
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   x: Average FL and FR damper position [mm]
        %   t: Timestamps [s]
        function [x, t] = getDamperPosAvgF(this, varargin)
            x = [];
            t = [];
        end

        % getDamperPosAvgR
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   x: Average RL and RR damper position [mm]
        %   t: Timestamps [s]
        function [x, t] = getDamperPosAvgR(this, varargin)
            x = [];
            t = [];
        end

        % getWheelSpeedXX
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   x: Effective wheel speed [m/s]
        %   t: Timestamps [s]
        function [x, t] = getWheelSpeedFL(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getWheelSpeedFR(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getWheelSpeedRL(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getWheelSpeedRR(this, varargin)
            x = [];
            t = [];
        end

        % getTirePressureXX
        %
        % INPUTS:
        %   t_start: Beginning of time range to extract (optional) [s]
        %   t_end: End of time range to extract (optional) [s]
        % OUTPUTS:
        %   x: Tire pressure [psi]
        %   t: Timestamps [s]
        function [x, t] = getTirePressureFL(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getTirePressureFR(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getTirePressureRL(this, varargin)
            x = [];
            t = [];
        end
        function [x, t] = getTirePressureRR(this, varargin)
            x = [];
            t = [];
        end
    end
end
