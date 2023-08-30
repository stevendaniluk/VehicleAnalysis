% MotecHandler
%
% Utility class for extracting data from a MoTeC log which has been exported to
% a Matlab format.
%
% This class provides:
%   - A consistent interface for accessing a common set of channels
%   - The ability to access any channel in the log
%   - An optional pre-processing step to perform any unit conversions or other operations
%
% Users of this class must provide:
%   - The mapping between channel names in the log data and pre defined list of internal
%     channel names (e.g. THROTTLE --> throttle_pos, where THROTTLE is the inernal name
%     used within this class, and throttle_pos is the name in the log data). Full list
%     below. This can be done via defineChannelMap() in derived classes, or manually via
%     addChannelMappingEntry().
%   - Optional step to pre-process log data to do any necessary conversions, this can be
%     done in preProcessLogData() in derived classes.
%
% The internal channel names are listed below:
%   LAP_NUMBER
%   LAP_DISTANCE
%   GROUND_SPEED
%   AIR_DYN_PRESSURE
%   AX
%   AY
%   AZ
%   WX
%   WY
%   WZ
%   ELEVATION
%   ENGINE_RPM
%   FUEL_LEVEL
%   THROTTLE
%   BRAKE
%   CLUTCH
%   STEERING_WHEEL
%   DAMPER_FL
%   DAMPER_FR
%   DAMPER_RL
%   DAMPER_RR
%   WS_FL
%   WS_FR
%   WS_RL
%   WS_RR
%   WS_ROT_FL
%   WS_ROT_FR
%   WS_ROT_RL
%   WS_ROT_RR
%   TIRE_P_FL
%   TIRE_P_FR
%   TIRE_P_RL
%   TIRE_P_RR
%   TIRE_T_FL
%   TIRE_T_FR
%   TIRE_T_RL
%   TIRE_T_RR
%
classdef MotecHandler
    properties (Access = protected)
        channel_map = containers.Map;
        % Struct provided by MoTeC, each field name matches the channel name, and each
        % entry has "Time", "Value", and "Units" fields.
        log;
    end

    methods
        % MotecHandler
        %
        % INPUTS:
        %   source: Optional - Either a filename to load, or a struct containing log
        %           data (the type will be detected and parsed appropriately)
        function this = MotecHandler(source)
            this = this.initializeChannelMap();
            this = this.defineChannelMap();

            if nargin > 0
                if isa(source, 'char') || isa(source, 'string')
                    this = this.loadFromFile(source);
                elseif isa(source, 'struct')
                    this = this.loadFromMatlabData(source);
                end
            end
        end

        % loadFromFile
        %
        % INPUTS:
        %   filename: Name of matlab file (exported from MoTeC) to load
        function this = loadFromFile(this, filename)
            filename = string(filename);

            [~, ~, ext] = fileparts(filename);
            if ~isequal(ext, ".mat")
                filename = strcat(filename, ".mat");
            end

            this = this.loadFromMatlabData(load(filename));
        end

        % loadFromMatlabData
        %
        % INPUTS:
        %   log: Matlab object to initialize from (exported from MoTeC)
        function this = loadFromMatlabData(this, log)
            this.log = log;
            this = this.preProcessLogData();
        end

        % trimTimeRange
        %
        % Trims the log data to be within a time window.
        %
        % INPUTS:
        %   t_start: Start time window (inclusive)
        %   t_end: End time of time window (inclusive)
        function this = trimTimeRange(this, t_start, t_end)
            t = this.getTimestamps();
            indices = and(t >= t_start, t <= t_end);
            this = this.trimIndices(indices);
        end

        % excludeTimeRange
        %
        % Trims removes a section of data from the log
        %
        % INPUTS:
        %   t_start: Start time window (inclusive)
        %   t_end: End time of time window (inclusive)
        function this = excludeTimeRange(this, t_start, t_end)
            t = this.getTimestamps();
            indices = or(t < t_start, t > t_end);
            this = this.trimIndices(indices);
        end

        % trimLaps
        %
        % Trims the log data to only include certain laps
        %
        % INPUTS:
        %   lap_start: First lap number to include
        %   lap_end: Last lap number to include
        function this = trimLaps(this, lap_start, lap_end)
            lap = this.getChannel('LAP_NUMBER');
            indices = and(lap >= lap_start, lap <= lap_end);
            this = this.trimIndices(indices);
        end

        % exludeLaps
        %
        % Trims the log data to only include certain laps
        %
        % INPUTS:
        %   exclude_laps: Array of lap numbers to exlucde
        function this = exludeLaps(this, exclude_laps)
            lap = this.getChannel('LAP_NUMBER');
            indices = {};
            for i = 1 : length(exclude_laps)
                indices{i} = lap == exclude_laps{i};
            end
            indices = cell2mat(indices);
            this = this.trimIndices(indices);
        end

        % removeInAndOutLaps
        %
        % Trims the first and final laps from the log. The first lap is assumed
        % to be lap 1, so from lap 2 onward will be retained.
        function this = removeInAndOutLaps(this)
            lap = this.getChannel('LAP_NUMBER');
            indices = and(lap > 1, lap < lap(end));
            this = this.trimIndices(indices);
        end

        % trimIndices
        %
        % Trims the log data to only include specified data points
        %
        % INPUTS:
        %   stamps: Indices of points to include
        function this = trimIndices(this, stamps)
            fields = fieldnames(this.log);
            for i=1:length(fields)
                this.log.(fields{i}).Time = this.log.(fields{i}).Time(stamps);
                this.log.(fields{i}).Value = this.log.(fields{i}).Value(stamps);
            end
        end

        % getTimestamps
        %
        % INPUTS:
        %   See note at top
        % OUTPUTS:
        %   t: Timestamps [s]
        function t = getTimestamps(this, varargin)
            % All fields have the same timestamps in MoTeC logs, so we just grab the
            % first one
            fields = fieldnames(this.log);
            if isempty(varargin)
                % Grab the full set of data points
                t = this.log.(fields{1}).Time;
            else
                if length(varargin) == 1 && isa(varargin{1}, 'logical')
                    % Only grab specific time points
                    t = this.log.(fields{1}).Time(varargin{1});
                elseif length(varargin) == 2 && isa(varargin{1}, 'double') && isa(varargin{2}, 'double')
                    % Grab within a time range
                    t_start = varargin{1};
                    t_end = varargin{2};

                    indices = and(this.log.(fields{1}).Time >= t_start, this.log.(fields{1}).Time <= t_end);
                    t = this.log.(fields{1}).Time(indices);
                else
                    error('Cannot parse inputs for getting channel');
                end
            end
        end

        % getChannel
        %
        % Access for any channel by name. Can optionally only retrieve for specific
        % timestamps or time ranges.
        %
        % This uses the internal channel names stored in the channel name mapping, not
        % the channel names originally present in the log data.
        %
        % INPUTS:
        %   (channel): Channel is the internal name of channel to retrieve
        %   (channel, timestamps): timestamps is a logical array of time indices to
        %       retrieve data for
        %   (channel, t_start, t_end): Data from channel will be retrieved between the
        %       timestamps t_start and t_end
        % OUTPUTS:
        %   x: Channel values, empty when channel not present
        function x = getChannel(this, channel, varargin)
            if isKey(this.channel_map, channel)
                source_channel_name = this.channel_map(channel);
                x = this.getChannelNoRemap(source_channel_name, varargin{:});
            else
                x = [];
            end
        end

        % getChannelNoRemap
        %
        % Version of getChannel() that does not remap channel names, this directly
        % accesses the log for the channel name provided.
        function x = getChannelNoRemap(this, channel, varargin)
            if ~isfield(this.log, channel)
                x = [];
                return;
            end

            if isempty(varargin)
                % Grab the full set of data points
                x = this.log.(channel).Value;
            else
                if length(varargin) == 1 && isa(varargin{1}, 'logical')
                    % Only grab specific time points
                    x = this.log.(channel).Value(varargin{1});
                elseif length(varargin) == 2 && isa(varargin{1}, 'double') && isa(varargin{2}, 'double')
                    % Grab within a time range
                    t_start = varargin{1};
                    t_end = varargin{2};

                    indices = and(this.log.(channel).Time >= t_start, this.log.(channel).Time <= t_end);
                    x = this.log.(channel).Value(indices);
                else
                    error('Cannot parse inputs for getting channel');
                end
            end
        end

        % getChannelUnits
        %
        % Access for any channel untis by name.
        %
        % This uses the internal channel names stored in the channel name mapping, not
        % the channel names originally present in the log data.
        %
        % INPUTS:
        %   channel: Internal name of channel units to retrieve
        % OUTPUTS:
        %   units: Channel units
        function units = getChannelUnits(this, channel)
            if isKey(this.channel_map, channel)
                source_channel_name = this.channel_map(channel);
                units = this.getChannelUnitsNoRemap(source_channel_name);
            else
                units = '';
            end
        end

        % getChannelUnitsNoRemap
        %
        % Version of getChannelUnits() that does not remap channel names, this directly
        % accesses the log for the channel name provided.
        function units = getChannelUnitsNoRemap(this, channel)
            if isfield(this.log, channel)
                units = this.log.(channel).Units;
            else
                units = '';
                return;
            end
        end

        % addChannelData
        %
        % Adds/updates data for a channel
        %
        % INPUTS:
        %   name: Internal name of the channel
        %   x: Data points
        %   units: Units for x
        function this = addChannelData(this, name, x, units)
            if isKey(this.channel_map, name)
                this.log.(name).Time = this.getTimestamps();
                this.log.(name).Value = x;
                this.log.(name).Units = units;
            end
        end

        % smoothChannel
        %
        % Applies a smoothing operation to the data on a channel.
        %
        % INPUTS:
        %   channel: Internal name of channel to smooth
        %   window: Size of window (number of points) to apply smoothing over
        %   method: Name of the smoothing method ('movmean', 'movmedian', etc)
        function this = smoothChannel(this, channel, window, method)
            if isKey(this.channel_map, channel)
                channel = this.channel_map(channel);
                this = this.smoothChannelNoRemap(channel, window, method);
            end
        end

        % smoothChannelNoRemap
        %
        % Version of smoothChannel() that does not remap channel names, this directly
        % accesses the log for the channel name provided.
        function this = smoothChannelNoRemap(this, channel, window, method)
            if isfield(this.log, channel)
                this.log.(channel).Value = smoothdata(this.log.(channel).Value, method, window);
            end
        end

        % addChannelMappingEntry
        %
        % Adds an entry to the map for channel names so that this class can access
        % common data channels in the log.
        %
        % INPUTS:
        %   internal_name: Channel name used internally (see initializeChannelMap() for
        %       complete list)
        %   source_name: Name of the channel in the log data
        function this = addChannelMappingEntry(this, internal_name, source_name)
            this.channel_map(internal_name) = source_name;
        end

        % listInternalChannels
        %
        % Provides a list of all channels present in the mapping
        %
        % OUTPUTS:
        %   channels: Cell array of all channel names
        function channels = listInternalChannels(this)
            channels = keys(this.channel_map);
        end

        % listLogChannels
        %
        % Provides a list of all channels present in the log data
        %
        % OUTPUTS:
        %   channels: Cell array of all channel names
        function channels = listLogChannels(this)
            channels = fieldnames(this.log);
        end

        % defineChannelMap
        %
        % Method to be implemented by derived classes to set all channel name mappings.
        %
        % Will be called when instantiated or a new log is loaded.
        function this = defineChannelMap(this)
        end

        % preProcessLogData
        %
        % Method to be implemented by derived classes to pre-process log data to apply any
        % necessary data conversions.
        %
        % Will be called when instantiated or a new log is loaded.
        function this = preProcessLogData(this)
        end
    end

    methods (Static)
        % thresholdIndices
        %
        % Extracts all indices from a vector that abides by some range and rate limits.
        %
        % INPUTS:
        %   x: Data to evaluate
        %   dt: Sample rate [s]
        %   min: Minimum allowed value (inclusive)
        %   max: Maximum allowed value (inclusive)
        %   x_dot_min: Minimum allowed value of the derivative of x
        %   x_dot_max: Maximum allowed value of the derivative of x
        % OUTPUTS:
        %   indices: Indices of the x vector that abides by all limits
        function indices = thresholdIndices(x, dt, x_min, x_max, x_dot_min, x_dot_max)
            min_inds = x >= x_min;
            max_inds = x <= x_max;

            x_dot = [diff(x), 0] / dt;
            x_dot_min_inds = x_dot >= x_dot_min;
            x_dot_max_inds = x_dot <= x_dot_max;

            indices = and(min_inds, and(max_inds, and(x_dot_min_inds, x_dot_max_inds)));
        end

        % trimLogicalEdges
        %
        % Performs an erosion operation on a logical data vector.
        %
        % INPUTS:
        %   x, n: x is the logical data vector to trim (true values are trimmed), n is
        %     the number of steps to trim on each side
        %   x, dt, dt_trim: x is the logical data vector, dt is the time step size of the
        %     data, and dt_trim is how much to trim on each side of the data
        % OUTPUTS:
        %   x: Trimmed version of x
        function x = trimLogicalEdges(varargin)
            x = varargin{1};

            n = 0;
            if length(varargin) == 2
                n = varargin{2};
            elseif length(varargin) == 3
                n = ceil(varargin{3} / varargin{2});
            end

            if n > 0
                % Apply a convolution to the indices to identify which ones are neighboured
                % by false values

                % Unity  kernel extending on each side the size of the buffer
                kernel = ones(1, 2 * n + 1);

                % Apply the convolution, then look for entries that were true for every element of
                % the kernel
                x = conv(x, kernel, 'same');
                x = x == length(kernel);
            end
        end

        % detectConditionPeriod
        %
        % Detects the periods in a data vector when some condition is true for at least
        % some period of time
        %
        % Example:
        %   x = [1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0];
        %   dt = 1.0
        %   dt_min = 3.0
        %
        %   The elements of x where it is true for at least 3 seconds are:
        %     [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0]
        %
        % INPUTS:
        %   x: Logical data vector to evaluate
        %   dt: Time step of data in x
        %   dt_min: Minimum amount of time for points in x to be true
        % OUTPUTS:
        %   x: Version of x where only elements that remain true for some period of time
        %     are left as true
        function x = detectConditionPeriod(x, dt, dt_min)
            % Detect periods where the condition is true by performing an erosion/closing
            % operation followed by a dilation/opening operation. The erosion will leave
            % only the points where the condition was true for the required period of
            % time, then the dilation operation will expand those periods back to their
            % original size.

            % Need a kernel that covers the required time window
            kernel_size = ceil(dt_min / dt);
            kernel_size = 2 * floor(kernel_size / 2) + 1;
            kernel = ones(1, kernel_size);

            % Erode, look for elements where the full kernel was touching true values
            x = conv(x, kernel, 'same');
            x = x == kernel_size;

            % Dilate, look for any points where the kernel touched a true value
            x = conv(x, kernel, 'same');
            x = x > 0;
        end

        % smoothData
        %
        % Performs a smoothing operation on a data vector.
        %
        % INPUTS:
        %   x, n: x is the data vector to smooth, n is window size to smooth over
        %   x, dt, dt_smooth: x is the data vector to smooth, dt is the time step size,
        %     dt_smooth is the time window to smooth over
        % OUTPUTS:
        %   x: Smoothed data
        function x = smoothData(varargin)
            x = varargin{1};

            n = 0;
            if length(varargin) == 2
                n = varargin{2};
            elseif length(varargin) == 3
                dt = varargin{2};
                dt_smooth = varargin{3};
                n = ceil(dt_smooth / dt);
            end

            if n > 0
                x = smoothdata(x, 'movmean', n);
            end
        end

        % shiftData
        %
        % Performs an erosion operation on a logical data vector.
        %
        % INPUTS:
        %   x, n: x is the logical data vector to trim (true values are trimmed), n is
        %     the number of steps to shift the data by (+ve pads beginning of vector)
        %   x, dt, dt_shift: x is the logical data vector, dt is the time step size of the
        %     data, and dt_shift is how much to time to shift the data by (+ve pads the
        %     beginning of the vector)
        % OUTPUTS:
        %   x: Time shifted data
        function x = shiftData(varargin)
            x = varargin{1};

            n = 0;
            if length(varargin) == 2
                n = varargin{2};
            elseif length(varargin) == 3
                n = ceil(varargin{3} / varargin{2});
            end

            if n > 0
                x = [x(1) * ones(1, n), x(1:end - n)];
            elseif n < 0
                n = -n;
                x = [x(1 + n:end), x(end) * ones(1, n)];
            end
        end
    end

    methods (Access = protected)
        % initializeChannelMap
        %
        % Populates an empty mapping of all common channels used:
        %   INTERNAL_NAME --> SOURCE_NAME
        % Where SOURCE_NAME is the name of channels in the log data.
        %
        % Map entries will be created for all channels listed in the description at the
        % top.
        function this = initializeChannelMap(this)
            this.channel_map = containers.Map('KeyType', 'char', 'ValueType', 'char');

            this = this.addChannelMappingEntry('LAP_NUMBER', '');
            this = this.addChannelMappingEntry('LAP_DISTANCE', '');
            this = this.addChannelMappingEntry('GROUND_SPEED', '');
            this = this.addChannelMappingEntry('AIR_DYN_PRESSURE', '');
            this = this.addChannelMappingEntry('AX', '');
            this = this.addChannelMappingEntry('AY', '');
            this = this.addChannelMappingEntry('AZ', '');
            this = this.addChannelMappingEntry('WX', '');
            this = this.addChannelMappingEntry('WY', '');
            this = this.addChannelMappingEntry('WZ', '');
            this = this.addChannelMappingEntry('ELEVATION', '');
            this = this.addChannelMappingEntry('ENGINE_RPM', '');
            this = this.addChannelMappingEntry('FUEL_LEVEL', '');
            this = this.addChannelMappingEntry('THROTTLE', '');
            this = this.addChannelMappingEntry('BRAKE', '');
            this = this.addChannelMappingEntry('CLUTCH', '');
            this = this.addChannelMappingEntry('STEERING_WHEEL', '');
            this = this.addChannelMappingEntry('DAMPER_FL', '');
            this = this.addChannelMappingEntry('DAMPER_FR', '');
            this = this.addChannelMappingEntry('DAMPER_RL', '');
            this = this.addChannelMappingEntry('DAMPER_RR', '');
            this = this.addChannelMappingEntry('WS_FL', '');
            this = this.addChannelMappingEntry('WS_FR', '');
            this = this.addChannelMappingEntry('WS_RL', '');
            this = this.addChannelMappingEntry('WS_RR', '');
            this = this.addChannelMappingEntry('WS_ROT_FL', '');
            this = this.addChannelMappingEntry('WS_ROT_FR', '');
            this = this.addChannelMappingEntry('WS_ROT_RL', '');
            this = this.addChannelMappingEntry('WS_ROT_RR', '');
            this = this.addChannelMappingEntry('TIRE_P_FL', '');
            this = this.addChannelMappingEntry('TIRE_P_FR', '');
            this = this.addChannelMappingEntry('TIRE_P_RL', '');
            this = this.addChannelMappingEntry('TIRE_P_RR', '');
            this = this.addChannelMappingEntry('TIRE_T_FL', '');
            this = this.addChannelMappingEntry('TIRE_T_FR', '');
            this = this.addChannelMappingEntry('TIRE_T_RL', '');
            this = this.addChannelMappingEntry('TIRE_T_RR', '');
        end
    end
end
