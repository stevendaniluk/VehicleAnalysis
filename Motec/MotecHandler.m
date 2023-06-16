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
%   GROUND_SPEED
%   AIR_DYN_PRESSURE
%   AX
%   AY
%   AZ
%   WX
%   WY
%   WZ
%   ENGINE_RPM
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
            fields = fieldnames(this.log);
            for i=1:length(fields)
                indices = ...
                    and(this.log.(fields{i}).Time >= t_start, this.log.(fields{i}).Time <= t_end);
                this.log.(fields{i}).Time = this.log.(fields{i}).Time(indices);
                this.log.(fields{i}).Value = this.log.(fields{i}).Value(indices);
            end
        end

        % trimTimeStamps
        %
        % Trims the log data to only include specified time stamps
        %
        % INPUTS:
        %   stamps: Time stamps to include
        function this = trimTimeStamps(this, stamps)
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

            this = this.addChannelMappingEntry('GROUND_SPEED', '');
            this = this.addChannelMappingEntry('AIR_DYN_PRESSURE', '');
            this = this.addChannelMappingEntry('AX', '');
            this = this.addChannelMappingEntry('AY', '');
            this = this.addChannelMappingEntry('AZ', '');
            this = this.addChannelMappingEntry('WX', '');
            this = this.addChannelMappingEntry('WY', '');
            this = this.addChannelMappingEntry('WZ', '');
            this = this.addChannelMappingEntry('ENGINE_RPM', '');
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
