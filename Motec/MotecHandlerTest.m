classdef MotecHandlerTest < matlab.unittest.TestCase
    properties
        sample_log_path = 'test_assets/simple_motec_log.mat';
        sample_log;
        sample_mapping;

        % Instance of the handler complete with data and mappings
        configured_handler;
    end

    methods (TestMethodSetup)
        function constructLogData(this)
            this.sample_log = createSimpleMotecLog(true, this.sample_log_path);

            % Create the example handler with data and add mappings
            this.configured_handler = MotecHandler(this.sample_log_path);
            this.configured_handler.addChannelMappingEntry('THROTTLE', 'throttle_pedal');
            this.configured_handler.addChannelMappingEntry('BRAKE', 'brake_pedal');
            this.configured_handler.addChannelMappingEntry('ENGINE_RPM', 'rpm_eng');
        end
    end

    methods (Test)
        function loadFromFile(this)
            handler = MotecHandler(this.sample_log_path);
            channels = handler.listLogChannels();
            this.verifyNotEmpty(channels);
        end

        function loadFromStruct(this)
            handler = MotecHandler(this.sample_log);
            channels = handler.listLogChannels();
            this.verifyNotEmpty(channels);
        end

        function channelNameMapping(this)
            % New handler will not have any mappings, so no data should be returned from
            % the query
            handler = MotecHandler(this.sample_log);
            throttle = handler.getChannel('THROTTLE');
            this.verifyEmpty(throttle);

            % Add the mapping, should get some data back now
            handler.addChannelMappingEntry('THROTTLE', 'throttle_pedal');
            throttle = handler.getChannel('THROTTLE');
            this.verifyNotEmpty(throttle);
        end

        function trimByTimeRange(this)
            % Determine the initial time range and some sample data to check as well
            t = this.configured_handler.getTimestamps();
            throttle = this.configured_handler.getChannel('THROTTLE');

            this.assertNotEmpty(t);
            this.assertNotEmpty(throttle);

            t_start = t(1);
            t_end = t(end);

            t_start_trimmed = t_start + 1.0;
            t_end_trimmed = t_end - 1.0;

            % Trim and extract new data
            this.configured_handler = this.configured_handler.trimTimeRange(t_start_trimmed, t_end_trimmed);
            t_trimmed = this.configured_handler.getTimestamps();
            throttle_trimmed = this.configured_handler.getChannel('THROTTLE');

            % Check the timestamps are within bounds and that the throttle data has also
            % been trimmed to the same length
            this.verifyGreaterThanOrEqual(t_trimmed, t_start_trimmed);
            this.verifyLessThanOrEqual(t_trimmed, t_end_trimmed);
            this.verifyLength(throttle_trimmed, length(t_trimmed));
        end

        function trimByTimestamps(this)
            % Determine the initial timestamps and some sample data to check as well
            t = this.configured_handler.getTimestamps();
            throttle = this.configured_handler.getChannel('THROTTLE');

            this.assertNotEmpty(t);
            this.assertNotEmpty(throttle);

            % Trim every other index
            indices = logical(ones(size(t)));
            indices(2:2:end) = false;

            this.configured_handler = this.configured_handler.trimIndices(indices);
            t_trimmed = this.configured_handler.getTimestamps();
            throttle_trimmed = this.configured_handler.getChannel('THROTTLE');

            this.verifyEqual(t_trimmed, t(indices));
            this.verifyEqual(throttle_trimmed, throttle(indices));
        end

        function timestampsAccessorVariations(this)
            t_ref = this.sample_log.throttle_pedal.Time;

            % No arguments should return all data
            t_no_arg = this.configured_handler.getTimestamps();
            this.verifyEqual(t_no_arg, t_ref);

            % Provide min and max time ranges (inclusive)
            t_start_trimmed = t_ref(1) + 1.0;
            t_end_trimmed = t_ref(end) - 1.0;
            t_range = this.configured_handler.getTimestamps(t_start_trimmed, t_end_trimmed);

            this.verifyGreaterThanOrEqual(t_range, t_start_trimmed);
            this.verifyLessThanOrEqual(t_range, t_end_trimmed);

            % Provide timestamp indices to keep
            indices = logical(ones(size(t_ref)));
            indices(2:2:end) = false;

            t_indices = this.configured_handler.getTimestamps(indices);
            this.verifyEqual(t_indices, t_ref(indices));
        end

        function getChannelAccessorVariations(this)
            t_ref = this.sample_log.throttle_pedal.Time;
            chan_ref = this.sample_log.throttle_pedal.Value;
            chan_name = 'THROTTLE';

            % No arguments should return all data
            chan_no_arg = this.configured_handler.getChannel(chan_name);
            this.verifyEqual(chan_no_arg, chan_ref);

            % Provide min and max time ranges (inclusive)
            t_start_trimmed = t_ref(1) + 1.0;
            t_end_trimmed = t_ref(end) - 1.0;
            chan_range = this.configured_handler.getChannel(chan_name, t_start_trimmed, t_end_trimmed);

            range_indices = and(t_ref >= t_start_trimmed, t_ref <= t_end_trimmed);
            this.verifyEqual(chan_range, chan_ref(range_indices));

            % Provide timestamp indices to keep
            indices = logical(ones(size(chan_ref)));
            indices(2:2:end) = false;

            chan_indices = this.configured_handler.getChannel(chan_name, indices);
            this.verifyEqual(chan_indices, chan_ref(indices));
        end
    end
end
