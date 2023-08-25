classdef SessionAnalysis
    properties (Access = public)
        vehicle;

        datasets;
        all_data;
        units;

        n_datasets = 0;
        filtering_params;
        plot_overrides;
    end

    methods
        % INPUTS:
        %   vehicle: Vehicle instance to compute data from
        function this = SessionAnalysis(vehicle)
            this.vehicle = vehicle;
        end

        % addDataset
        %
        % Extracts all channels from the provided handler and adds it to the collection
        % of datasets. This will also perform filtering for all channels which have
        % filtering parameters set.
        %
        % All data will be added to the 'datasets' struct under a field name matching the
        % channel name in the handler.
        %
        % INPUTS:
        %   handler: MotecHandler to extract data from
        function this = addDataset(this, handler)
            this.n_datasets = this.n_datasets + 1;

            % Extract the timestamps for the data - When adding multiple datasets we
            % alter the timestamps so that the this datasets begins immediately after
            % the previous dataset
            t = handler.getTimestamps();

            t_offset = 0;
            if this.n_datasets > 1
                t_offset = this.datasets.t{this.n_datasets - 1}(end);
                t_offset = t_offset + t(2) - t(1);
            end
            this.datasets.t{this.n_datasets} = t + t_offset;

            % Extract all data channels in the handler
            channels = handler.listInternalChannels();
            for i = 1 : length(channels)
                name = channels{i};
                this.datasets.(name){this.n_datasets} = handler.getChannel(name);
                this = this.applyFilteringToData(name, this.n_datasets);
            end

            % Extract units - only from the first dataset provided
            if this.n_datasets == 1
                for i = 1 : length(channels)
                    this.units.(channels{i}) = handler.getChannelUnits(channels{i});
                end
            end
        end

        % concatenateData
        %
        % Combines all datasets for each channel into one. The result will be written to
        % the 'all_data' member variable.
        function this = concatenateData(this)
            fields = fieldnames(this.datasets);
            for i = 1 : length(fields)
                channel = fields{i};
                this.all_data.(channel) = cell2mat(this.datasets.(channel));
            end
        end

        % addFilteringParams
        %
        % Defines some filtering operations to apply to a channel. These will be applied
        % when adding a dataset.
        %
        % INPUTS:
        %   dt_smooth: Time interval to apply a moving mean filter over [s]
        %   dt_shift: Duration to shift the data (+ve shifts points forward)
        function this = addFilteringParams(this, name, dt_smooth, dt_shift)
            this.filtering_params.(name).dt_smooth = dt_smooth;
            this.filtering_params.(name).dt_shift = dt_shift;
        end

        % function this = addPlottingOverride(this, channel, scale, units)
        %     this.plot_overrides.(channel).scale = scale;
        %     this.plot_overrides.(channel).units = units;
        % end

        function this = addPlottingOverride(this, varargin)
            channel = varargin{1};

            n = 2;
            while length(varargin) >= n + 1
                field = varargin{n};
                value = varargin{n + 1};
                this.plot_overrides.(channel).(field) = value;
                n = n + 2;
            end
        end

        % plotData
        %
        % Plots all data for a particular channel. If an plotting override has been added
        % for this channel then that scaling will be applied and the override units
        % will be shown.
        %
        % INPUTS:
        %  channel, spec: channel name to plot, a new figure will be created, and spec is
        %    the line spec to use (e.g. ':b')
        %  channel, spec, fig: fig is the figure handle to set as the current figure
        %  channel, spec, fig, subplot_id: subplot_id is the index of the subplot to make current
        function fig = plotData(this, varargin)
            t = cell2mat(this.datasets.t);
            x = cell2mat(this.datasets.(varargin{1}));
            fig = this.plotDataSingleSeries(t, x, varargin{:});
        end

    end

    methods (Access = protected)
        function this = applyFilteringToData(this, field, set)
            if isfield(this.datasets, field) && isfield(this.filtering_params, field)
                params = this.filtering_params.(field);
                x = this.datasets.(field){set};
                dt = this.datasets.t{set}(2) - this.datasets.t{set}(1);

                if isfield(params, 'dt_shift')
                    x = MotecHandler.shiftData(x, dt, params.dt_shift);
                end
                if isfield(params, 'dt_smooth')
                    x = MotecHandler.smoothData(x, dt, params.dt_smooth);
                end

                this.datasets.(field){set} = x;
            end
        end

        % plotDataSingleSeries
        %
        % Plots a single series of data for a channel. If an plotting override has been
        % added for this channel then that scaling will be applied and the override units
        % will be shown.
        %
        % INPUTS:
        %  t, x, channel, spec: t is the time series, x is the data series, channel is the
        %    name of the channel to plot, a new figure will be created, and spec is the
        %    line spec to use (e.g. ':b')
        %  channel, spec, fig: fig is the figure handle to set as the current figure
        %  channel, spec, fig, subplot_id: subplot_id is the index of the subplot to make current
        function fig = plotDataSingleSeries(this, varargin)
            t = varargin{1};
            x = varargin{2};
            channel = varargin{3};
            spec = varargin{4};

            init_args = varargin(5:end);
            fig = this.initFigure(init_args{:});

            %  Extract any plotting overrides if any are present
            scale = 1;
            bias = 0;
            unit_name = this.units.(channel);
            y_limits = [];
            if isfield(this.plot_overrides, channel)
                if isfield(this.plot_overrides.(channel), 'scale')
                    scale = this.plot_overrides.(channel).scale;
                end
                if isfield(this.plot_overrides.(channel), 'bias')
                    bias = this.plot_overrides.(channel).bias;
                end
                if isfield(this.plot_overrides.(channel), 'units')
                    unit_name = this.plot_overrides.(channel).units;
                end
                if isfield(this.plot_overrides.(channel), 'ylim')
                    y_limits = this.plot_overrides.(channel).ylim;
                end
            end

            % Remove any underscores in names so they don't appear as subscripts
            legend_name = strrep(channel, '_', ' ');

            hold on;
            plot(t, bias + scale * x, spec, 'DisplayName', legend_name);
            grid on;
            legend(gca, 'show');
            xlabel('Time [s]');
            ylabel(sprintf('[%s]', unit_name));
            if ~isempty(y_limits)
                ylim(y_limits);
            end
        end

    end

    methods (Static, Access = protected)

        % initFigure
        %
        % Helper function to handle creating a figure or setting a provided figure as
        % current, as well as setting the subplot.
        %
        % INPUTS:
        %   None: Creates a new figure
        %   fig: fig is the figure handle to set as the current figure
        %   fig, subplot_id, fig is the figure handle, and subplot_id is the index of the
        %     subplot to make current
        function fig = initFigure(varargin)
            if ~isempty(varargin) > 0
                fig = varargin{1};
                set(0, 'currentfigure', fig);
            else
                fig = figure;
            end

            if length(varargin) == 2
                % Set subplot
                subplot(varargin{2});
            end
        end

    end
end
