% SessionAnalysis
%
% Utility for analyzing and viewing data stored in MotecHandler's.
%
% This provides functionality for:
%   - Manages a dataset which multiple batches of data can be added to, they are stored
%     individually but can be combined together
%   - Applying filters to channels
%   - Plotting channels, with options of override plot attributes
classdef SessionAnalysis
    properties (Access = public)
        % Structure of all data channels, which are cell arrays with once cell
        % for every dataset
        datasets;
        % Structure of data channels that have been post processed, which are cell arrays
        % with one cell for every dataset
        proc_datasets;
        % Structure of strings defining units for each channel
        units;
        % Structure with fields for each channel containing filtering parameter values
        % (dt_smooth, dt_shift)
        filtering_params;
        % Structure with fields for each channel containing optional plot settings
        plot_overrides;
    end

    properties (Access = protected)
        % Count of how many datasets are present
        n_datasets = 0;
    end

    methods
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

            t_offset = -t(1);
            if this.n_datasets > 1
                t_offset = -t(1) + this.datasets.TIME{this.n_datasets - 1}(end) + (t(2) - t(1));
            end
            this.datasets.TIME{this.n_datasets} = t + t_offset;

            % Extract all data channels in the handler
            channels = handler.listInternalChannels();
            for i = 1 : length(channels)
                name = channels{i};
                this.datasets.(name){this.n_datasets} = handler.getChannel(name);
                this = this.applyFilteringToData(name, this.n_datasets);
            end

            % Extract units - only from the first dataset provided
            if this.n_datasets == 1
                this.units.TIME = 's';

                for i = 1 : length(channels)
                    this.units.(channels{i}) = handler.getChannelUnits(channels{i});
                end
            end
        end

        % postProcess
        %
        % Performs some post processing on the data and writes the results to the
        % proc_datasets member variable.
        %
        % To be implemented by derived classes.
        function this = postProcess(this)
            this.proc_datasets = this.datasets;
        end

        % concatenateData
        %
        % Combines data from all datasets.
        %
        % OUTPUTS:
        %   all_data: Struct with a field for ever channel with data from each dataset
        %     concatenated together
        function all_data = concatenateData(this)
            all_data = struct;

            fields = fieldnames(this.datasets);
            for i = 1 : length(fields)
                channel = fields{i};
                all_data.(channel) = cell2mat(this.datasets.(channel));
            end
        end

        % concatenateProcData
        %
        % Combines data from all post-processed datasets
        %
        % OUTPUTS:
        %   all_data: Struct with a field for ever channel with data from each dataset
        %     concatenated together
        function all_data = concatenateProcData(this)
            all_data = struct;

            fields = fieldnames(this.proc_datasets);
            for i = 1 : length(fields)
                channel = fields{i};
                all_data.(channel) = cell2mat(this.proc_datasets.(channel));
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

        % addPlottingOverride
        %
        % When added these settings will be applied anytime a channel is plotted.
        %
        % Options are:
        %   scale: double
        %   bias: double
        %   units: char
        %   ylim: 1x2 vector
        % INPUTS:
        %   channel, key, value: channel is the name of the channel the override applies
        %     to, key is the name of the parameter, value is value for the parameter
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
        %  x_channel, y_channel, spec: x_channel and y_channel are the X and Y axis
        %    channels to plot and spec is the line spec to use (e.g. ':b'), a new figure
        %    will be created,
        %  channel, spec, fig: fig is the figure handle to set as the current figure
        %  channel, spec, fig, subplot_id: subplot_id is the index of the subplot to
        %    make current
        function fig = plotData(this, varargin)
            x = cell2mat(this.datasets.(varargin{1}));
            y = cell2mat(this.datasets.(varargin{2}));
            fig = this.plotDataSingleSeries(x, y, varargin{:});
        end

        % plotTimeData
        %
        % Same as plotData() except the X axis series is Time
        function fig = plotTimeData(this, varargin)
            t = cell2mat(this.datasets.TIME);
            y = cell2mat(this.datasets.(varargin{1}));

            args = {'TIME', varargin{:}};
            fig = this.plotDataSingleSeries(t, y, args{:});
        end

        % plotProcData
        %
        % Plots post processed data.
        %
        % Variation of plotData() that uses 'proc_datasets' instead of 'datasets'. Input
        % arguments are the same.
        function fig = plotProcData(this, varargin)
            x = cell2mat(this.proc_datasets.(varargin{1}));
            y = cell2mat(this.proc_datasets.(varargin{2}));
            fig = this.plotDataSingleSeries(x, y, varargin{:});
        end

        % plotProcTimeData
        %
        % Plots post processed data.
        %
        % Same as plotProcData() except the X axis series is Time
        function fig = plotProcTimeData(this, varargin)
            t = cell2mat(this.proc_datasets.TIME);
            y = cell2mat(this.proc_datasets.(varargin{1}));

            args = {'TIME', varargin{:}};
            fig = this.plotDataSingleSeries(t, y, args{:});
        end

    end

    methods (Access = protected)
        % applyFilteringToData
        %
        % Applies all filters to a channel in datasets
        %
        % INPUTS:
        %   channel: Name of the channel to smooth
        %   n: Dataset number to operate on
        function this = applyFilteringToData(this, channel, n)
            if isfield(this.datasets, channel) && isfield(this.filtering_params, channel)
                params = this.filtering_params.(channel);
                x = this.datasets.(channel){n};
                dt = this.datasets.TIME{n}(2) - this.datasets.TIME{n}(1);

                if isfield(params, 'dt_shift')
                    x = MotecHandler.shiftData(x, dt, params.dt_shift);
                end
                if isfield(params, 'dt_smooth')
                    x = MotecHandler.smoothData(x, dt, params.dt_smooth);
                end

                this.datasets.(channel){n} = x;
            end
        end

        % plotDataSingleSeries
        %
        % Plots a single series of data for a channel. If an plotting override has been
        % added for this channel then that scaling will be applied and the override units
        % will be shown.
        %
        % INPUTS:
        %  x, y, x_channel, y_channel, spec: x is the X axis series, y is the Y axis
        %    series, x_channel s the name of the channel for x data, y_channel s the name
        %    of the channel for y data, and spec is the line spec to use (e.g. ':b'),
        %    a new figure will be created,
        %  channel, spec, fig: fig is the figure handle to set as the current figure
        %  channel, spec, fig, subplot_id: subplot_id is the index of the subplot to
        %    make current
        function fig = plotDataSingleSeries(this, varargin)
            x = varargin{1};
            y = varargin{2};
            x_channel = varargin{3};
            y_channel = varargin{4};
            spec = varargin{5};

            init_args = varargin(6:end);
            fig = this.initFigure(init_args{:});

            %  Extract any plotting overrides if any are present
            x_overrides = this.getChannelPlotOverrides(x_channel);
            y_overrides = this.getChannelPlotOverrides(y_channel);

            % Remove any underscores in names so they don't appear as subscripts
            legend_name = strrep(y_channel, '_', ' ');

            hold on;
            plot(x * x_overrides.scale + x_overrides.bias, y * y_overrides.scale + y_overrides.bias, spec, 'DisplayName', legend_name);
            grid on;
            legend(gca, 'show');
            xlabel(sprintf('[%s]', x_overrides.units));
            ylabel(sprintf('[%s]', y_overrides.units));
            if ~isempty(x_overrides.axis_lim)
                ylim(x_overrides.axis_lim);
            end
            if ~isempty(y_overrides.axis_lim)
                ylim(y_overrides.axis_lim);
            end
        end

        function overrides = getChannelPlotOverrides(this, channel)
            overrides.scale = 1;
            overrides.bias = 0;
            overrides.units = this.units.(channel);
            overrides.axis_lim = [];

            if isfield(this.plot_overrides, channel)
                if isfield(this.plot_overrides.(channel), 'scale')
                    overrides.scale = this.plot_overrides.(channel).scale;
                end
                if isfield(this.plot_overrides.(channel), 'bias')
                    overrides.bias = this.plot_overrides.(channel).bias;
                end
                if isfield(this.plot_overrides.(channel), 'units')
                    overrides.units = this.plot_overrides.(channel).units;
                end
                if isfield(this.plot_overrides.(channel), 'lim')
                    overrides.axis_lim = this.plot_overrides.(channel).lim;
                end
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
