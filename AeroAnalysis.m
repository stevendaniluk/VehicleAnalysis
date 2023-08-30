classdef AeroAnalysis < SessionAnalysis
    properties (Access = public)
        % Instance of "Vehicle" class to use for computations
        vehicle;
        % Structure with all computed aerodynamic forces and properties
        F_aero;
        % Structure of data with fields for every channel and computed property that is
        % a subset of "all_data", it is data that passes all filtering conditions
        analysis_data;
    end

    properties (Access = protected)
        % List of track zones, defined by a start and end lap distance, that will be
        % considered as valid data points (empty means all data is valid)
        lap_zones = {};
        % Cell array, one for each dataset, containing logical indices indicating which
        % points in each dataset are considered valid
        valid_dataset_pts;
        % Logical indices of "all_data" defining which data is considered valid
        valid_all_data_pts;
    end

    methods
        % INPUTS:
        %   vehicle: Vehicle instance to compute data from
        function this = AeroAnalysis(vehicle)
            this.vehicle = vehicle;
        end

        % addDataset
        %
        % Extracts all channels from the provided handler and adds it to the collection
        % of datasets. This will also perform filtering for all channels which have
        % filtering parameters set.
        %
        % All data will be added to the 'data' struct under a field name matching the
        % channel name in the handler.
        %
        % INPUTS:
        %   handler: MotecHandler to extract data from
        %   rho_air: Air density [kg/m^3]
        function this = addDataset(this, handler, rho_air)
            this = addDataset@SessionAnalysis(this, handler);
            n = this.n_datasets;
            dt = this.datasets.TIME{n}(2) - this.datasets.TIME{n}(1);

            % Record the air density
            this.datasets.RHO_AIR{n} = rho_air * ones(size(this.datasets.TIME{n}));

            % Compute air speed
            this.units.AIR_SPEED = 'm/s';
            this.datasets.AIR_SPEED{n} = ...
                sqrt(2 * this.datasets.AIR_DYN_PRESSURE{n} / rho_air);
            this = this.applyFilteringToData('AIR_SPEED', n);

            % Estimate vertical velocity from elevation
            this.units.VZ = 'm/s';
            VZ_end = this.datasets.ELEVATION{n}(end) - this.datasets.ELEVATION{n}(end - 1);
            this.datasets.VZ{n} = -[diff(this.datasets.ELEVATION{n}), VZ_end] / dt;
            this = this.applyFilteringToData('VZ', n);

            % Estimate road grade angle from vertical velocity and ground speed
            % Smooth the ground speed a bit more before using
            this.units.ROAD_GRADE = 'rad';

            dt_smooth_v_ground = 1.0;
            v_ground_smooth = smoothdata(this.datasets.GROUND_SPEED{n}, ...
                'movmean', dt_smooth_v_ground / dt + 1);

            this.datasets.ROAD_GRADE{n} = atan2(this.datasets.VZ{n}, v_ground_smooth);

            grade_v_min = 20 / 3.6;
            this.datasets.ROAD_GRADE{n}(v_ground_smooth < grade_v_min) = 0;

            this = this.applyFilteringToData('ROAD_GRADE', n);

            % Estimate vertical acceleration from elevation data, accounting for road grade
            this.units.AZ = 'm/s^2';
            AZ_end = this.datasets.VZ{n}(end) - this.datasets.VZ{n}(end - 1);
            this.datasets.AZ{n} = [diff(this.datasets.VZ{n}), AZ_end] / dt;

            g_road = this.vehicle.gravity * cos(this.datasets.ROAD_GRADE{n});
            this.datasets.AZ{n} = this.datasets.AZ{n} - g_road;

            this = this.applyFilteringToData('AZ', n);

            % Compute suspension pitch and rate
            this.units.PITCH_SUSP = 'rad';
            [RH_f, RH_r] = this.vehicle.avgRideHeightFromDamperPos(...
                this.datasets.DAMPER_FL{n}, ...
                this.datasets.DAMPER_FR{n}, ...
                this.datasets.DAMPER_RL{n}, ...
                this.datasets.DAMPER_RR{n});
            this.datasets.PITCH_SUSP{n} = this.vehicle.pitchFromAvgRideHeights(RH_f, RH_r);
            this = this.applyFilteringToData('PITCH_SUSP', n);

            this.units.WY_SUSP = 'rad/s';
            WY_susp_end = ...
                this.datasets.PITCH_SUSP{n}(end) - this.datasets.PITCH_SUSP{n}(end - 1);
            this.datasets.WY_SUSP{n} = [diff(this.datasets.PITCH_SUSP{n}), WY_susp_end] / dt;
            this = this.applyFilteringToData('WY_SUSP', n);

            % Adjust IMU pitch rate to account for suspension pitch motion
            this.datasets.WY{n} = this.datasets.WY{n} - this.datasets.WY_SUSP{n};

            this = this.computeAeroForcesForDataset(n);
        end

        % addLapZone
        %
        % Defines a zone along the lap in which data is considered valid. Multiple zones
        % can be added.
        %
        % INPUTS:
        %   d_start: Lap distance at which data is valid
        %   d_end: Lap distance art which data stops being valid
        function this = addLapZone(this, d_start, d_end)
            n = length(this.lap_zones) + 1;
            this.lap_zones{n} = [d_start, d_end];
        end

        function this = extractAnalysisData(this)
            % Go through every dataset finding the data points that are valid for each
            % condition we care about
            this.valid_dataset_pts = cell(1, this.n_datasets);
            for i = 1 : this.n_datasets
                this.valid_dataset_pts{i} = true(1, length(this.datasets.TIME{i}));
                dt = this.datasets.TIME{i}(2) - this.datasets.TIME{i}(1);

                % Found zones on the track where we want to process data
                if ~isempty(this.lap_zones)
                    zone_pts = false(size(this.valid_dataset_pts{i}));
                    for j = 1 : length(this.lap_zones)
                        d = this.lap_zones{j};

                        zone_pts_new = MotecHandler.thresholdIndices(...
                        this.datasets.LAP_DISTANCE{i}, dt, d(1), d(2), -inf, inf);
                        zone_pts = or(zone_pts_new, zone_pts);
                    end
                    this.valid_dataset_pts{i} = and(zone_pts, this.valid_dataset_pts{i});
                end

                % Find when suspension pitch rate is low
                wy_susp_min = deg2rad(-1.0);
                wy_susp_max = deg2rad(0.08);
                wy_susp_trim_dt = 1.0;

                wy_susp_low_pts = MotecHandler.thresholdIndices(...
                    this.datasets.WY_SUSP{i}, dt, wy_susp_min, wy_susp_max, -inf, inf);
                wy_susp_low_pts = MotecHandler.trimLogicalEdges(wy_susp_low_pts, ...
                    dt, wy_susp_trim_dt);
                this.valid_dataset_pts{i} = and(wy_susp_low_pts, this.valid_dataset_pts{i});
            end

            this.valid_all_data_pts = cell2mat(this.valid_dataset_pts);

            % Extract only the relevant fields for each channel from each dataset, then
            % concatenate all datasets together
            fields = fieldnames(this.datasets);
            for i = 1 : length(fields)
                channel = fields{i};

                for n = 1 : this.n_datasets
                    if ~isempty(this.datasets.(channel){n})
                        this.analysis_data.(channel){n} = ...
                            this.datasets.(channel){n}(this.valid_dataset_pts{n});

                        % if strcmp(channel, 'TIME')
                        %     % Recompute the time values to make all data points sequential
                        %     dt = this.analysis_data.TIME{n}(2) - ...
                        %         this.analysis_data.TIME{n}(1);
                        %
                        %     num_pts = length(this.analysis_data.TIME{n});
                        %     this.analysis_data.TIME{n} = 0:dt:(dt * (num_pts - 1));
                        %
                        %     if n > 1
                        %         % Shift forward to come after the previous dataset
                        %         t_offset = dt + this.analysis_data.TIME{n - 1}(end);
                        %         this.analysis_data.TIME{n} = ...
                        %             this.analysis_data.TIME{n} + t_offset;
                        %     end
                        % end
                    else
                        this.analysis_data.(channel){n} = [];
                    end
                end
                this.analysis_data.(channel) = cell2mat(this.analysis_data.(channel));
            end

            % Overwrite time to make all data points sequential
        end

        % plotAnalysisData
        %
        % Variation of plotData() that uses 'analysis_data' instead of 'data'. Input
        % arguments are the same.
        function fig = plotAnalysisData(this, varargin)
            x = this.analysis_data.(varargin{1});
            y = this.analysis_data.(varargin{2});
            fig = this.plotDataSingleSeries(x, y, varargin{:});
        end

        % plotAnalysisTimeData
        %
        % Same as plotAnalysisData() except the X axis series is Time
        function fig = plotAnalysisTimeData(this, varargin)
            t = this.analysis_data.TIME;
            y = this.analysis_data.(varargin{1});

            args = {'TIME', varargin{:}};
            fig = this.plotDataSingleSeries(t, y, args{:});
        end

        % % plotDownforceVsSpeed
        % %
        % % INPUTS:
        % %   None: Creates a new figure
        % %   fig: fig is the figure handle to set as the current figure
        % %   fig, subplot_id, fig is the figure handle, and subplot_id is the index of the
        % %     subplot to make current
        % function plotDownforceVsSpeed(this, varargin)
        %     fig = this.initFigure(varargin{:});
        %
        % end
    end

    methods (Access = protected)

        function this = computeAeroForcesForDataset(this, n)
            % Get all required input data
            rho_air = this.datasets.RHO_AIR{n};
            v_air = this.datasets.AIR_SPEED{n};
            V_fuel = this.datasets.FUEL_LEVEL{n};
            x_FL = this.datasets.DAMPER_FL{n};
            x_FR = this.datasets.DAMPER_FR{n};
            x_RL = this.datasets.DAMPER_RL{n};
            x_RR = this.datasets.DAMPER_RR{n};

            % Concatenate accelerations
            a = [this.datasets.AX{n}; this.datasets.AY{n}; this.datasets.AZ{n}];

            % Compute ride height from damper positions and tire radius
            [RH_FL, RH_FR, RH_RL, RH_RR] = ...
                this.vehicle.rideHeightFromDamperPos(x_FL, x_FR, x_RL, x_RR);

            % Estimate aero loads
            [FL_aero, FR_aero, RL_aero, RR_aero] = ...
                this.vehicle.estimatedAeroWheelLoad(a, V_fuel, ...
                        RH_FL, RH_FR, RH_RL, RH_RR, ...
                        x_FL, x_FR, x_RL, x_RR, ...
                        0, 0, 0, 0);

            % Extract the force components
            this.datasets.F_AERO_SPRING_F{n} = FL_aero.spring + FR_aero.spring;
            this = this.applyFilteringToData('F_AERO_SPRING_F', n);
            this.units.F_AERO_SPRING_F = 'N';

            this.datasets.F_AERO_SPRING_R{n} = RL_aero.spring + RR_aero.spring;
            this = this.applyFilteringToData('F_AERO_SPRING_R', n);
            this.units.F_AERO_SPRING_R = 'N';

            this.datasets.F_AERO_SPRING{n} = FL_aero.spring + FR_aero.spring + ...
                RL_aero.spring + RR_aero.spring;
            this = this.applyFilteringToData('F_AERO_SPRING', n);
            this.units.F_AERO_SPRING = 'N';

            this.datasets.F_AERO_STATIC_F{n} = FL_aero.static + FR_aero.static;
            this = this.applyFilteringToData('F_AERO_STATIC_F', n);
            this.units.F_AERO_STATIC_F = 'N';

            this.datasets.F_AERO_STATIC_R{n} = RL_aero.static + RR_aero.static;
            this = this.applyFilteringToData('F_AERO_STATIC_R', n);
            this.units.F_AERO_STATIC_R = 'N';

            this.datasets.F_AERO_STATIC{n} = FL_aero.static + FR_aero.static + ...
                RL_aero.static + RR_aero.static;
            this = this.applyFilteringToData('F_AERO_STATIC', n);
            this.units.F_AERO_STATIC = 'N';

            this.datasets.F_AERO_ANTI_F{n} = FL_aero.anti + FR_aero.anti;
            this = this.applyFilteringToData('F_AERO_ANTI_F', n);
            this.units.F_AERO_ANTI_F = 'N';

            this.datasets.F_AERO_ANTI_R{n} = RL_aero.anti + RR_aero.anti;
            this = this.applyFilteringToData('F_AERO_ANTI_R', n);
            this.units.F_AERO_ANTI_R = 'N';

            this.datasets.F_AERO_ANTI{n} = FL_aero.anti + FR_aero.anti + ...
                RL_aero.anti + RR_aero.anti;
            this = this.applyFilteringToData('F_AERO_ANTI', n);
            this.units.F_AERO_ANTI = 'N';


            this.datasets.F_AERO_FRONT{n} = FL_aero.aero + FR_aero.aero;
            this = this.applyFilteringToData('F_AERO_FRONT', n);
            this.units.F_AERO_FRONT = 'N';

            this.datasets.F_AERO_REAR{n} = RL_aero.aero + RR_aero.aero;
            this = this.applyFilteringToData('F_AERO_REAR', n);
            this.units.F_AERO_REAR = 'N';

            this.datasets.F_AERO_TOTAL{n} = this.datasets.F_AERO_FRONT{n} + ...
                this.datasets.F_AERO_REAR{n};
            this = this.applyFilteringToData('F_AERO_TOTAL', n);
            this.units.F_AERO_TOTAL = 'N';

            this.datasets.AERO_BALANCE{n} = 100 * this.datasets.F_AERO_FRONT{n} ./ ...
                this.datasets.F_AERO_TOTAL{n};
            this = this.applyFilteringToData('AERO_BALANCE', n);
            this.units.AERO_BALANCE = '%';

            this.datasets.CL{n} = this.vehicle.aeroForceCoefficient(v_air, ...
                this.datasets.F_AERO_TOTAL{n}, rho_air);
            this = this.applyFilteringToData('CL', n);
            this.units.CL = '';
        end

    end

end
