classdef AeroAnalysis < SessionAnalysis
    properties (Access = public)
        lap_zones = {};

        analysis_data;
        valid_dataset_pts;
        valid_all_data_pts;

        F_aero;
    end

    methods

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
            dt = this.datasets.t{n}(2) - this.datasets.t{n}(1);

            % Record the air density
            this.datasets.RHO_AIR{n} = rho_air * ones(size(this.datasets.t{n}));

            % Compute air speed
            this.units.AIR_SPEED = 'm/s';
            this.datasets.AIR_SPEED{n} = sqrt(2 * this.datasets.AIR_DYN_PRESSURE{n} / rho_air);
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
            WY_susp_end = this.datasets.PITCH_SUSP{n}(end) - this.datasets.PITCH_SUSP{n}(end - 1);
            this.datasets.WY_SUSP{n} = [diff(this.datasets.PITCH_SUSP{n}), WY_susp_end] / dt;
            this = this.applyFilteringToData('WY_SUSP', n);

            % Adjust IMU pitch rate to account for suspension pitch motion
            this.datasets.WY{n} = this.datasets.WY{n} - this.datasets.WY_SUSP{n};

            this = this.computeAeroForcesForDataset(n);
        end

        function this = addLapZone(this, d_start, d_end)
            n = length(this.lap_zones) + 1;
            this.lap_zones{n} = [d_start, d_end];
        end

        function this = extractAnalysisData(this)
            % Go through every dataset finding the data points that are valid for each
            % condition we care about
            this.valid_dataset_pts = cell(1, this.n_datasets);
            for i = 1 : this.n_datasets
                this.valid_dataset_pts{i} = true(1, length(this.datasets.t{i}));
                dt = this.datasets.t{i}(2) - this.datasets.t{i}(1);

                % Found zones on the track where we want to process data
                zone_pts = false(size(this.valid_dataset_pts));
                for j = 1 : length(this.lap_zones)
                    d = this.lap_zones{j};

                    zone_pts_new = MotecHandler.thresholdIndices(this.datasets.LAP_DISTANCE{i}, ...
                    dt, d(1), d(2), -inf, inf);
                    zone_pts = or(zone_pts_new, zone_pts);
                end
                this.valid_dataset_pts{i} = and(zone_pts, this.valid_dataset_pts{i});

                % Find when suspension pitch rate is low
                wy_susp_min = deg2rad(-1.0);
                wy_susp_max = deg2rad(0.08);
                wy_susp_trim_dt = 1.0;

                wy_susp_low_pts = MotecHandler.thresholdIndices(this.datasets.WY_SUSP{i}, dt, wy_susp_min, wy_susp_max, -inf, inf);
                wy_susp_low_pts = MotecHandler.trimLogicalEdges(wy_susp_low_pts, dt, wy_susp_trim_dt);
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
                        this.analysis_data.(channel){n} = this.datasets.(channel){n}(this.valid_dataset_pts{n});
                    else
                        this.analysis_data.(channel){n} = [];
                    end
                end
                this.analysis_data.(channel) = cell2mat(this.analysis_data.(channel));
            end
        end

        % plotAnalysisData
        %
        % Variation of plotData() that uses 'analysis_data' instead of 'data'. Input
        % arguments are the same.
        function fig = plotAnalysisData(this, varargin)
            t = this.analysis_data.t;
            x = this.analysis_data.(varargin{1});
            fig = this.plotDataSingleSeries(t, x, varargin{:});
        end
    end

    methods (Access = protected)

        function this = computeAeroForcesForDataset(this, n)
            % Get all required input data
            rho_air = this.datasets.RHO_AIR{n};
            v_ground = this.datasets.GROUND_SPEED{n};
            v_air = this.datasets.AIR_SPEED{n};
            V_fuel = this.datasets.FUEL_LEVEL{n};
            x_FL = this.datasets.DAMPER_FL{n};
            x_FR = this.datasets.DAMPER_FR{n};
            x_RL = this.datasets.DAMPER_RL{n};
            x_RR = this.datasets.DAMPER_RR{n};

            % Concatenate accelerations
            a = [this.datasets.AX{n}; this.datasets.AY{n}; this.datasets.AZ{n}];

            % Compute ride height from damper positions and tire radius
            [RH_FL, RH_FR, RH_RL, RH_RR] = this.vehicle.rideHeightFromDamperPos(x_FL, x_FR, x_RL, x_RR);

            % Estimate aero loads
            [FL_aero, FR_aero, RL_aero, RR_aero] = this.vehicle.estimatedAeroWheelLoad(a, V_fuel, ...
                        RH_FL, RH_FR, RH_RL, RH_RR, ...
                        x_FL, x_FR, x_RL, x_RR, ...
                        0, 0, 0, 0);

            this.datasets.F_AERO_FRONT{n} = FL_aero.aero + FR_aero.aero;
            this = this.applyFilteringToData('F_AERO_FRONT', n);
            this.units.F_AERO_FRONT = 'N';

            this.datasets.F_AERO_REAR{n} = RL_aero.aero + RR_aero.aero;
            this = this.applyFilteringToData('F_AERO_REAR', n);
            this.units.F_AERO_REAR = 'N';

            this.datasets.F_AERO_TOTAL{n} = this.datasets.F_AERO_FRONT{n} + this.datasets.F_AERO_REAR{n};
            this = this.applyFilteringToData('F_AERO_TOTAL', n);
            this.units.F_AERO_TOTAL = 'N';

            this.datasets.AERO_BALANCE{n} = 100 * this.datasets.F_AERO_FRONT{n} ./ this.datasets.F_AERO_TOTAL{n};
            this = this.applyFilteringToData('AERO_BALANCE', n);
            this.units.AERO_BALANCE = '%';

            this.datasets.CL{n} = this.vehicle.aeroForceCoefficient(v_air, this.datasets.F_AERO_TOTAL{n}, rho_air);
            this = this.applyFilteringToData('CL', n);
            this.units.CL = '';
        end

    end

end
