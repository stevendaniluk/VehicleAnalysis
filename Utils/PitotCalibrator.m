% PitotCalibrator
%
% Utility for calibrating data from a pitot tube to correct the pressure reading such
% that it matches the actual ground speed. This can be necessary if the pitot tube is
% positioned close enough to the surface of a body that the airflow is distrubed, or if
% the pressure sensor is simply miscalibrated.
%
% Features:
%   -Can take in multiple datasets separately, no need to combine prior to adding
%   -Can fit polynomial to the pressure data to convert raw to corrected pressured
%    (the order of the polynomial is user configurable)
%   -Can bin the input data to get uniform coverage over the desired range to improved
%    the fitting process. Binning is done based on ground speed, and will randomly select
%    a prescribed number of data points from within each bin.
classdef PitotCalibrator
    properties (Access = protected)
        % Samples from input datasets
        v_ground = {};
        p_meas = {};
        rho = {};

        % Computed quantities from input datasets
        p_target = {};
        v_air = {};

        % Data that has been binned and subsampled
        v_ground_binned = {};
        p_meas_binned = {};

        % Computed quantities from binned data
        p_target_binned = {};
        v_air_binned = {};

        % Polynomial coefficients for pressure correction
        % P_corr = x1 * P^2 + x2 * P + x3
        press_poly;
    end

    methods
        % addData
        %
        % Adds a batch of data to the dataset. This can be done repeatedly.
        %
        % INPUTS:
        %   v_ground: Ground speed [m/s]
        %   p_meas: Measured dynamic pressure [Pa]
        %   rho: Air density [kg/m^3]
        function this = addData(this, v_ground, p_meas, rho)
            i = length(this.v_ground) + 1;
            this.v_ground{i} = v_ground;
            this.p_meas{i} = p_meas;
            this.rho{i} = rho;
            this.v_air{i} = this.pToV(p_meas, rho);
            this.p_target{i} = this.vToP(v_ground, rho);
        end

        % binData
        %
        % Divides the dataset into bins of equal width, then randomly subsamples the
        % data within each bin to obtain an equal number of data points in each bin.
        % Binning is done based on ground speed.
        %
        % INPUTS:
        %   bin_min: Beginning of the first bin [m/s]
        %   bin_max: End of the las bin [m/s]
        %   n_bins: Number of bins between bin_min, bin_max
        %   bin_pt_count: Number of points to sample within each bin
        function this = binData(this, bin_min, bin_max, n_bins, bin_pt_count)
            % Clear any existing bin data
            this.v_ground_binned = {};
            this.p_meas_binned = {};
            this.v_air_binned = {};
            this.p_target_binned = {};

            % Concatenate all data together
            v_ground_all = cell2mat(this.v_ground);
            p_meas_all = cell2mat(this.p_meas);
            v_air_all = cell2mat(this.v_air);
            p_target_all = cell2mat(this.p_target);

            bin_width = (bin_max - bin_min) / n_bins;
            edges = bin_min:bin_width:bin_max;

            v_ground_bin_ids = discretize(v_ground_all, edges);

            for i=1:n_bins
                % Get all data points within this bin
                this.v_ground_binned{i} = v_ground_all(v_ground_bin_ids == i);
                this.p_meas_binned{i} = p_meas_all(v_ground_bin_ids == i);
                this.v_air_binned{i} = v_air_all(v_ground_bin_ids == i);
                this.p_target_binned{i} = p_target_all(v_ground_bin_ids == i);

                % Trim the number of points present, randomly select the prescribed number
                % of points
                n_points = length(this.v_ground_binned{i});
                keep_indices = zeros(1, n_points);
                keep_indices(1:bin_pt_count) = 1;
                keep_indices = logical(keep_indices);
                keep_indices = keep_indices(randperm(n_points));

                this.v_ground_binned{i} = this.v_ground_binned{i}(keep_indices);
                this.p_meas_binned{i} = this.p_meas_binned{i}(keep_indices);
                this.v_air_binned{i} = this.v_air_binned{i}(keep_indices);
                this.p_target_binned{i} = this.p_target_binned{i}(keep_indices);

                % Check if we got the targeted number of points for this bin
                n_points_found = sum(keep_indices);
                if n_points_found < bin_pt_count
                    warning('Bin %d (%.1f - %.1f) does not have enough points, needed %d but only found %d', ...
                        i, bin_min + bin_width * (i - 1), bin_min + bin_width * i, ...
                        bin_pt_count, n_points_found);
                end
            end
        end

        % fitPolynomial
        %
        % Fits an n-order polynomial to the data. This will fit to the binned data if
        % present, otherwise it will fit to the full dataset.
        %
        % INPUTS:
        %   order: Order of the polynomial to fit
        function this = fitPolynomial(this, order)
            % Aggregate all data together
            p_meas_all = [];
            p_target_all = [];
            if ~isempty(this.v_ground_binned)
                p_meas_all = cell2mat(this.p_meas_binned);
                p_target_all = cell2mat(this.p_target_binned);
            else
                p_meas_all = cell2mat(this.p_meas_all);
                p_target_all = cell2mat(this.p_target_all);
            end

            [this.press_poly, err] = polyfit(p_meas_all, p_target_all, order);

            % Disply the polynomial info
            fprintf('Pressure Correction Polynomial:\n\tP_corr = ');
            for i=0:order
                coeff = this.press_poly(i + 1);
                exp_val = order - i;

                if exp_val > 1
                    fprintf('%.5f * P^%d + ', coeff, exp_val)
                elseif exp_val == 1
                    fprintf('%.5f * P + ', coeff);
                else
                    fprintf('%.5f', coeff);
                end
            end

            fprintf('\n\tResdiduals Norm: %.5f\n', err.normr);
        end

        % plotVelocityData
        %
        % Plots ground speed vs. measured air speed for the full dataset as will as
        % binned data (if present).
        function this = plotVelocityData(this)
            figure;
            clf;
            hold on;

            % Plot the unbinnned data (need to aggregate all data together so that only
            % a single legend entry is shown)
            v_ground_all = cell2mat(this.v_ground);
            v_air_all = cell2mat(this.v_air);
            plot(v_ground_all, v_air_all, 'bx', 'DisplayName', 'Dataset');

            % Plot each bin of data
            for i=1:length(this.v_ground_binned)
                label = sprintf('Bin %d', i);
                plot(this.v_ground_binned{i}, this.v_air_binned{i}, 'o', 'DisplayName', label);
            end

            grid on;
            ylabel('Air Speed [m/s]');
            xlabel('Ground Speed [m/s]');
            leg = legend(gca, 'show');
            leg.Location = 'northwest';

            % Make plot show origin
            x = xlim;
            y = ylim;
            xlim([0, x(2)]);
            ylim([0, y(2)]);
        end

        % plotPolynomialFit
        %
        % Plots the data used for polynomial fitting alongside the fitted polynomial.
        function this = plotPolynomialFit(this)
            figure;
            clf;
            hold on;

            % Aggregate all pressure data together and plot
            p_meas_all = [];
            p_target_all = [];
            if ~isempty(this.v_ground_binned)
                p_meas_all = cell2mat(this.p_meas_binned);
                p_target_all = cell2mat(this.p_target_binned);
            else
                p_meas_all = cell2mat(this.p_meas_all);
                p_target_all = cell2mat(this.p_target_all);
            end

            plot(p_meas_all, p_target_all, 'bx');

            % Plot the fitted polynomial
            p_samples = linspace(0, max(p_meas_all));
            p_fit = polyval(this.press_poly, p_samples);
            plot(p_samples, p_fit, 'r');

            grid on;
            xlabel('Measured Pressure [Pa]');
            ylabel('Corrected Pressure [Pa]');
            title('Pitot Tube Pressure Correction')
            leg = legend('Samples', 'Fitted Polynomial');
            leg.Location = 'northwest';

            % Make plot show origin and be square
            x = xlim;
            y = ylim;
            xlim([0, max(x(2), y(2))]);
            ylim([0, max(x(2), y(2))]);
        end
    end

    methods (Static)
        % pToV
        %
        % INPUTS:
        %   p: Dynamic air pressure [Pa]
        %   rho: Air density [kg/m^3]
        % OUTPUTS:
        %   v: Air speed [m/s]
        function v = pToV(p, rho)
            v = sqrt(2 * p / rho);
        end

        % vToP
        %
        % INPUTS:
        %   v: Air speed [m/s]
        %   rho: Air density [kg/m^3]
        % OUTPUTS:
        %   p: Dynamic air pressure [Pa]
        function p = vToP(v, rho)
            p = 0.5 * rho * v.^2;
        end
    end
end
