% TireAnalysis
%
% Utility for working with tire models and logged data. Functionality includes:
%   - Extracting relevant tire data from MoTeC logs
%   - Generating plots to show coverage of data
%   - Fitting tire models to log data
%   - Generating plots of tire model characteristics
%   - Estimating a longitudinal slip correction factor to account for effective rolling radius
%
%   [1] Tire and Vehicle Dynamics - 3rd Edition, Hans Pacejka
classdef TireAnalysis < handle
    properties
        % Number of data points
        n;
        % Timestamps
        t;

        %%%%% Vehicle properties %%%%%
        % Linear velocity in X direction [m/s]
        v;
        % Angular velocity about Z axis [rad/s]
        w;
        % Longtidunal acceleration [m/s^2]
        ax;
        % Lateral acceleration [m/s^2]
        ay;
        % Longitudinal force on the vehicle body [N]
        Fx_body;
        % Lateral force on the vehicle body [N]
        Fy_body;
        % Roll angle on front axle [rad]
        roll_f;
        % Roll angle on rear axle [rad]
        roll_r;
        % Steering wheel angle [rad]
        delta_steer;

        % Slip correction factors
        kappa_calibration = sfit(fittype('poly22'), 0, 0, 0, 0, 0, 0);

        %%%%% Per Tire Properties #####
        % Structure fields are:
        %   -gamma: Camber angle [rad]
        %   -delta: Wheel angle [rad]
        %   -P: Pressure [psi]
        %   -Fz: Normal load [N]
        %   -V_meas: Measured wheel speed [m/s]
        %   -V_est wheel speed [m/s]
        %   -Vsx: Longitudinal slip velocity [m/s]
        %   -Vsy: Lateral slip velocity [m/s]
        %   -alpha: Slip angle [rad]
        %   -kappa: Slip ratio []
        FL;
        FR;
        RL;
        RR;
    end

    methods (Access = public)

        % Constructor
        %
        function this = TireAnalysis()
            this = this.resetData();
        end

        % resetData
        %
        % Clears all log data for analysis.
        %
        % Note this only clears the data samples, it does not clear the slip
        % ratio correction factors.
        function this = resetData(this)
            this.n = 0;
            this.t = [];
            this.v = [];
            this.w = [];
            this.ax = [];
            this.ay = [];
            this.Fx_body = [];
            this.Fy_body = [];
            this.delta_steer = [];

            % Initialize corner properties
            corners = {'FL', 'FR', 'RL', 'RR'};
            for i=1:length(corners)
                this.(corners{i}).P = [];
                this.(corners{i}).Fz = [];
                this.(corners{i}).gamma = [];
                this.(corners{i}).delta = [];
                this.(corners{i}).V_meas = [];
                this.(corners{i}).V_est = [];
                this.(corners{i}).Vsx = [];
                this.(corners{i}).Vsy = [];
                this.(corners{i}).alpha = [];
                this.(corners{i}).kappa = [];
            end
        end

        % cornerCombined
        %
        % INPUTS:
        %   field: Name of field
        % OUTPUTS:
        %   data: Data for all corners concatenated in a single row vector as [FL, FR, RL, RR]
        function data = cornerCombined(this, field)
            data = [this.FL.(field), this.FR.(field), this.RL.(field), this.RR.(field)];
        end

        % addLogData
        %
        % Adds more log data to the analysis, appending it to the current set.
        %
        % INPUTS:
        %   handler: Motec handler to get data  from
        %   setup: Car setup object to query for vehicle info
        function this = addLogData(this, handler, setup)
            % Vehicle reference velocity
            [v_in, t_in] = handler.getSpeed();
            n_in = length(t_in);
            this.n = this.n + length(t_in);
            this.t = [this.t, t_in];
            this.v = [this.v, v_in];

            % Angular rate
            [w_in, ~] = handler.getYawRate();
            this.w = [this.w, w_in];

            % Tire pressures
            this.FL.P = [this.FL.P, handler.getTirePressureFL()];
            this.FR.P = [this.FR.P, handler.getTirePressureFR()];
            this.RL.P = [this.RL.P, handler.getTirePressureRL()];
            this.RR.P = [this.RR.P, handler.getTirePressureRR()];

            % Damper positions for loads and rake and roll angles
            [x_FL, ~] = handler.getDamperPosFL();
            [x_FR, ~] = handler.getDamperPosFR();
            [x_RL, ~] = handler.getDamperPosRL();
            [x_RR, ~] = handler.getDamperPosRR();

            % Wheel normal loads
            Fz_FL = setup.damperToWheelForce(setup.springForce(x_FL, true), true);
            this.FL.Fz = [this.FL.Fz, Fz_FL];

            Fz_FR = setup.damperToWheelForce(setup.springForce(x_FR, true), true);
            this.FR.Fz = [this.FR.Fz, Fz_FR];

            Fz_RL = setup.damperToWheelForce(setup.springForce(x_RL, false), false);
            this.RL.Fz = [this.RL.Fz, Fz_RL];

            Fz_RR = setup.damperToWheelForce(setup.springForce(x_RR, false), false);
            this.RR.Fz = [this.RR.Fz, Fz_RR];

            % Rake angle
            x_F = 0.5 * (x_FL + x_FR);
            x_R = 0.5 * (x_RL + x_RR);
            [RH_f, RH_r] = setup.rideHeightFromDamperPos(x_F, x_R);
            rake = setup.rakeFromRideHeights(RH_f, RH_r);

            % Roll angle
            [RH_FL, RH_RL] = setup.rideHeightFromDamperPos(x_FL, x_RL);
            [RH_FR, RH_RR] = setup.rideHeightFromDamperPos(x_FR, x_RR);
            this.roll_f = deg2rad(setup.rollFromRideHeights(RH_FL, RH_FR, true));
            this.roll_r = deg2rad(setup.rollFromRideHeights(RH_RL, RH_RR, false));

            % Camber
            this.FL.gamma = [this.FL.gamma, -setup.camber_f * ones(1, n_in) + this.roll_f];
            this.FR.gamma = [this.FR.gamma, setup.camber_f * ones(1, n_in) + this.roll_f];
            this.RL.gamma = [this.RL.gamma, -setup.camber_r * ones(1, n_in) + this.roll_r];
            this.RR.gamma = [this.RR.gamma, setup.camber_r * ones(1, n_in) + this.roll_r];

            % Body longitudinal and lateral forces, need to correct longitudinal force for
            % aerodynamic drag force drag
            ax_in = handler.getLongitudinalAccel();
            this.ax = [this.ax, ax_in];

            Fx_body_in = setup.totalMass() * ax_in + setup.drag(v_in, RH_f, rake);
            this.Fx_body = [this.Fx_body, Fx_body_in];

            ay_in = handler.getLateralAccel();
            this.ay = [this.ay, ay_in];
            this.Fy_body = [this.Fy_body, setup.totalMass() * ay_in];

            % Wheel angles with respect to vehicle body
            [delta_steer_in, ~] = handler.getSteeringWheelAngle();
            [delta_wheel_FL, delta_wheel_FR] = setup.steerToWheelAngle(delta_steer_in);
            delta_wheel_RL = -setup.toe_r * ones(size(delta_steer_in));
            delta_wheel_RR = setup.toe_r * ones(size(delta_steer_in));

            this.FL.delta = [this.FL.delta, delta_wheel_FL];
            this.FR.delta = [this.FR.delta, delta_wheel_FR];
            this.RL.delta = [this.RL.delta, delta_wheel_RL];
            this.RR.delta = [this.RR.delta, delta_wheel_RR];

            % Predicted wheel speeds
            this.delta_steer = [this.delta_steer, delta_steer_in];

            [V_FL_est_in, V_FR_est_in, V_RL_est_in, V_RR_est_in] = ...
                setup.expectedWheelSpeed(v_in, w_in, delta_steer_in);

            this.FL.V_est = [this.FL.V_est, V_FL_est_in];
            this.FR.V_est = [this.FR.V_est, V_FR_est_in];
            this.RL.V_est = [this.RL.V_est, V_RL_est_in];
            this.RR.V_est = [this.RR.V_est, V_RR_est_in];

            % Measured wheel speeds
            [V_FL_meas_in, ~] = handler.getWheelSpeedFL();
            [V_FR_meas_in, ~] = handler.getWheelSpeedFR();
            [V_RL_meas_in, ~] = handler.getWheelSpeedRL();
            [V_RR_meas_in, ~] = handler.getWheelSpeedRR();

            this.FL.V_meas = [this.FL.V_meas, V_FL_meas_in];
            this.FR.V_meas = [this.FR.V_meas, V_FR_meas_in];
            this.RL.V_meas = [this.RL.V_meas, V_RL_meas_in];
            this.RR.V_meas = [this.RR.V_meas, V_RR_meas_in];

            % Compute the slip angles
            this.FL.alpha = [this.FL.alpha, -atan2(V_FL_est_in(2, :), V_FL_est_in(1, :))];
            this.FR.alpha = [this.FR.alpha, -atan2(V_FR_est_in(2, :), V_FR_est_in(1, :))];
            this.RL.alpha = [this.RL.alpha, -atan2(V_RL_est_in(2, :), V_RL_est_in(1, :))];
            this.RR.alpha = [this.RR.alpha, -atan2(V_RR_est_in(2, :), V_RR_est_in(1, :))];

            % Compute the slip ratio, correct for effective tire radius, and
            % prune singularities
            kappa_v_min = 5.0;

            kappa_FL_in = (V_FL_meas_in - V_FL_est_in(1, :)) ./ V_FL_est_in(1, :);
            kappa_FL_in(V_FL_est_in(1, :) < kappa_v_min) = 0;
            kappa_FL_in = kappa_FL_in - this.kappa_calibration(v_in, Fz_FL);
            this.FL.kappa = [this.FL.kappa, kappa_FL_in];

            kappa_FR_in = (V_FR_meas_in - V_FR_est_in(1, :)) ./ V_FR_est_in(1, :);
            kappa_FR_in(V_FR_est_in(1, :) < kappa_v_min) = 0;
            kappa_FR_in = kappa_FR_in - this.kappa_calibration(v_in, Fz_FR);
            this.FR.kappa = [this.FR.kappa, kappa_FR_in];

            kappa_RL_in = (V_RL_meas_in - V_RL_est_in(1, :)) ./ V_RL_est_in(1, :);
            kappa_RL_in(V_RL_est_in(1, :) < kappa_v_min) = 0;
            kappa_RL_in = kappa_RL_in - this.kappa_calibration(v_in, Fz_RL);
            this.RL.kappa = [this.RL.kappa, kappa_RL_in];

            kappa_RR_in = (V_RR_meas_in - V_RR_est_in(1, :)) ./ V_RR_est_in(1, :);
            kappa_RR_in(V_RR_est_in(1, :) < kappa_v_min) = 0;
            kappa_RR_in = kappa_RR_in - this.kappa_calibration(v_in, Fz_RR);
            this.RR.kappa = [this.RR.kappa, kappa_RR_in];

            % Slip velocity
            this.FL.Vsx = [this.FL.Vsx, V_FL_est_in(1, :) - V_FL_meas_in];
            this.FL.Vsy = [this.FL.Vsy, V_FL_est_in(2, :)];

            this.FR.Vsx = [this.FR.Vsx, V_FR_est_in(1, :) - V_FR_meas_in];
            this.FR.Vsy = [this.FR.Vsy, V_FR_est_in(2, :)];

            this.RL.Vsx = [this.RL.Vsx, V_RL_est_in(1, :) - V_RL_meas_in];
            this.RL.Vsy = [this.RL.Vsy, V_RL_est_in(2, :)];

            this.RR.Vsx = [this.RR.Vsx, V_RR_est_in(1, :) - V_RR_meas_in];
            this.RR.Vsy = [this.RR.Vsy, V_RR_est_in(2, :)];
        end

        % estimateSlipCorrection
        %
        % Fits a 2 dimensional second order polynomial for the form:
        %   kappa_corr = f(V, Fz)
        % to log data to estimate a correction factor for slip ratios. This is
        % intended to account for changes in the slip ratio due to normal load
        % and radial expansion.
        %
        % INPUTS:
        %   accel_limit: Maximum resultant acceleration to allow in the data
        %                used for estimating the correction factors [m/s^2]
        %   show_plot: When true a plot of the data and fit will be generated (optional)
        function this = estimateSlipCorrection(this, accel_limit, show_plot)
            if nargin < 3
                show_plot = false;
            end

            % Only use the slip ratio of the front wheels under low acceleration conditions
            a_combined = sqrt(this.ax.^2 + this.ay.^2);
            indices = abs(a_combined) < accel_limit;

            V_meas_cal_data = [this.FL.V_meas(indices), this.FR.V_meas(indices)];
            Fz_cal_data = [this.FL.Fz(indices), this.FR.Fz(indices)];
            kappa_cal_data = [this.FL.kappa(indices), this.FR.kappa(indices)];

            objective = ...
                @(p, x) p(1) * x(1, :) + p(2) * x(1, :).^2 + p(3) * x(2, :) + p(4) * x(2, :).^2;
            p0 = [0, 0, 0, 0];
            lb = [0, 0, 0, -inf];
            ub = [inf, inf, inf, inf];
            optim_options = optimoptions('lsqcurvefit');
            optim_options.Display = 'off';

            p_fit = lsqcurvefit(objective, p0, [V_meas_cal_data; Fz_cal_data], kappa_cal_data, ...
                lb, ub, optim_options);
            this.kappa_calibration = ...
                sfit(fittype('poly22'), 0, p_fit(1), p_fit(3), p_fit(2), 0, p_fit(4));

            if show_plot
                figure;
                hold on;
                scatter3(V_meas_cal_data, Fz_cal_data, kappa_cal_data);
                plot(this.kappa_calibration);
                title('Measured Slip Ratio and Estimated Correction Function');
                xlabel('Wheel Speed [m/s]');
                ylabel('Normal Load F_z [N]');
                zlabel('\kappa');
                axis([0, 80, 0, 5e3, 0, 0.02]);
            end
        end

        % fitModelFx0
        %
        % Fits the Fx0 parameters based on the current log data.
        %
        % INPUTS:
        %   model: PacejkaTireModel to fit
        %   driven: Logical array for which wheels are driven
        % OUTPUTS:
        %   model: Model with Fx0 parameters fit
        function model = fitModelFx0(this, model, driven)
            model = model.fitFx0(this.Fx_body, this.cornerCombined('Fz'), ...
                this.cornerCombined('P'), this.cornerCombined('kappa'), ...
                this.cornerCombined('gamma'), this.cornerCombined('Vsx'), this.v, 4, driven);
        end

        % fitModelFx
        %
        % Fits the Fx0 and Fx parameters based on the current log data.
        %
        % INPUTS:
        %   model: PacejkaTireModel to fit
        %   driven: Logical array for which wheels are driven
        % OUTPUTS:
        %   model: Model with Fx0 and Fx parameters fit
        function model = fitModelFx(this, model, driven)
            model = model.fitFx(this.Fx_body, this.cornerCombined('Fz'), ...
                this.cornerCombined('P'), this.cornerCombined('kappa'), ...
                this.cornerCombined('alpha'), this.cornerCombined('gamma'), ...
                this.cornerCombined('Vsx'), this.v, 4, driven);
        end

        % fitModelFy0
        %
        % Fits the Fy0 parameters based on the current log data.
        %
        % INPUTS:
        %   model: PacejkaTireModel to fit
        % OUTPUTS:
        %   model: Model with Fy0 parameters fit
        function model = fitModelFy0(this, model)
            model = model.fitFy0(this.Fy_body, this.cornerCombined('Fz'), ...
                this.cornerCombined('P'), this.cornerCombined('alpha'), ...
                this.cornerCombined('gamma'), this.cornerCombined('Vsy'), this.v, 4);
        end

        % fitModelFx
        %
        % Fits the Fy0 and Fy parameters based on the current log data.
        %
        % INPUTS:
        %   model: PacejkaTireModel to fit
        % OUTPUTS:
        %   model: Model with Fy0 and Fy parameters fit
        function model = fitModelFy(this, model)
            model = model.fitFx(this.Fx_body, this.cornerCombined('Fz'), ...
                this.cornerCombined('P'), this.cornerCombined('kappa'), ...
                this.cornerCombined('alpha'), this.cornerCombined('gamma'), ...
                this.cornerCombined('Vsy'), this.v, 4);
        end

        % fitModelFxFy
        %
        % Fits the Fx0, Fx, Fy0, and Fy parameters based on the current log data.
        %
        % INPUTS:
        %   model: PacejkaTireModel to fit
        %   driven: Logical array for which wheels are driven
        % OUTPUTS:
        %   model: Model with all parameters fit
        function model = fitModelFxFy(this, model, driven)
            model = model.fitFxFy(this.Fx_body, this.Fy_body, ...
                this.cornerCombined('delta'), ...
                this.cornerCombined('Fz'), this.cornerCombined('P'), ...
                this.cornerCombined('kappa'), this.cornerCombined('alpha'), ...
                this.cornerCombined('gamma'), this.cornerCombined('Vsx'), ...
                this.cornerCombined('Vsy'), this.v, 4, driven);
        end

        % getFxFyError
        %
        % INPUTS:
        %   model: PacejkaTireModel to evaluate
        % OUTPUTS:
        %   Fx_err: Sum of squared Fx error
        %   Fy_err: Sum of squared Fy error
        function [Fx_err, Fy_err] = getFxFyError(this, model)
            [Fx_est, ~, ~, ~, ~] = this.getEstimatedForce(model, 'Fx');
            Fx_err = sum((this.Fx_body - Fx_est).^2);

            [Fy_est, ~, ~, ~, ~] = this.getEstimatedForce(model, 'Fy');
            Fy_err = sum((this.Fy_body - Fy_est).^2);
        end

        % plotInputData
        %
        % Plots the input data vs time:
        %   -Velocity
        %   -Longitudinal and lateral forces
        %   -Steer angle
        %   -Slip angle
        %   -Slip ratio
        %   -Slip velocity
        %   -Normal load
        %   -Pressure
        function this = plotInputData(this)
            figure;
            plot_n = 8;

            subplot(plot_n, 1, 1);
            hold on;
            grid on;
            plot(this.t, this.v * 3.6);
            ylabel('Velocity [km/h]');
            xlabel('Time [s]');

            subplot(plot_n, 1, 2);
            hold on;
            grid on;
            plot(this.t, [1e-3 * this.Fx_body; 1e-3 * this.Fy_body]);
            ylabel('F [kN]');
            xlabel('Time [s]');
            legend({'F_x', 'F_y'});

            subplot(plot_n, 1, 3);
            hold on;
            grid on;
            plot(this.t, rad2deg(this.delta_steer));
            ylabel('Steer Angle [deg]');
            ylabel('Wheel Angle [deg]');
            xlabel('Time [s]');

            subplot(plot_n, 1, 4);
            hold on;
            grid on;
            plot(this.t, rad2deg([this.FL.alpha; this.FR.alpha; this.RL.alpha; this.RR.alpha]));
            ylabel('Slip Angle [deg]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});
            ylim([-15, 15]);

            subplot(plot_n, 1, 5);
            hold on;
            grid on;
            plot(this.t, 100 * [this.FL.kappa; this.FR.kappa; this.RL.kappa; this.RR.kappa]);
            ylabel('Slip Ratio [%]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});
            ylim([-15, 15]);

            subplot(plot_n, 1, 6);
            hold on;
            grid on;
            plot(this.t, rad2deg([this.FL.gamma; this.FR.gamma; this.RL.gamma; this.RR.gamma]));
            ylabel('Camber [\circ]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});

            subplot(plot_n, 1, 7);
            hold on;
            grid on;
            plot(this.t, 1e-3 * [this.FL.Fz; this.FR.Fz; this.RL.Fz; this.RR.Fz]);
            ylabel('Normal Load [kN]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});

            subplot(plot_n, 1, 8);
            hold on;
            grid on;
            plot(this.t, 1e-3 * [this.FL.P; this.FR.P; this.RL.P; this.RR.P]);
            ylabel('Pressure [psi]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});
        end

        % plotDataCoverage
        %
        % Generates plots to asses the coverage of data for tire model
        % estimation. This will plot the following:
        %   -Histograms of:
        %     -Velocity
        %     -Camber angle
        %     -Pressure
        %     -Normal load
        %     -Slip Angle
        %     -Slip Ratio
        %  -Scatter plots for
        %    -Slip angle vs normal load
        %    -Slip angle vs pressure
        %    -Slip angle vs camber angle
        %    -Slip ratio vs normal load
        %    -Slip ratio vs pressure
        %    -Slip ratio vs camber angle
        function this = plotDataCoverage(this)
            % Combine all the data together for each corner
            V = 3.6 * [this.v, this.v, this.v, this.v];
            gamma = 180 / pi * abs([this.FL.gamma, this.FL.gamma, this.RL.gamma, this.RR.gamma]);
            P = [this.FL.P, this.FR.P, this.RL.P, this.RR.P];
            Fz = 1e-3 * [this.FL.Fz, this.FR.Fz, this.RL.Fz, this.RR.Fz];
            alpha = 180 / pi * [this.FL.alpha, this.FR.alpha, this.RL.alpha, this.RR.alpha];
            kappa = 100 * [this.FL.kappa, this.FR.kappa, this.RL.kappa, this.RR.kappa];

            % Histograms
            figure;

            subplot(2, 3, 1);
            histogram(3.6 * this.v);
            grid on;
            xlabel('Velocity [km/h]');
            ylabel('Samples');

            subplot(2, 3, 2);
            histogram(gamma);
            grid on;
            xlabel('Camber \gamma [\circ]');
            ylabel('Samples');

            subplot(2, 3, 3);
            histogram(P);
            grid on;
            xlabel('Pressure [psi]');
            ylabel('Samples');

            subplot(2, 3, 4);
            histogram(Fz);
            grid on;
            xlabel('Normal Load F_z [kN]');
            ylabel('Samples');

            subplot(2, 3, 5);
            histogram(alpha);
            grid on;
            xlabel('Slip Angle \alpha [\circ]');
            ylabel('Samples');

            subplot(2, 3, 6);
            histogram(kappa);
            grid on;
            xlabel('Slip Ratio \kappa [%]');
            ylabel('Samples');


            % Relative coverage
            figure;

            subplot(2, 3, 1);
            scatter(alpha, Fz, [], V);
            grid on;
            xlabel('Slip Angle \alpha [deg]');
            ylabel('Normal Load F_z [kN]');
            title('Slip angle and normal load coverage coloured for velocity');

            subplot(2, 3, 2);
            scatter(alpha, P, [], V);
            grid on;
            xlabel('Slip Angle \alpha [deg]');
            ylabel('Pressure [psi]');
            title('Slip angle and pressure coverage coloured for velocity');

            subplot(2, 3, 3);
            scatter(alpha, gamma, [], V);
            grid on;
            xlabel('Slip Angle \alpha [deg]');
            ylabel('Camber \gamma [\circ]');
            title('Slip angle and camber coverage coloured for velocity');

            subplot(2, 3, 4);
            scatter(kappa, Fz, [], V);
            grid on;
            xlabel('Slip Ratio \kappa [%]');
            ylabel('Normal Load F_z [kN]');
            title('Slip ratio and normal load coverage coloured for velocity');

            subplot(2, 3, 5);
            scatter(kappa, P, [], V);
            grid on;
            xlabel('Slip Ratio \kappa [%]');
            ylabel('Pressure [psi]');
            title('Slip ratio and pressure coverage coloured for velocity');

            subplot(2, 3, 6);
            scatter(kappa, gamma, [], V);
            grid on;
            xlabel('Slip Ratio \kappa [%]');
            ylabel('Camber \gamma [\circ]');
            title('Slip ratio and camber coverage coloured for velocity');
        end

        % plotMeasuredVsPredicted
        %
        % Generates plots for:
        %   -Velocity
        %   -Measured and predicted Fy
        %   -Per corner predicted Fy
        %   -Measured and predicted Fx
        %   -Per corner predicted Fx
        function this = plotMeasuredVsPredicted(this, model)
            [Fx_est_total, Fx_est_FL, Fx_est_FR, Fx_est_RL, Fx_est_RR] = ...
                this.getEstimatedForce(model, 'Fx');
            [Fy_est_total, Fy_est_FL, Fy_est_FR, Fy_est_RL, Fy_est_RR] = ...
                this.getEstimatedForce(model, 'Fy');

            figure;
            plot_n = 5;

            subplot(plot_n, 1, 1);
            hold on;
            grid on;
            plot(this.t, this.v * 3.6);
            ylabel('Velocity [km/h]');
            xlabel('Time [s]');

            subplot(plot_n, 1, 2);
            hold on;
            grid on;
            plot(this.t, 1e-3 * this.Fy_body);
            plot(this.t, 1e-3 * Fy_est_total);
            ylabel('F_y [kN]');
            xlabel('Time [s]');
            legend({'Measured', 'Estimated'});

            subplot(plot_n, 1, 3);
            hold on;
            grid on;
            plot(this.t, 1e-3 * [Fy_est_FL; Fy_est_FR; Fy_est_RL; Fy_est_RR]);
            ylabel('F_y Estimated [kN]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});

            subplot(plot_n, 1, 4);
            hold on;
            grid on;
            plot(this.t, 1e-3 * this.Fx_body);
            plot(this.t, 1e-3 * Fx_est_total);
            ylabel('F_x [kN]');
            xlabel('Time [s]');
            legend({'Measured', 'Estimated'});

            subplot(plot_n, 1, 5);
            hold on;
            grid on;
            plot(this.t, 1e-3 * [Fx_est_FL; Fx_est_FR; Fx_est_RL; Fx_est_RR]);
            ylabel('F_x Estimated [kN]');
            xlabel('Time [s]');
            legend({'FL', 'FR', 'RL', 'RR'});
        end

        % plotForceCurves
        %
        % Generates plots for:
        %   -Fx vs kappa, for varying Fz
        %   -Fx vs kappa, for varying alpha
        %   -Fx vs kappa, for varying camber
        %   -Fy vs alpha, for varying Fz
        %   -Fy vs alpha, for varying kappa
        %   -Fy vs alpha, for varying camber
        %   -Fy vs Fx, for varying alpha
        %   -Fy vs Fx, for varying kappa
        function this = plotForceCurves(this, model, mu_max, n_curves, Fz_range, P_range, ...
            alpha_range, kappa_range, gamma_range)

            alpha_range_rad = deg2rad(alpha_range);
            gamma_range_rad = deg2rad(gamma_range);

            alpha_plot = linspace(-alpha_range(3) * 0, alpha_range(3));
            alpha_plot_rad = deg2rad(alpha_plot);

            kappa_plot = linspace(kappa_range(1), kappa_range(3));

            V0_plot = 30.0;
            Vs_plot = 0.0 * V0_plot;

            Fz_vals = linspace(min(Fz_range), max(Fz_range), n_curves);
            P_vals = linspace(min(P_range), max(P_range), n_curves);
            alpha_vals = linspace(alpha_range(2), alpha_range(3), n_curves);
            kappa_vals = linspace(kappa_range(1), kappa_range(3), n_curves);
            gamma_vals = linspace(gamma_range(1), gamma_range(3), n_curves);

            figure;

            % Fx vs. kappa for varying Fz
            subplot(2, 4, 1);
            hold on;
            grid on;
            for i=1:length(Fz_vals)
                label = sprintf('F_z=%.1f kN', 1e-3 * Fz_vals(i));

                [Fx_est, ~] = ...
                    model.Fx(Fz_vals(i), model.P0, kappa_plot, alpha_range_rad(2), ...
                    gamma_range_rad(2), Vs_plot, V0_plot);
                plot(100 * kappa_plot, Fx_est / Fz_vals(i), 'DisplayName', label);
                xlabel('\kappa [%]');
                ylabel('F_x / F_z');
                title(sprintf(['Pure Longitudinal Force (\\alpha=%.1f\\circ,', ...
                    ' \\gamma=%.1f\\circ, P=%.1f psi)'], ...
                    alpha_range(2), gamma_range(2), model.P0));
            end
            ylim([0, mu_max]);
            legend(gca, 'show');

            % Fx vs. kappa for varying alpha
            subplot(2, 4, 2);
            hold on;
            grid on;
            for i=1:length(alpha_vals)
                label = sprintf('\\alpha=%.1f\\circ', alpha_vals(i));

                [Fx_est, ~] = ...
                    model.Fx(Fz_range(2), model.P0, kappa_plot, deg2rad(alpha_vals(i)), ...
                    gamma_range_rad(2), Vs_plot, V0_plot);
                plot(100 * kappa_plot, 1e-3 * Fx_est, 'DisplayName', label);
                xlabel('\kappa [%]');
                ylabel('F_x [kN]');
                title(sprintf(['Pure Longitudinal Force (F_z=%d kN,', ...
                    ' \\gamma=%.1f\\circ, P=%.1f psi)'], ...
                    1e-3 * Fz_range(2), gamma_range(2), model.P0));
            end
            ylim([0, mu_max * Fz_range(2) * 1e-3]);
            legend(gca, 'show');

            % Fx vs. kappa for varying gamma
            subplot(2, 4, 3);
            hold on;
            grid on;
            for i=1:length(gamma_vals)
                label = sprintf('\\gamma=%.1f\\circ', gamma_vals(i));

                [Fx_est, ~] = ...
                    model.Fx(Fz_range(2), model.P0, kappa_plot, alpha_range_rad(2), ...
                    deg2rad(gamma_vals(i)), Vs_plot, V0_plot);
                plot(100 * kappa_plot, 1e-3 * Fx_est, 'DisplayName', label);
                xlabel('\kappa [%]');
                ylabel('F_x [kN]');
                title(sprintf(['Pure Longitudinal Force (F_z=%d kN, \\alpha=%.1f\\circ,', ...
                    ' P=%.1f psi)'], ...
                    1e-3 * Fz_range(2), alpha_range(2), model.P0));
            end
            ylim([0, mu_max * Fz_range(2) * 1e-3]);
            legend(gca, 'show');

            % Fx vs. kappa for varying pressure
            subplot(2, 4, 4);
            hold on;
            grid on;
            for i=1:length(P_vals)
                label = sprintf('P=%.1f psi', P_vals(i));

                [Fx_est, ~] = ...
                    model.Fx(Fz_range(2), P_vals(i), kappa_plot, alpha_range_rad(2), ...
                    gamma_range_rad(2), Vs_plot, V0_plot);
                plot(100 * kappa_plot, 1e-3 * Fx_est, 'DisplayName', label);
                xlabel('\kappa [%]');
                ylabel('F_x [kN]');
                title(sprintf(['Pure Longitudinal Force (F_z=%d kN, \\alpha=%.1f\\circ,', ...
                    ' \\gamma=%.1f\\circ)'], ...
                    1e-3 * Fz_range(2), alpha_range(2), gamma_range(2)));
            end
            ylim([0, mu_max * Fz_range(2) * 1e-3]);
            legend(gca, 'show');

            % Fy vs. alpha for varying Fz
            subplot(2, 4, 5);
            hold on;
            grid on;
            for i=1:length(Fz_vals)
                label = sprintf('F_z=%.1f kN', 1e-3 * Fz_vals(i));

                [Fy_est, ~] = ...
                    model.Fy(Fz_vals(i), model.P0, kappa_range(2), alpha_plot_rad, ...
                    gamma_range_rad(2), Vs_plot, V0_plot);
                plot(alpha_plot, Fy_est / Fz_vals(i), 'DisplayName', label);
                xlabel('\alpha [\circ]');
                ylabel('F_y / F_z');
                title(sprintf(['Pure Lateral Force (\\kappa=%.1f%%, \\gamma=%.1f\\circ,', ...
                    ' P=%.1f psi)'], ...
                    kappa_range(2), gamma_range(2), model.P0));
            end
            ylim([0, mu_max]);
            legend(gca, 'show');

            % Fy vs. alpha for varying kappa
            subplot(2, 4, 6);
            hold on;
            grid on;
            for i=1:length(kappa_vals)
                label = sprintf('\\kappa=%.1f%%', 100 * kappa_vals(i));

                [Fy_est, ~] = ...
                    model.Fy(Fz_range(2), model.P0, kappa_vals(i), alpha_plot_rad, ...
                    gamma_range_rad(2), Vs_plot, V0_plot);
                plot(alpha_plot, 1e-3 * Fy_est, 'DisplayName', label);
                xlabel('\alpha [\circ]');
                ylabel('F_y [kN]');
                title(sprintf('Pure Lateral Force (F_z=%d kN, \\gamma=%.1f\\circ, P=%.1f psi)', ...
                    1e-3 * Fz_range(2), gamma_range(2), model.P0));
            end
            ylim([0, mu_max * Fz_range(2) * 1e-3]);
            legend(gca, 'show');

            % Fy vs. alpha for varying gamma
            subplot(2, 4, 7);
            hold on;
            grid on;
            for i=1:length(gamma_vals)
                label = sprintf('\\gamma=%.1f\\circ', gamma_vals(i));

                [Fy_est, ~] = ...
                    model.Fy(Fz_range(2), model.P0, kappa_range(2), alpha_plot_rad, ...
                    deg2rad(gamma_vals(i)), Vs_plot, V0_plot);
                plot(alpha_plot, 1e-3 * Fy_est, 'DisplayName', label);
                xlabel('\alpha [\circ]');
                ylabel('F_y [kN]');
                title(sprintf('Pure Lateral Force (F_z=%d kN, \\kappa=%.1f%%, P=%.1f psi)', ...
                    1e-3 * Fz_range(2), kappa_range(2), model.P0));
            end
            ylim([0, mu_max * Fz_range(2) * 1e-3]);
            legend(gca, 'show');

            % Fy vs. alpha for varying pressure
            subplot(2, 4, 8);
            hold on;
            grid on;
            for i=1:length(P_vals)
                label = sprintf('P=%.1f psi', P_vals(i));

                [Fy_est, ~] = ...
                    model.Fy(Fz_range(2), P_vals(i), kappa_range(2), alpha_plot_rad, ...
                    gamma_range_rad(2), Vs_plot, V0_plot);
                plot(alpha_plot, 1e-3 * Fy_est, 'DisplayName', label);
                xlabel('\alpha [\circ]');
                ylabel('F_y [kN]');
                title(sprintf(['Pure Lateral Force (Fz=%.d kN, \\kappa=%.1f%%,', ...
                    ' \\gamma=%.1f\\circ)'], ...
                    1e-3 * Fz_range(2), kappa_range(2), gamma_range(2)));
            end
            ylim([0, mu_max * Fz_range(2) * 1e-3]);
            legend(gca, 'show');
        end

        % plotFxVsFy
        %
        % Generates plots for:
        %   -Fx vs Fy with varying slip angle, multiple Fz plots
        %   -Fx vs Fy with varying slip angle, multiple camber plots
        %   -Fx vs Fy with varying slip angle, multiple pressure plots
        function this = plotFxVsFy(this, model, alpha_vals, Fz_range, P_range, ...
            gamma_range, kappa_range)

            figure;

            kappa_plot_mirrored = linspace(-abs(kappa_range(3)), abs(kappa_range(3)));

            V0_plot = 30.0;
            Vs_plot = 0.0 * V0_plot;

            % Fx vs Fy with varying slip angle, varying Fz
            for i=1:3
                subplot(3, 3, i);
                hold on;
                grid on;
                for j=1:length(alpha_vals)
                    label = sprintf('\\alpha=%.1f\\circ', alpha_vals(j));

                    [Fx_est, ~] = model.Fx(Fz_range(i), P_range(2), kappa_plot_mirrored, ...
                        deg2rad(alpha_vals(j)), deg2rad(gamma_range(2)), Vs_plot, V0_plot);
                    [Fy_est, ~] = model.Fy(Fz_range(i), P_range(2), kappa_plot_mirrored, ...
                        deg2rad(alpha_vals(j)), deg2rad(gamma_range(2)), Vs_plot, V0_plot);

                    plot(1e-3 * Fx_est, 1e-3 * Fy_est, 'DisplayName', label);
                end

                title(sprintf(...
                    '\\kappa=[%.1f%%, %.1f%%], F_z=%.1f kN, P=%.1f psi, \\gamma=%.1f\\circ', ...
                    100 * -abs(kappa_range(3)), 100 * abs(kappa_range(3)), 1e-3 * Fz_range(i), ...
                    P_range(2), gamma_range(2)));
                xlabel('F_x [kN]');
                ylabel('F_y [kN]');
                legend(gca, 'show');
                axis equal;
            end

            % Fx vs Fy with varying slip angle, varying camber
            for i=1:3
                subplot(3, 3, i + 3);
                hold on;
                grid on;
                for j=1:length(alpha_vals)
                    label = sprintf('\\alpha=%.1f\\circ', alpha_vals(j));

                    [Fx_est, ~] = model.Fx(Fz_range(2), P_range(2), kappa_plot_mirrored, ...
                        deg2rad(alpha_vals(j)), deg2rad(gamma_range(i)), Vs_plot, V0_plot);
                    [Fy_est, ~] = model.Fy(Fz_range(2), P_range(2), kappa_plot_mirrored, ...
                        deg2rad(alpha_vals(j)), deg2rad(gamma_range(i)), Vs_plot, V0_plot);

                    plot(1e-3 * Fx_est, 1e-3 * Fy_est, 'DisplayName', label);
                end

                title(sprintf(...
                    '\\kappa=[%.1f%%, %.1f%%], F_z=%.1f kN, P=%.1f psi, \\gamma=%.1f\\circ', ...
                    100 * -abs(kappa_range(3)), 100 * abs(kappa_range(3)), 1e-3 * Fz_range(2), ...
                    P_range(2), gamma_range(i)));
                xlabel('F_x [kN]');
                ylabel('F_y [kN]');
                legend(gca, 'show');
                axis equal;
            end

            % Fx vs Fy with varying slip angle, varying pressure
            for i=1:3
                subplot(3, 3, i + 6);
                hold on;
                grid on;
                for j=1:length(alpha_vals)
                    label = sprintf('\\alpha=%.1f\\circ', alpha_vals(j));

                    [Fx_est, ~] = model.Fx(Fz_range(2), P_range(i), kappa_plot_mirrored, ...
                        deg2rad(alpha_vals(j)), deg2rad(gamma_range(2)), Vs_plot, V0_plot);
                    [Fy_est, ~] = model.Fy(Fz_range(2), P_range(i), kappa_plot_mirrored, ...
                        deg2rad(alpha_vals(j)), deg2rad(gamma_range(2)), Vs_plot, V0_plot);

                    plot(1e-3 * Fx_est, 1e-3 * Fy_est, 'DisplayName', label);
                end

                title(sprintf(...
                    '\\kappa=[%.1f%%, %.1f%%], F_z=%.1f kN, P=%.1f psi, \\gamma=%.1f\\circ', ...
                    100 * -abs(kappa_range(3)), 100 * abs(kappa_range(3)), 1e-3 * Fz_range(2), ...
                    P_range(i), gamma_range(i)));
                xlabel('F_x [kN]');
                ylabel('F_y [kN]');
                legend(gca, 'show');
                axis equal;
            end
        end
    end

    methods(Static, Access = public)
        % resetModelToSampleParameters
        %
        % Copy parameter values from Table 4.2 pg. 190 of [1]
        function model = resetModelToSampleParameters(model)
            model.Fz0 = 3000;
            model.P0 = 30;

            model.PFx0(model.ID_Ppx1) = 0;
            model.PFx0(model.ID_Ppx2) = 0;
            model.PFx0(model.ID_Ppx3) = 0;
            model.PFx0(model.ID_Ppx4) = 0;
            model.PFx0(model.ID_PCx1) = 1.65;
            model.PFx0(model.ID_PDx1) = 1;
            model.PFx0(model.ID_PDx2) = 0;
            model.PFx0(model.ID_PDx3) = 0;
            model.PFx0(model.ID_PEx1) = -0.5;
            model.PFx0(model.ID_PEx2) = 0;
            model.PFx0(model.ID_PEx3) = 0;
            model.PFx0(model.ID_PEx4) = 0;
            model.PFx0(model.ID_PKx1) = 12;
            model.PFx0(model.ID_PKx2) = 10;
            model.PFx0(model.ID_PKx3) = -0.6;
            model.PFx0(model.ID_PHx1) = 0;
            model.PFx0(model.ID_PHx2) = 0;
            model.PFx0(model.ID_PVx1) = 0;
            model.PFx0(model.ID_PVx2) = 0;

            model.PFy0(model.ID_Ppy1) = 0;
            model.PFy0(model.ID_Ppy2) = 0;
            model.PFy0(model.ID_Ppy3) = 0;
            model.PFy0(model.ID_Ppy4) = 0;
            model.PFy0(model.ID_Ppy5) = 0;
            model.PFy0(model.ID_PCy1) = 1.3;
            model.PFy0(model.ID_PDy1) = 1;
            model.PFy0(model.ID_PDy2) = 0;
            model.PFy0(model.ID_PDy3) = 0;
            model.PFy0(model.ID_PEy1) = -1;
            model.PFy0(model.ID_PEy2) = 0;
            model.PFy0(model.ID_PEy3) = 0;
            model.PFy0(model.ID_PEy4) = 0;
            model.PFy0(model.ID_PEy5) = 0;
            model.PFy0(model.ID_PKy1) = 10;
            model.PFy0(model.ID_PKy2) = 1.5;
            model.PFy0(model.ID_PKy3) = 0;
            model.PFy0(model.ID_PKy4) = 2;
            model.PFy0(model.ID_PKy5) = 0;
            model.PFy0(model.ID_PKy6) = 2.5;
            model.PFy0(model.ID_PKy7) = 0;
            model.PFy0(model.ID_PHy1) = 0;
            model.PFy0(model.ID_PHy2) = 0;
            model.PFy0(model.ID_PVy1) = 0;
            model.PFy0(model.ID_PVy2) = 0;
            model.PFy0(model.ID_PVy3) = 0.15;
            model.PFy0(model.ID_PVy4) = 0;

            model.PFx(model.ID_RBx1) = 5;
            model.PFx(model.ID_RBx2) = 8;
            model.PFx(model.ID_RBx3) = 0;
            model.PFx(model.ID_RCx1) = 1;
            model.PFx(model.ID_REx1) = 0;
            model.PFx(model.ID_REx2) = 0;
            model.PFx(model.ID_RHx1) = 0;

            model.PFy(model.ID_RBy1) = 7;
            model.PFy(model.ID_RBy2) = 2.5;
            model.PFy(model.ID_RBy3) = 0;
            model.PFy(model.ID_RBy4) = 0;
            model.PFy(model.ID_RCy1) = 1;
            model.PFy(model.ID_REy1) = 0;
            model.PFy(model.ID_REy2) = 0;
            model.PFy(model.ID_RHy1) = 0.02;
            model.PFy(model.ID_RHy2) = 0;
            model.PFy(model.ID_RVy1) = 0;
            model.PFy(model.ID_RVy2) = 0;
            model.PFy(model.ID_RVy3) = -0.2;
            model.PFy(model.ID_RVy4) = 14;
            model.PFy(model.ID_RVy5) = 1.9;
            model.PFy(model.ID_RVy6) = 10;
        end

        % resetModelToApproximateGT
        %
        % Approximate model parameters for a GT car tire. The keyword here being "approximate"...
        %
        % This is tuned to have the following trends:
        %   -Fx/Fz decreases as Fz increases
        %   -Peak Fx/Fz occurs at higher slip angle as Fz increases
        %   -Fx decreases as slip angle increases
        %   -Fx decreases as camber angle increases
        %   -Longitudinal stiffness decreases as pressure decreases
        %   -Fx decreases less for lower pressures than higher pressures
        %   -Fy/Fz decreases as Fz increases
        %   -Peak Fy/Fz occurs at higher slip angle as Fz increases
        %   -Fy decreases as longitudinal slip increases
        %   -Peak Fy increases as camber increases
        %   -Fy at zero slip angle increases as camber increases
        %   -Lateral stiffness decreases as pressure decreases
        %   -Fy decreases less for lower pressures than higher pressures
        %
        % This also sets constraints on each parameter to fall into intervals with reasonable
        % values, although the intervals are quite large.
        function model = resetModelToApproximateGT(model)
            model.Fz0 = 4000;
            model.P0 = 27.5;

            % Fx0
            model.PFx0(model.ID_Ppx1) = 1.0;
            model.PFx0(model.ID_Ppx2) = 0;
            model.PFx0(model.ID_Ppx3) = -0.2;
            model.PFx0(model.ID_Ppx4) = -0.2;
            model.PFx0(model.ID_PCx1) = 1.5;
            model.PFx0(model.ID_PDx1) = 2;
            model.PFx0(model.ID_PDx2) = -0.1;
            model.PFx0(model.ID_PDx3) = 5;
            model.PFx0(model.ID_PEx1) = -1.0;
            model.PFx0(model.ID_PEx2) = 0;
            model.PFx0(model.ID_PEx3) = 0;
            model.PFx0(model.ID_PEx4) = 0;
            model.PFx0(model.ID_PKx1) = 75;
            model.PFx0(model.ID_PKx2) = -10;
            model.PFx0(model.ID_PKx3) = 0;
            model.PFx0(model.ID_PHx1) = 0;
            model.PFx0(model.ID_PHx2) = 0;
            model.PFx0(model.ID_PVx1) = 0;
            model.PFx0(model.ID_PVx2) = 0;

            model = model.Fx0Limits(model.ID_Ppx1, -3, 3);
            model = model.Fx0Limits(model.ID_Ppx2, -20, 20);
            model = model.Fx0Limits(model.ID_Ppx3, -1, 1);
            model = model.Fx0Limits(model.ID_Ppx4, -3, 0);
            model = model.Fx0Limits(model.ID_PCx1, 1, 2);
            model = model.Fx0Limits(model.ID_PDx1, 0.5, 3);
            model = model.Fx0Limits(model.ID_PDx2, -0.5, 0.5);
            model = model.Fx0Limits(model.ID_PDx3, -20, 20);
            model = model.Fx0Limits(model.ID_PEx1, -100, 1);
            model = model.Fx0Limits(model.ID_PEx2, -5, 5);
            model = model.Fx0Limits(model.ID_PEx3, -5, 5);
            model = model.Fx0Limits(model.ID_PKx1, -150, 0);
            model = model.Fx0Limits(model.ID_PKx2, -50, 0);
            model = model.Fx0Limits(model.ID_PKx3, 0, 2);
            model = model.Fx0Limits(model.ID_PHx1, -0.1, 0.1);
            model = model.Fx0Limits(model.ID_PHx2, -0.1, 0.1);
            model = model.Fx0Limits(model.ID_PVx1, -0.5, 0.5);
            model = model.Fx0Limits(model.ID_PVx2, -0.5, 0.5);

            % Fx
            model.PFx(model.ID_RBx1) = 5;
            model.PFx(model.ID_RBx2) = 0;
            model.PFx(model.ID_RBx3) = 0;
            model.PFx(model.ID_RCx1) = 0.5;
            model.PFx(model.ID_REx1) = 0;
            model.PFx(model.ID_REx2) = 0;
            model.PFx(model.ID_RHx1) = 0;

            model = model.FxLimits(model.ID_RBx1, 0, 10);
            model = model.FxLimits(model.ID_RBx2, 0, 10);
            model = model.FxLimits(model.ID_RBx3, -200, 200);
            model = model.FxLimits(model.ID_RCx1, 0, 1.5);
            model = model.FxLimits(model.ID_REx1, -10, 1);
            model = model.FxLimits(model.ID_REx2, -10, 1);
            model = model.FxLimits(model.ID_RHx1, -0.1, 0.1);

            % Fy0
            model.PFy0(model.ID_Ppy1) = 1.0;
            model.PFy0(model.ID_Ppy2) = 0;
            model.PFy0(model.ID_Ppy3) = -0.2;
            model.PFy0(model.ID_Ppy4) = -0.2;
            model.PFy0(model.ID_Ppy5) = 0;
            model.PFy0(model.ID_PCy1) = 1.5;
            model.PFy0(model.ID_PDy1) = 2;
            model.PFy0(model.ID_PDy2) = -0.1;
            model.PFy0(model.ID_PDy3) = 0;
            model.PFy0(model.ID_PEy1) = -1.0;
            model.PFy0(model.ID_PEy2) = 0;
            model.PFy0(model.ID_PEy3) = 0;
            model.PFy0(model.ID_PEy4) = 0;
            model.PFy0(model.ID_PEy5) = 0;
            model.PFy0(model.ID_PKy1) = 80;
            model.PFy0(model.ID_PKy2) = 1.5;
            model.PFy0(model.ID_PKy3) = 0;
            model.PFy0(model.ID_PKy4) = 1;
            model.PFy0(model.ID_PKy5) = 0;
            model.PFy0(model.ID_PKy6) = 4.0;
            model.PFy0(model.ID_PKy7) = 0;
            model.PFy0(model.ID_PHy1) = 0;
            model.PFy0(model.ID_PHy2) = 0;
            model.PFy0(model.ID_PVy1) = 0;
            model.PFy0(model.ID_PVy2) = 0;
            model.PFy0(model.ID_PVy3) = 0.5;
            model.PFy0(model.ID_PVy4) = 0;

            model = model.Fy0Limits(model.ID_Ppy1, 0, 3);
            model = model.Fy0Limits(model.ID_Ppy2, -3, 0);
            model = model.Fy0Limits(model.ID_Ppy3, -1, 1);
            model = model.Fy0Limits(model.ID_Ppy4, -3, 0);
            model = model.Fy0Limits(model.ID_Ppy5, -2, 2);
            model = model.Fy0Limits(model.ID_PCy1, 1, 2);
            model = model.Fy0Limits(model.ID_PDy1, 0.5, 3);
            model = model.Fy0Limits(model.ID_PDy2, -0.5, 0.5);
            model = model.Fy0Limits(model.ID_PDy3, -20, 20);
            model = model.Fy0Limits(model.ID_PEy1, -100, 1);
            model = model.Fy0Limits(model.ID_PEy2, -5, 5);
            model = model.Fy0Limits(model.ID_PEy3, 0, 0);
            model = model.Fy0Limits(model.ID_PEy4, 0, 0);
            model = model.Fy0Limits(model.ID_PEy5, -150, 1000);
            model = model.Fy0Limits(model.ID_PKy1, 0, 200);
            model = model.Fy0Limits(model.ID_PKy2, 0, 5);
            model = model.Fy0Limits(model.ID_PKy3, -5, 5);
            model = model.Fy0Limits(model.ID_PKy4, 0, 2);
            model = model.Fy0Limits(model.ID_PKy5, -100, 100);
            model = model.Fy0Limits(model.ID_PKy6, 0, 10);
            model = model.Fy0Limits(model.ID_PKy7, 0, 10);
            model = model.Fy0Limits(model.ID_PHy1, -0.1, 0.1);
            model = model.Fy0Limits(model.ID_PHy2, -0.05, 0.05);
            model = model.Fy0Limits(model.ID_PVy1, -0.2, 0.2);
            model = model.Fy0Limits(model.ID_PVy2, -0.2, 0.2);
            model = model.Fy0Limits(model.ID_PVy3, -2, 2);
            model = model.Fy0Limits(model.ID_PVy4, -2, 2);

            % Fy
            model.PFy(model.ID_RBy1) = 5;
            model.PFy(model.ID_RBy2) = 0;
            model.PFy(model.ID_RBy3) = 0;
            model.PFy(model.ID_RBy4) = 0;
            model.PFy(model.ID_RCy1) = 0.5;
            model.PFy(model.ID_REy1) = -1.0;
            model.PFy(model.ID_REy2) = 0;
            model.PFy(model.ID_RHy1) = 0;
            model.PFy(model.ID_RHy2) = 0;
            model.PFy(model.ID_RVy1) = 0;
            model.PFy(model.ID_RVy2) = 0;
            model.PFy(model.ID_RVy3) = 0;
            model.PFy(model.ID_RVy4) = 10;
            model.PFy(model.ID_RVy5) = 1;
            model.PFy(model.ID_RVy6) = 10;

            model = model.FyLimits(model.ID_RBy1, 0, 10);
            model = model.FyLimits(model.ID_RBy2, 0, 10);
            model = model.FyLimits(model.ID_RBy3, 0, 0.5);
            model = model.FyLimits(model.ID_RBy4, -200, 200);
            model = model.FyLimits(model.ID_RCy1, 0, 1.5);
            model = model.FyLimits(model.ID_REy1, -10, 1);
            model = model.FyLimits(model.ID_REy2, -10, 1);
            model = model.FyLimits(model.ID_RHy1, -0.1, 0.1);
            model = model.FyLimits(model.ID_RHy2, -0.1, 0.1);
            model = model.FyLimits(model.ID_RVy1, -0.1, 0.1);
            model = model.FyLimits(model.ID_RVy2, -0.1, 0.1);
            model = model.FyLimits(model.ID_RVy3, -1, 0);
            model = model.FyLimits(model.ID_RVy4, 0, 20);
            model = model.FyLimits(model.ID_RVy5, 0, 2);
            model = model.FyLimits(model.ID_RVy6, 0, 20);
        end
    end

    methods (Access = protected)
        % getEstimatedForce
        %
        % INPUTS:
        %   model: PacejkaTireModel to evaluate
        %   type: 'Fx' or 'Fy' for longitudinal or lateral force
        % OUTPUTS:
        %  F_total: Total force of all 4 wheels
        %  FL: Front left wheel force
        %  FR: Front right wheel force
        %  RL: Rear left wheel force
        %  RR: Rear right wheel force
        function [F_total, FL, FR, RL, RR] = getEstimatedForce(this, model, type)
            V = [this.v, this.v, this.v, this.v];

            switch type
                case 'Fx'
                    [F_est_raw, ~] = model.Fx(this.cornerCombined('Fz'), ...
                        this.cornerCombined('P'), this.cornerCombined('kappa'), ...
                        this.cornerCombined('alpha'), this.cornerCombined('gamma'), ...
                        this.cornerCombined('Vsx'), V);
                case 'Fy'
                    [F_est_raw, ~] = model.Fy(this.cornerCombined('Fz'), ...
                        this.cornerCombined('P'), this.cornerCombined('kappa'), ...
                        this.cornerCombined('alpha'), this.cornerCombined('gamma'), ...
                        this.cornerCombined('Vsy'), V);
                otherwise
                    warning('Unexpected force type')
            end

            F_est_raw = reshape(F_est_raw, [this.n, 4])';
            FL = F_est_raw(1, :);
            FR = F_est_raw(2, :);
            RL = F_est_raw(3, :);
            RR = F_est_raw(4, :);
            F_total = sum(F_est_raw, 1);
        end
    end
end
