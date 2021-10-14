% PacejkaTireModelTest
%
% Unit tests for the PacejkaTireModel class.
classdef PacejkaTireModelTest < matlab.unittest.TestCase
    methods(Test)
        % Estimate the Fx0 parameters from measured data
        function fitFx0(this)
            % Create a new model with sample parameters
            model = PacejkaTireModel();
            model = this.resetModelToSampleParameters(model);
            [Fz, P, kappa, ~, gamma, Vs, V0] = this.getSampleDataFitInputs(model);

            [F_ref, ~] = model.Fx0(Fz, P, kappa, gamma, Vs, V0);

            % Perturb the model parameters
            model.PFx0 = this.perturbModelParameters(model.PFx0, 0.25);
            [F_perturbed, ~] = model.Fx0(Fz, P, kappa, gamma, Vs, V0);

            % Add noise to some sample data generated with the original model
            F_meas = normrnd(F_ref, 100);

            % Fit a new model
            model = model.fitFx0(F_meas, Fz, P, kappa, gamma, Vs, V0, 1, true);
            [F_fitted, ~] = model.Fx0(Fz, P, kappa, gamma, Vs, V0);

            % Make sure all force values are within a tolerance of 5% of the
            % peak force
            dF = F_ref - F_fitted;
            F_tolerance = 0.05 * max(abs(F_ref));
            this.verifyLessThanOrEqual(abs(dF), F_tolerance);

            % Debugging plot
%             figure;
%             hold on;
%             grid on;
%             xlabel('\kappa');
%             ylabel('F_x_0 [N]');
%             plot(kappa, F_ref, 'b-', 'DisplayName', 'Ref');
%             plot(kappa, F_perturbed, 'r-', 'DisplayName', 'Perturbed');
%             plot(kappa, F_meas, 'rx', 'DisplayName', 'Measured');
%             plot(kappa, F_fitted, 'g--', 'DisplayName', 'Fitted');
%             legend(gca, 'show');
        end

        % Estimate the Fy0 parameters from measured data
        function fitFy0(this)
            % Create a new model with sample parameters
            model = PacejkaTireModel();
            model = this.resetModelToSampleParameters(model);
            [Fz, P, ~, alpha, gamma, Vs, V0] = this.getSampleDataFitInputs(model);
            Vs = 0 * Vs;

            [F_ref, ~] = model.Fy0(Fz, P, alpha, gamma, Vs, V0);

            % Perturb the model parameters
            model.PFy0 = this.perturbModelParameters(model.PFy0, 0.25);
            [F_perturbed, ~] = model.Fy0(Fz, P, alpha, gamma, Vs, V0);

            % Add noise to some sample data generated with the original model
            F_meas = normrnd(F_ref, 100);

            % Fit a new model
            model = model.fitFy0(F_meas, Fz, P, alpha, gamma, Vs, V0, 1);
            [F_fitted, ~] = model.Fy0(Fz, P, alpha, gamma, Vs, V0);

            % Make sure all force values are within a tolerance of 5% of the
            % peak force
            dF = F_ref - F_fitted;
            F_tolerance = 0.05 * max(abs(F_ref));
            this.verifyLessThanOrEqual(abs(dF), F_tolerance);

            % Debugging plot
%             figure;
%             hold on;
%             grid on;
%             xlabel('\alpha [\circ]');
%             ylabel('F_x_0 [N]');
%             plot(rad2deg(alpha), F_ref, 'b-', 'DisplayName', 'Ref');
%             plot(rad2deg(alpha), F_perturbed, 'r-', 'DisplayName', 'Perturbed');
%             plot(rad2deg(alpha), F_meas, 'rx', 'DisplayName', 'Measured');
%             plot(rad2deg(alpha), F_fitted, 'g--', 'DisplayName', 'Fitted');
%             legend(gca, 'show');
        end

        % Estimate the Fx parameters from measured data
        function fitFx(this)
            % Create a new model with sample parameters
            model = PacejkaTireModel();
            model = this.resetModelToSampleParameters(model);
            [Fz, P, kappa, alpha, gamma, Vs, V0] = this.getSampleDataFitInputs(model);

            [F_ref, ~] = model.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Perturb the model parameters
            model.PFx0 = this.perturbModelParameters(model.PFx0, 0.25);
            model.PFx = this.perturbModelParameters(model.PFx, 0.25);
            [F_perturbed, ~] = model.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Add noise to some sample data generated with the original model
            F_meas = normrnd(F_ref, 100);

            % Fit a new model
            model = model.fitFx(F_meas, Fz, P, kappa, alpha, gamma, Vs, V0, 1, true);
            [F_fitted, ~] = model.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Make sure all force values are within a tolerance of 5% of the
            % peak force
            dF = F_ref - F_fitted;
            F_tolerance = 0.05 * max(abs(F_ref));
            this.verifyLessThanOrEqual(abs(dF), F_tolerance);

            % Debugging plot
%             figure;
%             hold on;
%             grid on;
%             xlabel('\kappa');
%             ylabel('F_x_0 [N]');
%             plot(kappa, F_ref, 'b-', 'DisplayName', 'Ref');
%             plot(kappa, F_perturbed, 'r-', 'DisplayName', 'Perturbed');
%             plot(kappa, F_meas, 'rx', 'DisplayName', 'Measured');
%             plot(kappa, F_fitted, 'g--', 'DisplayName', 'Fitted');
%             legend(gca, 'show');
        end

        % Estimate the Fy parameters from measured data
        function fitFy(this)
            % Create a new model with sample parameters
            model = PacejkaTireModel();
            model = this.resetModelToSampleParameters(model);
            [Fz, P, kappa, alpha, gamma, Vs, V0] = this.getSampleDataFitInputs(model);
            Vs = 0 * Vs;

            [F_ref, ~] = model.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Perturb the model parameters
            model.PFy0 = this.perturbModelParameters(model.PFy0, 0.25);
            model.PFy = this.perturbModelParameters(model.PFy, 0.25);
            [F_perturbed, ~] = model.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Add noise to some sample data generated with the original model
            F_meas = normrnd(F_ref, 100);

            % Fit a new model
            model = model.fitFy(F_meas, Fz, P, kappa, alpha, gamma, Vs, V0, 1);
            [F_fitted, ~] = model.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Make sure all force values are within a tolerance of 5% of the
            % peak force
            dF = F_ref - F_fitted;
            F_tolerance = 0.05 * max(abs(F_ref));
            this.verifyLessThanOrEqual(abs(dF), F_tolerance);

            % Debugging plot
%             figure;
%             hold on;
%             grid on;
%             xlabel('\alpha [\circ]');
%             ylabel('F_x_0 [N]');
%             plot(rad2deg(alpha), F_ref, 'b-', 'DisplayName', 'Ref');
%             plot(rad2deg(alpha), F_perturbed, 'r-', 'DisplayName', 'Perturbed');
%             plot(rad2deg(alpha), F_meas, 'rx', 'DisplayName', 'Measured');
%             plot(rad2deg(alpha), F_fitted, 'g--', 'DisplayName', 'Fitted');
%             legend(gca, 'show');
        end

        % Estimate the Fx and Fy parameters from measured data
        function fitFxFy(this)
            % Create a new model with sample parameters
            model = PacejkaTireModel();
            model = this.resetModelToSampleParameters(model);
            [Fz, P, kappa, alpha, gamma, Vs, V0] = this.getSampleDataFitInputs(model);
            delta = zeros(size(Fz));

            [Fx_ref, ~] = model.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);
            [Fy_ref, ~] = model.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Perturb the model parameters
            model.PFx0 = this.perturbModelParameters(model.PFx0, 0.25);
            model.PFx = this.perturbModelParameters(model.PFx, 0.25);

            model.PFy0 = this.perturbModelParameters(model.PFy0, 0.25);
            model.PFy = this.perturbModelParameters(model.PFy, 0.25);

            [Fx_perturbed, ~] = model.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);
            [Fy_perturbed, ~] = model.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Add noise to some sample data generated with the original model
            Fx_meas = normrnd(Fx_ref, 100);
            Fy_meas = normrnd(Fy_ref, 100);

            % Fit a new model
            model = model.fitFxFy(Fx_meas, Fy_meas, delta, Fz, P, kappa, alpha, gamma, Vs, Vs, ...
                V0, 1, true);
            [Fx_fitted, ~] = model.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);
            [Fy_fitted, ~] = model.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);

            % Make sure all force values are within a tolerance of 5% of the
            % peak force
            dFx = Fx_ref - Fx_fitted;
            Fx_tolerance = 0.05 * max(abs(Fx_ref));
            this.verifyLessThanOrEqual(abs(dFx), Fx_tolerance);

            dFy = Fy_ref - Fy_fitted;
            Fy_tolerance = 0.05 * max(abs(Fy_ref));
            this.verifyLessThanOrEqual(abs(dFy), Fy_tolerance);

            % Debugging plot
%             figure;
%             hold on;
%             grid on;
%             xlabel('\kappa');
%             ylabel('F_x_0 [N]');
%             plot(kappa, Fx_ref, 'b-', 'DisplayName', 'Ref');
%             plot(kappa, Fx_perturbed, 'r-', 'DisplayName', 'Perturbed');
%             plot(kappa, Fx_meas, 'rx', 'DisplayName', 'Measured');
%             plot(kappa, Fx_fitted, 'g--', 'DisplayName', 'Fitted');
%             legend(gca, 'show');
%
%             figure;
%             hold on;
%             grid on;
%             xlabel('\alpha [\circ]');
%             ylabel('F_x_0 [N]');
%             plot(rad2deg(alpha), Fy_ref, 'b-', 'DisplayName', 'Ref');
%             plot(rad2deg(alpha), Fy_perturbed, 'r-', 'DisplayName', 'Perturbed');
%             plot(rad2deg(alpha), Fy_meas, 'rx', 'DisplayName', 'Measured');
%             plot(rad2deg(alpha), Fy_fitted, 'g--', 'DisplayName', 'Fitted');
%             legend(gca, 'show');
        end
    end

    methods (Static, Access = public)

        % getSampleDataFitInputs
        %
        % Helper function to get sample input data for the force functions. This will generate
        % sweeps of slip ratio and slip angle, with all other values as a constant.
        %
        % OUTPUTS:
        %   Fz: Normal force
        %   P: Tire pressure
        %   kappa: Slip ratio
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vs: Slip velocity
        %   V0: Reference velocity
        function [Fz, P, kappa, alpha, gamma, Vs, V0] = getSampleDataFitInputs(model)
            n = 100;

            alpha_max = deg2rad(30.0);
            alpha = linspace(-alpha_max, alpha_max, n);

            kappa_max = 0.50;
            kappa = linspace(-kappa_max, kappa_max, n);

            Fz = model.Fz0 * ones(1, n);
            P = model.P0 * ones(1, n);
            gamma = zeros(1, n);
            V0 = 30.0 * ones(1, n);
            Vs = zeros(1, n);
        end

        % perturbModelParameters
        %
        % INPUTS:
        %   P_in: Parameter vector to perturb
        %   scale: Maximum coefficient value to scale a parameter value by
        % OUTPUTS:
        %   P: Parameter vector with values multiplied by a random value in the range
        %      [1 - scale, 1 + scale]
        function P = perturbModelParameters(P_in, scale)
            rand_coeffs = 1 + scale * 2 * (rand(size(P_in)) - 0.5);

            P = P_in;
            P(P ~= 0) = rand_coeffs(P ~= 0) .* P(P ~= 0);
        end

        % Resets all parameter values from Table 4.2 pg. 190 of "Tire and Vehicle Dynamics - 3rd
        % Edition, Hans Pacejka", as well as constrains all parameters that are set to zero.
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

            % Constrain all values at zero to remain at zero
            model = model.constrainParameters(find(model.PFx0 == 0), find(model.PFx == 0), ...
                find(model.PFy0 == 0), find(model.PFy == 0));
        end
    end
end
