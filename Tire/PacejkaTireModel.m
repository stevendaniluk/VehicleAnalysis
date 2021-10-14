% PacejkaTireModel
%
% Implementation of the Pacejka tire model from:
%   [1] Tire and Vehicle Dynamics - 3rd Edition, Hans Pacejka
%
% This implements the equations for Fx0, Fy0, Fx, and Fy as detailed in
% Section 4.3 of [1]. This does not currently implement the aligning torque
% equations or the extension for turn slip.
%
% This contains functionality for estimating the model parameters based on
% empircally measured data. Constraints can be placed on individual parameters
% during the optimization.
classdef PacejkaTireModel
    properties
        % Nominal normal load
        Fz0 = 0;
        % Nominal tire pressure
        P0 = 0;
        % Pure longitudinal slip coefficients
        PFx0 = [];
        % Pure side slip coefficients
        PFy0 = [];
        % Combined longitudinal force coefficients
        PFx = [];
        % Combined lateral force coefficients
        PFy = [];
        % Scaling factors
        lambda = [];
        % Spin factors (turn slip is ignored)
        zeta_0 = 1;
        zeta_1 = 1;
        zeta_2 = 1;
        zeta_3 = 1;
        zeta_4 = 1;
        % Constant for digressive friction scaling factor, see eq. 4.E8
        A_mu = 10;
        % Epsilons added to denominators in various equations
        epsilon_x = 1e-6;
        epsilon_y = 1e-6;
        epsilon_K = 1e-6;
        % Minimum values to allow for function parameters that must be >0
        B_min = 1e-6;
        C_min = 1e-6;
        D_min = 1e-6;
        % Optimization settings
        optim_options = optimoptions('lsqnonlin');
        % Bounds on model parameters to optimize
        Fx0_lb = [];
        Fx0_ub = [];
        Fy0_lb = [];
        Fy0_ub = [];
        Fx_lb = [];
        Fx_ub = [];
        Fy_lb = [];
        Fy_ub = [];
    end

    properties (Constant)
        % Indices of pure longitudinal slip coefficients
        ID_Ppx1 = 1;
        ID_Ppx2 = 2;
        ID_Ppx3 = 3;
        ID_Ppx4 = 4;
        ID_PCx1 = 5;
        ID_PDx1 = 6;
        ID_PDx2 = 7;
        ID_PDx3 = 8;
        ID_PEx1 = 9;
        ID_PEx2 = 10;
        ID_PEx3 = 11;
        ID_PEx4 = 12;
        ID_PKx1 = 13;
        ID_PKx2 = 14;
        ID_PKx3 = 15;
        ID_PHx1 = 16;
        ID_PHx2 = 17;
        ID_PVx1 = 18;
        ID_PVx2 = 19;

        DIM_Fx0 = 19;

        % Indices of pure side slip coefficients
        ID_Ppy1 = 1;
        ID_Ppy2 = 2;
        ID_Ppy3 = 3;
        ID_Ppy4 = 4;
        ID_Ppy5 = 5;
        ID_PCy1 = 6;
        ID_PDy1 = 7;
        ID_PDy2 = 8;
        ID_PDy3 = 9;
        ID_PEy1 = 10;
        ID_PEy2 = 11;
        ID_PEy3 = 12;
        ID_PEy4 = 13;
        ID_PEy5 = 14;
        ID_PKy1 = 15;
        ID_PKy2 = 16;
        ID_PKy3 = 17;
        ID_PKy4 = 18;
        ID_PKy5 = 19;
        ID_PKy6 = 20;
        ID_PKy7 = 21;
        ID_PHy1 = 22;
        ID_PHy2 = 23;
        ID_PVy1 = 24;
        ID_PVy2 = 25;
        ID_PVy3 = 26;
        ID_PVy4 = 27;

        DIM_Fy0 = 27;

        % Indices of combined longitudinal force coefficients
        ID_RBx1 = 1;
        ID_RBx2 = 2;
        ID_RBx3 = 3;
        ID_RCx1 = 4;
        ID_REx1 = 5;
        ID_REx2 = 6;
        ID_RHx1 = 7;

        DIM_Fx = 7;

        % Indices of combined lateral force coefficients
        ID_RBy1 = 1;
        ID_RBy2 = 2;
        ID_RBy3 = 3;
        ID_RBy4 = 4;
        ID_RCy1 = 5;
        ID_REy1 = 6;
        ID_REy2 = 7;
        ID_RHy1 = 8;
        ID_RHy2 = 9;
        ID_RVy1 = 10;
        ID_RVy2 = 11;
        ID_RVy3 = 12;
        ID_RVy4 = 13;
        ID_RVy5 = 14;
        ID_RVy6 = 15;

        DIM_Fy = 15;

        % Indices of scaling factors
        ID_Fz0 = 1;
        ID_mu_x = 2;
        ID_mu_y = 3;
        ID_mu_V = 4;
        ID_Ky_alpha = 5;
        ID_Cx = 6;
        ID_Cy = 7;
        ID_Ex = 8;
        ID_Ey = 9;
        ID_Hx = 10;
        ID_Hy = 11;
        ID_Vx = 12;
        ID_Vy = 13;
        ID_Ky_gamma = 14;
        ID_x_alpha = 15;
        ID_y_kappa = 16;

        DIM_lambda = 16;
    end

    methods (Access = public)

        function this = PacejkaTireModel()
            this = this.resetParameters();
            this = this.resetParameterLimits();

            % Define some initial optimization settings
            this.optim_options.Display = 'off';
            this.optim_options.MaxIterations = 500;
            this.optim_options.MaxFunctionEvaluations = 10000;
        end

        % printParameters
        %
        % Prints the value of all parameters to the console.
        function printParameters(this)
            % This implementation relies on the assumption that the indicies for
            % each parameter are class properties and are defined in order.
            names = properties(PacejkaTireModel);

            disp('Fx0 Parameters:');
            i = 1;
            for j=1:length(names)
                name = names{j};
                if startsWith(name, 'ID_P') && contains(name, 'x')
                    fprintf('\t %s: %.3f\n', name(4:end), this.PFx0(i));
                    i = i + 1;
                end
            end

            disp('Fx Parameters:');
            i = 1;
            for j=1:length(names)
                name = names{j};
                if startsWith(name, 'ID_R') && contains(name, 'x')
                    fprintf('\t %s: %.3f\n', name(4:end), this.PFx(i));
                    i = i + 1;
                end
            end

            disp('Fy0 Parameters:');
            i = 1;
            for j=1:length(names)
                name = names{j};
                if startsWith(name, 'ID_P') && contains(name, 'y')
                    fprintf('\t %s: %.3f\n', name(4:end), this.PFy0(i));
                    i = i + 1;
                end
            end

            disp('Fy Parameters:');
            i = 1;
            for j=1:length(names)
                name = names{j};
                if startsWith(name, 'ID_R') && contains(name, 'y')
                    fprintf('\t %s: %.3f\n', name(4:end), this.PFy(i));
                    i = i + 1;
                end
            end
        end

        % saveModel
        %
        % Saves a copy of this model.
        %
        % INPUTS:
        %   name: Filename to save as (optional, will begin with date if not present)
        function saveModel(this, name)
            if nargin == 1
                name = strcat('tire_model_', datestr(datetime, 'yyyy-mm-dd_HH-MM-SS'));
            end

            model = this;
            save(name, 'model');
        end

        % resetParameters
        %
        % Resets all coefficients to 0 and scalaing parameters to 1 (except for
        % lambda_mu_V, which is set to 0).
        function this = resetParameters(this)
            % Reset most parameters to zero, leaving a few non zero to produce
            % reasonable starting curves
            this.Fz0 = 1e3;
            this.P0 = 30.0;

            this.PFx0 = zeros(this.DIM_Fx0, 1);
            this.PFx0(this.ID_PCx1) = 1.5;
            this.PFx0(this.ID_PDx1) = 1;
            this.PFx0(this.ID_PEx1) = -1;
            this.PFx0(this.ID_PKx1) = 50;

            this.PFy0 = zeros(this.DIM_Fy0, 1);
            this.PFy0(this.ID_PCy1) = 1.5;
            this.PFy0(this.ID_PDy1) = 1;
            this.PFy0(this.ID_PEy1) = -1;
            this.PFy0(this.ID_PKy1) = 100;
            this.PFy0(this.ID_PKy2) = 1;
            this.PFy0(this.ID_PKy4) = 1;

            this.PFx = zeros(this.DIM_Fx, 1);
            this.PFx(this.ID_RBx1) = 1;
            this.PFx(this.ID_RCx1) = 1;

            this.PFy = zeros(this.DIM_Fy, 1);
            this.PFy(this.ID_RBy1) = 1;
            this.PFy(this.ID_RCy1) = 1;

            this.lambda = ones(this.DIM_lambda, 1);
            this.lambda(this.ID_mu_V) = 1;
        end

        % resetParameterLimits
        %
        % Sets all parameter limits be [-inf, inf].
        function this = resetParameterLimits(this)
            this.Fx0_lb = -inf(this.DIM_Fx0, 1);
            this.Fx0_ub = inf(this.DIM_Fx0, 1);

            this.Fy0_lb = -inf(this.DIM_Fy0, 1);
            this.Fy0_ub = inf(this.DIM_Fy0, 1);

            this.Fx_lb = -inf(this.DIM_Fx, 1);
            this.Fx_ub = inf(this.DIM_Fx, 1);

            this.Fy_lb = -inf(this.DIM_Fy, 1);
            this.Fy_ub = inf(this.DIM_Fy, 1);
        end

        % Fx0Limits
        %
        % INPUTS:
        %   id: Fx0 parameter id
        %   lower: Minimum allowed value
        %   upper: Maximum allowed value
        function this = Fx0Limits(this, id, lower, upper)
            this.Fx0_lb(id) = lower;
            this.Fx0_ub(id) = upper;
        end

        % FxLimits
        %
        % INPUTS:
        %   id: Fx parameter id
        %   lower: Minimum allowed value
        %   upper: Maximum allowed value
        function this = FxLimits(this, id, lower, upper)
            this.Fx_lb(id) = lower;
            this.Fx_ub(id) = upper;
        end

        % Fy0Limits
        %
        % INPUTS:
        %   id: Fy0 parameter id
        %   lower: Minimum allowed value
        %   upper: Maximum allowed value
        function this = Fy0Limits(this, id, lower, upper)
            this.Fy0_lb(id) = lower;
            this.Fy0_ub(id) = upper;
        end

        % FyLimits
        %
        % INPUTS:
        %   id: Fy parameter id
        %   lower: Minimum allowed value
        %   upper: Maximum allowed value
        function this = FyLimits(this, id, lower, upper)
            this.Fy_lb(id) = lower;
            this.Fy_ub(id) = upper;
        end

        % constrainParameters
        %
        % Constrains parameters at their current value.
        %
        % INPUTS:
        %   Fx0: Indices for Fx0 parameters
        %   Fx: Indices for Fx parameters
        %   Fy0: Indices for Fy0 parameters
        %   Fy: Indices for Fy parameters
        function this = constrainParameters(this, Fx0, Fx, Fy0, Fy)
            this.Fx0_lb(Fx0) = this.PFx0(Fx0);
            this.Fx0_ub(Fx0) = this.PFx0(Fx0);

            this.Fx_lb(Fx) = this.PFx(Fx);
            this.Fx_ub(Fx) = this.PFx(Fx);

            this.Fy0_lb(Fy0) = this.PFy0(Fy0);
            this.Fy0_ub(Fy0) = this.PFy0(Fy0);

            this.Fy_lb(Fy) = this.PFy(Fy);
            this.Fy_ub(Fy) = this.PFy(Fy);
        end

        % unconstrainParameters
        %
        % Removes constrains on parameters, setting the lower and upper bounds
        % to -inf and inf, respectively.
        %
        % INPUTS:
        %   Fx0: Indices for Fx0 parameters
        %   Fx: Indices for Fx parameters
        %   Fy0: Indices for Fy0 parameters
        %   Fy: Indices for Fy parameters
        function this = unconstrainParameters(this, Fx0, Fx, Fy0, Fy)
            this.Fx0_lb(Fx0) = -inf;
            this.Fx0_ub(Fx0) = inf;

            this.Fx_lb(Fx) = -inf;
            this.Fx_ub(Fx) = inf;

            this.Fy0_lb(Fy0) = -inf;
            this.Fy0_ub(Fy0) = inf;

            this.Fy_lb(Fy) = -inf;
            this.Fy_ub(Fy) = inf;
        end

        % constrainAllParameters
        %
        % Constrains all parameters to their current values.
        function this = constrainAllParameters(this)
            Fx0 = 1:this.DIM_Fx0;
            Fx = 1:this.DIM_Fx;
            Fy0 = 1:this.DIM_Fy0;
            Fy = 1:this.DIM_Fy;

            this = this.constrainParameters(Fx0, Fx, Fy0, Fy);
        end

        % unconstrainAllParameters
        %
        % Removes constrains from all parameters.
        function this = unconstrainAllParameters(this)
            Fx0 = 1:this.DIM_Fx0;
            Fx = 1:this.DIM_Fx;
            Fy0 = 1:this.DIM_Fy0;
            Fy = 1:this.DIM_Fy;

            this = this.unconstrainParameters(Fx0, Fx, Fy0, Fy);
        end

        % constrainPressureParameters
        %
        % Constrains all pressure related parameters to their current values.
        function this = constrainPressureParameters(this)
            Fx0 = this.ID_Ppx1:this.ID_Ppx4;
            Fx = [];
            Fy0 = this.ID_Ppy1:this.ID_Ppy5;
            Fy = [];

            this = this.constrainParameters(Fx0, Fx, Fy0, Fy);
        end

        % unconstrainPressureParameters
        %
        % Removes constraints from all pressure related parameters.
        function this = unconstrainPressureParameters(this)
            Fx0 = this.ID_Ppx1:this.ID_Ppx4;
            Fx = [];
            Fy0 = this.ID_Ppy1:this.ID_Ppy5;
            Fy = [];

            this = this.unconstrainParameters(Fx0, Fx, Fy0, Fy);
        end

        % constrainCamberParameters
        %
        % Constrains all camber related parameters to their current values.
        function this = constrainCamberParameters(this)
            Fx0 = this.ID_PDx3;
            Fx = this.ID_RBx3;
            Fy0 = [this.ID_Ppy5, this.ID_PDy3, this.ID_PEy4:this.ID_PEy5, this.ID_PKy3, ...
                   this.ID_PKy5:this.ID_PKy7, this.ID_PVy3:this.ID_PVy4];
            Fy = [this.ID_RBy4, this.ID_RVy3];

            this = this.constrainParameters(Fx0, Fx, Fy0, Fy);
        end

        % unconstrainCamberParameters
        %
        % Removes constraints from all camber related parameters.
        function this = unconstrainCamberParameters(this)
            Fx0 = this.ID_PDx3;
            Fx = this.ID_RBx3;
            Fy0 = [this.ID_Ppy5, this.ID_PDy3, this.ID_PEy4:this.ID_PEy5, this.ID_PKy3, ...
                   this.ID_PKy5:this.ID_PKy7, this.ID_PVy3:this.ID_PVy4];
            Fy = [this.ID_RBy4, this.ID_RVy3];

            this = this.unconstrainParameters(Fx0, Fx, Fy0, Fy);
        end

        % zeroPressureParameters
        %
        % Sets all pressure related parameters to zero.
        %
        % INPUTS:
        %   Fx0: Set true to zero Fx0 parameters
        %   Fy0: Set true to zero Fy0 parameters
        function this = zeroPressureParameters(this, Fx0, Fy0)
            if Fx0
                this.PFx0(this.ID_Ppx1:this.ID_Ppx4) = 0;
            end
            if Fy0
                this.PFy0(this.ID_Ppy1:this.ID_Ppy5) = 0;
            end
        end

        % Fx0
        %
        % Computes the longitudinal force due to pure longitudinal slip. This implements equations
        % 4.E9-4.E18.
        %
        % INPUTS:
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        % OUTPUTS:
        %   Fx: Longitudinal force
        %   params: Structure with force function parameters {mu, B, C, D, etc}
        function [Fx0, params] = Fx0(this, Fz, P, kappa, gamma, Vs, V0)
            % Eqs. 4.E1, 4.E2a, 4.E2b
            [~, dfz] = this.dfz(Fz);
            dp = this.dp(P);
            % Eqs. 4.E7 and 4.E8
            [lamda_mu_x_star, lamda_mu_x_prime] = ...
                this.frictionFactors(this.lambda(this.ID_mu_x), Vs, V0);
            % Eq. 4.E13
            params.mu_x = (this.PFx0(this.ID_PDx1) + this.PFx0(this.ID_PDx2)* dfz) .* ...
                (1 + this.PFx0(this.ID_Ppx3) * dp + this.PFx0(this.ID_Ppx4) * dp.^2) ...
                .* (1 - this.PFx0(this.ID_PDx3) * gamma.^2) .* lamda_mu_x_star;
            % Eq. 4.E15
            K_x_kappa = Fz .* (this.PFx0(this.ID_PKx1) + this.PFx0(this.ID_PKx2) * dfz) .* ...
                exp(this.PFx0(this.ID_PKx3) * dfz) .* ...
                (1 + this.PFx0(this.ID_Ppx1) * dp + this.PFx0(this.ID_Ppx2) * dp.^2);
            % Eq. 4.E17
            params.S_Hx = (this.PFx0(this.ID_PHx1) + this.PFx0(this.ID_PHx2) * dfz) * ...
                this.lambda(this.ID_Hx);
            % Eq. 4.E18
            params.S_Vx = Fz .* (this.PFx0(this.ID_PVx1) + this.PFx0(this.ID_PVx2) * dfz) .* ...
                lamda_mu_x_prime * this.lambda(this.ID_Vx) * this.zeta_1;
            % Eq. 4.E11
            params.Cx = this.PFx0(this.ID_PCx1) * this.lambda(this.ID_Cx);
            params.Cx = max(params.Cx, this.C_min);
            % Eq. 4.E12
            params.Dx = params.mu_x .* Fz * this.zeta_1;
            params.Dx = max(params.Dx, this.D_min);
            % Eq. 4.E16
            params.Bx = K_x_kappa ./ (params.Cx .* params.Dx + this.epsilon_x);
            % Eq. 4.E14
            params.Ex = (this.PFx0(this.ID_PEx1) + this.PFx0(this.ID_PEx2) * dfz + ...
                this.PFx0(this.ID_PEx3) * dfz.^2) .* ...
                (1 - this.PFx0(this.ID_PEx4) * sign(kappa + params.S_Hx)) * this.lambda(this.ID_Ex);
            params.Ex = min(1, params.Ex);
            % Eq. 4.E9
            Fx0 = this.F0(...
                kappa, params.Bx, params.Cx, params.Dx, params.Ex, params.S_Hx, params.S_Vx);
        end

        % Fy0
        %
        % Computes the lateral force due to pure side slip. This implements equations
        % 4.E19-4.E30.
        %
        % INPUTS:
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        % OUTPUTS:
        %   Fy: Lateral force
        %   params: Structure with force function parameters {mu, B, C, D, etc}
        function [Fy0, params] = Fy0(this, Fz, P, alpha, gamma, Vs, V0)
            % Eq. 4.E3
            alpha_star = tan(alpha);
            % Eq. 4.E4
            gamma_star = sin(gamma);
            % Eqs. 4.E1, 4.E2a, 4.E2b
            [Fz0_prime, dfz] = this.dfz(Fz);
            dp = this.dp(P);
            % Eqs. 4.E7 and 4.E8
            [lamda_mu_y_star, lamda_mu_y_prime] = ...
                this.frictionFactors(this.lambda(this.ID_mu_y), Vs, V0);
            % Eq. 4.E25
            Ky_alpha = this.PFy0(this.ID_PKy1) * Fz0_prime .* (1 + this.PFy0(this.ID_Ppy1) * dp) ...
                .* (1 - this.PFy0(this.ID_PKy3) * abs(gamma_star)) ...
                .* sin(this.PFy0(this.ID_PKy4) * ...
                atan((Fz ./ Fz0_prime) ./ ...
                ((this.PFy0(this.ID_PKy2) + this.PFy0(this.ID_PKy5) * gamma_star.^2) .* ...
                (1 + this.PFy0(this.ID_Ppy2) * dp)))) * this.zeta_3 * this.lambda(this.ID_Ky_alpha);
            % Eq. 4.E25
            Ky_gamma_0 = Fz .* (this.PFy0(this.ID_PKy6) + this.PFy0(this.ID_PKy7) * dfz) .* ...
                (1 + this.PFy0(this.ID_Ppy5) * dp) * this.lambda(this.ID_Ky_gamma);
            % Eq. 4.E28
            S_Vy_gamma = Fz .* (this.PFy0(this.ID_PVy3) + this.PFy0(this.ID_PVy4) * dfz) .* ...
                gamma_star * this.lambda(this.ID_Ky_gamma) .* lamda_mu_y_prime * this.zeta_2;
            % Eq. 4.E27
            params.S_Hy = (this.PFy0(this.ID_PHy1) + this.PFy0(this.ID_PHy2) * dfz) *...
                this.lambda(this.ID_Hy) + ...
                this.zeta_0 * (Ky_gamma_0 .* gamma_star - S_Vy_gamma) ./ ...
                (Ky_alpha + this.epsilon_K) + this.zeta_4 - 1;
            % Eq. 4.E29
            params.S_Vy = Fz .* (this.PFy0(this.ID_PVy1) + this.PFy0(this.ID_PVy2) * dfz) .* ...
                this.lambda(this.ID_Vy) .* lamda_mu_y_prime * this.zeta_2 + S_Vy_gamma;
            % Eq. 4.E30
            params.Ey = (this.PFy0(this.ID_PEy1) + this.PFy0(this.ID_PEy2) * dfz) .* ...
                (1 + this.PFy0(this.ID_PEy5) * gamma_star.^2 - ...
                (this.PFy0(this.ID_PEy3) + this.PFy0(this.ID_PEy4) * gamma_star) .* ...
                sign(alpha_star + params.S_Hy)) * this.lambda(this.ID_Ey);
            params.Ey = min(params.Ey, 1);
            % Eq. 4.E23
            params.mu_y = (this.PFy0(this.ID_PDy1) + this.PFy0(this.ID_PDy2) * dfz) .* ...
                (1 + this.PFy0(this.ID_Ppy3) * dp + this.PFy0(this.ID_Ppy4) .* dp.^2) ...
                .* (1 - this.PFy0(this.ID_PDy3) * gamma_star.^2) .* lamda_mu_y_star;
            % Eq. 4.E22
            params.Dy = params.mu_y .* Fz * this.zeta_2;
            params.Dy = max(params.Dy, this.D_min);
            % Eq. 4.E21
            params.Cy = this.PFy0(this.ID_PCy1) * this.lambda(this.ID_Cy);
            params.Cy = max(params.Cy, this.C_min);
            % Eq. 4.E26
            params.By = Ky_alpha ./ (params.Cy .* params.Dy + this.epsilon_y);
            % Eq. 4.E19
            Fy0 = this.F0(...
                alpha_star, params.By, params.Cy, params.Dy, params.Ey, params.S_Hy, params.S_Vy);
        end

        % Fx
        %
        % Computes the longitudinal force due to combined longitudinal and side slip. This
        % implements equations 4.E50-4.E57.
        %
        % INPUTS:
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        % OUTPUTS:
        %   Fx: Longitudinal force
        %   params: Structure with force function parameters {mu, B, C, D, etc}
        function [Fx, params] = Fx(this, Fz, P, kappa, alpha, gamma, Vs, V0)
            % Eq. 4.E3
            alpha_star = tan(alpha);
            % Eq. 4.E4
            gamma_star = sin(gamma);
            % Eqs. 4.E1, 4.E2a, 4.E2b
            [~, dfz] = this.dfz(Fz);
            % Eq. 4.E9
            [Fx0, params] = this.Fx0(Fz, P, kappa, gamma, Vs, V0);
            % Eq. 4.E54
            params.Bx_alpha = ...
                (this.PFx(this.ID_RBx1) + this.PFx(this.ID_RBx3) * gamma_star.^2) .* ...
                cos(atan(this.PFx(this.ID_RBx2) * kappa)) * this.lambda(this.ID_x_alpha);
            params.Bx_alpha = max(params.Bx_alpha, this.B_min);
            % Eq. 4.E55
            params.Cx_alpha = this.PFx(this.ID_RCx1);
            % Eq. 4.E56
            params.Ex_alpha = this.PFx(this.ID_REx1) + this.PFx(this.ID_REx2) * dfz;
            params.Ex_alpha = min(params.Ex_alpha, 1);
            % Eq. 4.E57
            params.S_Hx_alpha = this.PFx(this.ID_RHx1);
            params.S_Vx_alpha = 0;
            % Eq. 4.50
            [Fx, params.Gx_alpha] = this.FCombined(alpha_star, Fx0, params.Bx_alpha, ...
                params.Cx_alpha, params.Ex_alpha, params.S_Hx_alpha, params.S_Vx_alpha);
        end

        % Fy
        %
        % Computes the laetral force due to combined longitudinal and side slip. This
        % implements equations 4.E58-4.E67.
        %
        % INPUTS:
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        % OUTPUTS:
        %   Fy: Lateral force
        %   params: Structure with force function parameters {mu, B, C, D, etc}
        function [Fy, params] = Fy(this, Fz, P, kappa, alpha, gamma, Vs, V0)
            % Eq. 4.E3
            alpha_star = tan(alpha);
            % Eq. 4.E4
            gamma_star = sin(gamma);
            % Eqs. 4.E1, 4.E2a, 4.E2b
            [~, dfz] = this.dfz(Fz);
            % Eq. 4.E9
            [Fy0, params] = this.Fy0(Fz, P, alpha, gamma, Vs, V0);
            % Eq. 4.E62
            params.By_kappa = ...
                (this.PFy(this.ID_RBy1) + this.PFy(this.ID_RBy4) * gamma_star.^2) .* ...
                cos(atan(this.PFy(this.ID_RBy2) * (alpha_star - this.PFy(this.ID_RBy3)))) * ...
                this.lambda(this.ID_y_kappa);
            params.By_kappa = max(params.By_kappa, this.B_min);
            % Eq. 4.E63
            params.Cy_kappa = this.PFy(this.ID_RCy1);
            % Eq. 4.E64
            params.Ey_kappa = this.PFy(this.ID_REy1) + this.PFy(this.ID_REy2) * dfz;
            params.Ey_kappa = min(params.Ey_kappa, 1);
            % Eq. 4.E67
            mu_y = params.mu_y;
            Dy = mu_y .* Fz .* (this.PFy(this.ID_RVy1) + this.PFy(this.ID_RVy2) * dfz + ...
                this.PFy(this.ID_RVy3) * gamma_star) .* ...
                cos(atan(this.PFy(this.ID_RVy4) * alpha_star)) .* this.zeta_2;
            % Eq. 4.E65
            params.S_Hy_kappa = this.PFy(this.ID_RHy1) + this.PFy(this.ID_RHy2) * dfz;
            % Eq. 4.E66
            params.S_Vy_kappa = ...
                Dy .* sin(this.PFy(this.ID_RVy5) * atan(this.PFy(this.ID_RVy6) * kappa)) * ...
                this.lambda(this.ID_Vy);
            % Eq. 4.50
            [Fy, params.Gy_kappa] = this.FCombined(kappa, Fy0, params.By_kappa, ...
                params.Cy_kappa, params.Ey_kappa, params.S_Hy_kappa, params.S_Vy_kappa);
        end

        % fitFx0
        %
        % Fits the tire model parameters in a least squares sense to match
        % measured longitudinal force under pure longitudinal slip.
        %
        % INPUTS:
        %   Fx_meas: Measured longitudinal force
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        %   n: Number of tires in the data
        %   driven: Logical vector of which tires are driven tires
        function this = fitFx0(this, Fx_meas, Fz, P, kappa, gamma, Vs, V0, n, driven)
            disp('Fitting Fx0 parameters...');

            % Will need to duplicate the reference velocity for each corner
            V0 = repmat(V0, [1, n]);

            objective = @(p, x) this.FObjective(p, Fx_meas, Fz, P, kappa, [], gamma, Vs, V0, n, ...
                driven, 'Fx0');
            this.PFx0 = lsqnonlin(objective, this.PFx0, this.Fx0_lb, this.Fx0_ub, ...
                this.optim_options);

            disp('Done fitting Fx0 parameters');
        end

        % fitFy0
        %
        % Fits the tire model parameters in a least squares sense to match
        % measured lateral force under pure side slip.
        %
        % INPUTS:
        %   Fy_meas: Measured lateral force
        %   Fz: Normal load
        %   P: Tire pressure
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        %   n: Number of tires in the data
        function this = fitFy0(this, Fy_meas, Fz, P, alpha, gamma, Vs, V0, n)
            disp('Fitting Fy0 parameters...');

            % Will need to duplicate the reference velocity for each corner
            V0 = repmat(V0, [1, n]);

            objective = @(p, x) this.FObjective(p, Fy_meas, Fz, P, [], alpha, gamma, Vs, V0, n, ...
                true(1, n), 'Fy0');
            this.PFy0 = lsqnonlin(objective, this.PFy0, this.Fy0_lb, this.Fy0_ub, ...
            this.optim_options);

            disp('Done fitting Fy0 parameters');
        end

        % fitFx
        %
        % Fits the tire model parameters in a least squares sense to match
        % measured longitudinal force under combined longitudinal and side slip.
        %
        % INPUTS:
        %   Fx_meas: Measured longitudinal force
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        %   n: Number of tires in the data
        %   driven: Logical vector of which tires are driven tires
        function this = fitFx(this, Fx_meas, Fz, P, kappa, alpha, gamma, Vs, V0, n, driven)
            disp('Fitting Fx parameters...');

            % Will need to duplicate the reference velocity for each corner
            V0 = repmat(V0, [1, n]);

            objective = @(p, x) this.FObjective(p, Fx_meas, Fz, P, kappa, alpha, gamma, Vs, V0, ...
                n, driven, 'Fx');

            p0 = [this.PFx0; this.PFx];
            p_fit = lsqnonlin(objective, p0, [this.Fx0_lb; this.Fx_lb], ...
                [this.Fx0_ub; this.Fx_ub], this.optim_options);
            this.PFx0 = p_fit(1:this.DIM_Fx0);
            this.PFx = p_fit(this.DIM_Fx0 + 1:end);

            disp('Done fitting Fx parameters');
        end

        % fitFy
        %
        % Fits the tire model parameters in a least squares sense to match
        % measured lateral force under combined longitudinal and side slip.
        %
        % INPUTS:
        %   Fy_meas: Measured lateral force
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vs: Contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        %   n: Number of tires in the data
        function this = fitFy(this, Fy_meas, Fz, P, kappa, alpha, gamma, Vs, V0, n)
            disp('Fitting Fy parameters...');

            % Will need to duplicate the reference velocity for each corner
            V0 = repmat(V0, [1, n]);

            objective = @(p, x) this.FObjective(p, Fy_meas, Fz, P, kappa, alpha, gamma, Vs, V0, ...
                n, true(1, n), 'Fy');

            p0 = [this.PFy0; this.PFy];
            p_fit = lsqnonlin(objective, p0, [this.Fy0_lb; this.Fy_lb], ...
                [this.Fy0_ub; this.Fy_ub], this.optim_options);

            this.PFy0 = p_fit(1:this.DIM_Fy0);
            this.PFy = p_fit(this.DIM_Fy0 + 1:end);

            disp('Done fitting Fy parameters');
        end

        % fitFxFy
        %
        % Fits both the Fx and Fy tire model parameters in a least squares sense to match
        % measured longitudinal and lateral force under combine slip conditions.
        %
        % INPUTS:
        %   Fx_meas: Measured longitudinal force in the vehicle body frame
        %   Fy_meas: Measured lateral force in the vehicle body frame
        %   delta: Wheel angle
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vsx: Longtiudinal contact patch slip velocity magnitude
        %   Vsy: Lateral contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        %   n: Number of tires in the data
        %   driven: Logical vector of which tires are driven tires
        function this = fitFxFy(this, Fx_meas, Fy_meas, delta, Fz, P, kappa, alpha, gamma, Vsx, ...
                Vsy, V0, n, driven)
            disp('Fitting Fx and Fy...');

            % Will need to duplicate the reference velocity for each corner
            V0 = repmat(V0, [1, n]);

            objective = @(p) this.fitFxFyObjective(p, Fx_meas, Fy_meas, delta, Fz, P, kappa, ...
                alpha, gamma, Vsx, Vsy, V0, n, driven);

            p0 = [this.PFx0; this.PFx; this.PFy0; this.PFy];
            lb = [this.Fx0_lb; this.Fx_lb; this.Fy0_lb; this. Fy_lb];
            ub = [this.Fx0_ub; this.Fx_ub; this.Fy0_ub; this. Fy_ub];

            p_fit = lsqnonlin(objective, p0, lb, ub, this.optim_options);

            this.PFx0 = p_fit(1:this.DIM_Fx0);
            this.PFx = p_fit((1:this.DIM_Fx) + this.DIM_Fx0);
            this.PFy0 = p_fit((1:this.DIM_Fy0) + this.DIM_Fx0 + this.DIM_Fx);
            this.PFy = p_fit((1:this.DIM_Fy) + this.DIM_Fx0 + this.DIM_Fx + this.DIM_Fy0);

            disp('Done fitting Fx and Fy');
        end

    end

    methods (Access = protected)

        % FObjective
        %
        % Helper for creating an objective function for optimization that
        % assigns parameter values before evaluation.
        function J = FObjective(this, p, F_meas, Fz, P, kappa, alpha, gamma, Vs, V0, n, ...
            driven, type)
            % Set our new parameters, and run the data through the model
            switch type
                case 'Fx0'
                    this.PFx0 = p;
                    [F, ~] = this.Fx0(Fz, P, kappa, gamma, Vs, V0);
                case 'Fx'
                    this.PFx0 = p(1:this.DIM_Fx0);
                    this.PFx = p(this.DIM_Fx0 + 1:end);
                    [F, ~] = this.Fx(Fz, P, kappa, alpha, gamma, Vs, V0);
                case 'Fy0'
                    this.PFy0 = p;
                    [F, ~] = this.Fy0(Fz, P, alpha, gamma, Vs, V0);
                case 'Fy'
                    this.PFy0 = p(1:this.DIM_Fy0);
                    this.PFy = p(this.DIM_Fy0 + 1:end);
                    [F, ~] = this.Fy(Fz, P, kappa, alpha, gamma, Vs, V0);
                otherwise
                    warning('Unexpected force type')
            end

            pts = length(F) / n;
            F = reshape(F, [pts, n]);

            if strcmp(type, 'Fx0') || strcmp(type, 'Fx')
                % Zero out any positive longitudinal forces for non driven wheels
                F(F(:, ~logical(driven)) > 0) = 0;
            end

            % Want the sum of forces for each tire
            F = sum(F, 2)';

            J = F_meas - F;
        end

        % fitFxFyObjective
        %
        % Objective function for simultaneously fitting Fx and Fy parameters.
        %
        % INPUTS:
        %   p: All parameters [PFx0, PFx, PFy0, PFy]
        %   Fx_meas: Measured longitudinal force
        %   Fy_meas: Measured lateral force
        %   delta: Wheel angle
        %   Fz: Normal load
        %   P: Tire pressure
        %   kappa: Longitudinal slip
        %   alpha: Slip angle
        %   gamma: Camber angle
        %   Vsx: Longtidunal contact patch slip velocity magnitude
        %   Vsy: Lateral contact patch slip velocity magnitude
        %   V0: Vehicle reference velocity
        %   n: Number of tires in the data
        %   driven: Logical vector of which tires are driven tires
        % OUTPUTS:
        %   J: Error in resultant force [(Fx - Fx_meas); (Fy - Fy_meas)]
        function J = fitFxFyObjective(this, p, Fx_meas, Fy_meas, delta, Fz, P, kappa, alpha, ...
                gamma, Vsx, Vsy, V0, n, driven)
            pts = length(delta) / n;

            % Set our new parameters, and run the data through the model
            this.PFx0 = p(1:this.DIM_Fx0);
            this.PFx = p((1:this.DIM_Fx) + this.DIM_Fx0);
            this.PFy0 = p((1:this.DIM_Fy0) + this.DIM_Fx0 + this.DIM_Fx);
            this.PFy = p((1:this.DIM_Fy) + this.DIM_Fx0 + this.DIM_Fx + this.DIM_Fy0);

            [Fx_tire, ~] = this.Fx(Fz, P, kappa, alpha, gamma, Vsx, V0);
            [Fy_tire, ~] = this.Fy(Fz, P, kappa, alpha, gamma, Vsy, V0);

            % Zero out any positive longitudinal forces for non driven wheels
            non_driven_indices = false(n, pts);
            non_driven_indices(~logical(driven), :) = true;
            non_driven_indices = reshape(non_driven_indices', [1, pts * n]);
            Fx_tire(Fx_tire(non_driven_indices) > 0) = 0;

            % Convert the individual tire forces to body forces
            Fx_body = cos(delta) .* Fx_tire - sin(delta) .* Fy_tire;
            Fy_body = sin(delta) .* Fx_tire + cos(delta) .* Fy_tire;

            % Sum the tire forces from each wheel
            Fx_body = reshape(Fx_body, [pts, n])';
            Fx_body = sum(Fx_body, 1);

            Fy_body = reshape(Fy_body, [pts, n])';
            Fy_body = sum(Fy_body, 1);

            % Compute our net force error
            Fx_error = Fx_meas - Fx_body;
            Fy_error = Fy_meas - Fy_body;
            J = [Fx_error; Fy_error];
        end

        % dfz
        %
        % INPUTS:
        %   Fz: Normal load
        % OUTPUTS:
        %   Fz0_prime: Scaled vertical load eq. 4.E1
        %   dfz: Normalized change in vertical load eq. 4.E2a
        function [Fz0_prime, dfz] = dfz(this, Fz)
            Fz0_prime = this.lambda(this.ID_Fz0) * this.Fz0;
            dfz = (Fz - Fz0_prime) / Fz0_prime;
        end

        % dp
        %
        % Eq. 4.E2b [1].
        %
        % INPUTS:
        %   P: Inflation pressure
        % OUTPUTS:
        %   dp: Normalized change in inflation pressure
        function dp = dp(this, P)
            dp = (P - this.P0) / this.P0;
        end

        % frictionFactors
        %
        % Eqs. 4.E7 and 4.E8.
        %
        % INPUTS:
        %   lambda_mu_xy: Peak friction coefficient scaling parameter
        %   Vs: Slip velocity magnitude
        %   V0: Vehicle reference velocity
        % OUTPUTS:
        %   lambda_mu_xy_star: Composite friction scaling factor, eq. 4.E7
        %   lambda_mu_xy_prime: Digressive friction factor, eq. 4.E8
        function [lambda_mu_xy_star, lambda_mu_xy_prime] = ...
                frictionFactors(this, lambda_mu_xy, Vs, V0)
            lambda_mu_xy_star = lambda_mu_xy ./ (1 + this.lambda(this.ID_mu_V) * abs(Vs ./ V0));
            lambda_mu_xy_prime = this.A_mu * lambda_mu_xy_star ./ ...
                (1 + (this.A_mu - 1) * lambda_mu_xy_star);
        end
    end

    methods(Static, Access = protected)

        % F0
        %
        % Basic form of the Pacejka tire model, eq. 4.49 [1].
        %
        % INPUTS:
        %   x: Independent variable
        %   B: Stiffneess factor
        %   C: Shape factor
        %   D: Peak value
        %   E: Curvature factor
        %   S_H: Horizontal shift
        %   S_V: Vertical shift
        % OUTPUTS:
        %   F0: Longitudinal/lateral force
        function F0 = F0(x, B, C, D, E, S_H, S_V)
            x_prime = x + S_H;
            F0 = D .* sin(C .* atan(B .* x_prime - E .* (B .* x_prime - atan(B .* x_prime)))) + S_V;
        end

        % FCombined
        %
        % Form of the combined force equations, eq. 4.E50-4.E53 and 4.E58-4.E61
        %
        % INPUTS:
        %   x: Independent variable
        %   F0: Pure longitudinal/lateral force
        %   B: Stiffneess factor
        %   C: Shape factor
        %   E: Curvature factor
        %   S_H: Horizontal shift
        %   S_V: Vertical shift
        % OUTPUTS:
        %   F: Longitudinal/lateral combined force
        %   G: Scaling factor
        function [F, G] = FCombined(x, F0, B, C, E, S_H, S_V)
            x_prime = x + S_H;
            G0 = cos(C .* atan(B .* S_H - E .* (B .* S_H - atan(B .* S_H))));
            G = cos(C .* atan(B .* x_prime - E .* (B .* x_prime - atan(B .* x_prime)))) ./ G0;
            G = max(G, 0);
            F = F0 .* G + S_V;
        end

    end
end
