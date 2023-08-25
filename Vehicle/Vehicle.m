% Setup
%
% Defines all the properties for a vehicle setup, as well as utilities for
% querying vehicle properties (e.g. wheel force from damper position).
%
% This uses the SAE coordinate convention: X forward, Y right, Z down.
classdef Vehicle
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Chassis

        % Wheelbase [m]
        L;
        % Front track width [m]
        track_f;
        % Rear track width [m]
        track_r;

        % Sprung mass at zero fuel load [kg]
        m_sprung = 0;
        % Sprung mass CoG distance from front axle [m]
        l_sprung = 0;
        % Sprung mass CoG lateral offset (+ve towards right)
        dy_sprung = 0;
        % Sprung mass CoG height for a ride height of zero [m]
        h_sprung = 0;

        % Unsprung mass for front axle [kg]
        m_unsprung_f = 0;
        % Height of front unsprung mass CoG [m]
        h_unsprung_f = 0;
        % Unsprung mass for rear axle [kg]
        m_unsprung_r = 0;
        % Height of rear unsprung mass CoG [m]
        h_unsprung_r = 0;

        % Fuel density [kg/L], default is density for gasoline
        rho_fuel = 0.7429;
        % Fuel level [L]
        V_fuel = 0;
        % Fuel tank CoG distance from front axle [m]
        l_fuel = 0;
        % Fuel tank CoG height for a ride height of zero [m]
        h_fuel = 0;

        % Which axles power is delivered through
        FWD = false;
        RWD = false;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Suspension and Wheel Geometry

        % Steering ratio (input/output)
        steer_ratio = 1;
        % Ackermann ratio
        % (0 = equal wheel angles, +1.0 = 100% --> coincident IC's, -ve for anti-ackermann)
        ackermann = 0;

        % Front brake bias (0 --> 100% rear, 1.0 --> 100% front)
        brake_bias = 0.5;

        % Suspension and geometry for individual corners
        FL_corner = Corner;
        FR_corner = Corner;
        RL_corner = Corner;
        RR_corner = Corner;

        % ARB motion ratio (Wheel/Bar)
        MR_arb_f = 1;
        MR_arb_r = 1;
        % ARB rate [N/m]
        karb_f = 0;
        karb_r = 0;

        % Roll center as a function of ride height (based on nominal wheel radius) [m]
        h_roll_f = @(RH) 0;
        h_roll_r = @(RH) 0;

        % Side view instant centre locations for front and rear suspension [rad]
        %
        % These values describe the angle to the IC location. For outboard brakes, this
        % is relative to the ground plane, for inboard brakes this is releative to
        % the wheel centre An angle of zero corresponds to no anti effects (all
        % longitudinal loads transmitted through the shocks).
        SVIC_ang_f = 0;
        SVIC_ang_r = 0;

        % Nominal wheel radius
        % (should correspond to the result of r_eff_f/r_eff_r given the pressure and
        % loading values present during ride height measurement)
        r_nominal_f = 0;
        r_nominal_r = 0;

        % Effective dynamic wheel radius as a function of pressure, P [psi], angular
        % speed, w [rad/s], and normal load, Fz [N]
        r_eff_f = @(P, w, Fz) 0;
        r_eff_r = @(P, w, Fz) 0;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Aerodynamics

        % Front view reference area [m^2]
        A_ref = 0;

        % Functions for front downforce, rear downforce, and drag force of the
        % chassis/floor (no additional aero elements) as a function of velocity
        % [m/s], front ride height [m], and pitch angle (+ve nose up) [deg].
        %
        % Can be evaluated as:
        %   F = DF_f(v, RH_f, pitch)
        DF_f = @(v, RH_f, pitch) 0;
        DF_r = @(v, RH_f, pitch) 0;
        drag = @(v, RH_f, pitch) 0;

        % Polynomial parameters for the above functions
        DF_f_params;
        DF_r_params;
        DF_drag_params;

        % Functions for front downforce, rear downforce, and drag force of
        % individual front/rear aero elements (e.g. rear wing, or front
        % splitter) as a function of velocity [m/s] and element setting.
        %
        % Can be evaluated as:
        %   F = DF_f(v, setting)
        DF_f_rear_element = @(v, x) 0;
        DF_r_rear_element = @(v, x) 0;
        drag_rear_element = @(v, x) 0;

        DF_f_front_element = @(v, x) 0;
        DF_r_front_element = @(v, x) 0;
        drag_front_element = @(v, x) 0;

        % Polynomial parameters for the above functions
        DF_f_rear_element_params;
        DF_r_rear_element_params;
        drag_rear_element_params;
        DF_f_front_element_params;
        DF_r_front_element_params;
        drag_front_element_params;

        % Front and rear wing settings (can be integers for aero packages,
        % degrees, etc)
        wing_front = 0;
        wing_rear = 0;

        % Gravitational acceleration [m/s^2]
        gravity = 9.8067;
    end

    properties (Access = private)
        % Ride height at zero damper positions [m]
        RH_0_fl = 0;
        RH_0_fr = 0;
        RH_0_rl = 0;
        RH_0_rr = 0;
    end

    properties (Constant)

    end

    methods

        % dryMass
        %
        % OUTPUTS:
        %   m: Total vehicle mass excluding fuel [kg]
        function m = dryMass(this)
            m = this.m_sprung + this.m_unsprung_f + this.m_unsprung_r;
        end

        % sprungMass
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   m: Total vehicle sprung mass (chassis and fuel) [kg]
        function m = sprungMass(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            m = this.m_sprung + this.fuelMass(V_f);
        end

        % fuelMass
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   m: Mass of fuel load [kg]
        function m = fuelMass(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            m = this.rho_fuel * V_f;
        end

        % totalMass
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   m: Total vehicle mass (sprung, unsprung, fuel) [kg]
        function m = totalMass(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            m = this.dryMass() + this.fuelMass(V_f);
        end

        % totalCogHeight
        %
        % INPUTS:
        %   RH_f: Front ride height [m]
        %   RH_f: Rear ride height [m]
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   h: CoG height of full vehicle (sprung, unsprung, fuel) [m]
        function h = totalCogHeight(this, varargin)
            RH_f = varargin{1};
            RH_r = varargin{2};

            V_f = this.V_fuel;
            if length(varargin) == 3 && isa(varargin{3}, 'double')
                V_f = varargin{3};
            end

            M_sprung = this.m_sprung * this.h_sprung;
            M_unsprung = this.m_unsprung_f * this.h_unsprung_f + this.m_unsprung_r * this.h_unsprung_r;
            M_fuel = this.fuelMass(V_f) * this.h_fuel;
            h = (M_sprung + M_unsprung + M_fuel) ./ this.totalMass(V_f);

            l_cog = this.totalCogLongPosition(V_f);
            h = h + RH_f + (RH_r - RH_f) .* l_cog / this.L;
        end

        % sprungCogHeight
        %
        % INPUTS:
        %   RH_f: Front ride height [m]
        %   RH_f: Rear ride height [m]
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   h: CoG height of sprung mass (including fuel) [m]
        function h = sprungCogHeight(this, varargin)
            RH_f = varargin{1};
            RH_r = varargin{2};

            V_f = this.V_fuel;
            if length(varargin) == 3 && isa(varargin{3}, 'double')
                V_f = varargin{1};
            end

            l_sprung_cog = this.sprungCogLongPosition(V_f);

            M_sprung = this.m_sprung * this.h_sprung;
            M_fuel = this.fuelMass(V_f) * this.h_fuel;
            h = (M_sprung + M_fuel) ./ this.sprungMass(V_f);
            h = h + RH_f + (l_sprung_cog / this.L) .* (RH_r - RH_f);
        end

        % totalCogLongPosition
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   l1: CoG distance from front axle for full vehicle (sprung, unsprung, fuel) [m]
        function l1 = totalCogLongPosition(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            M_sprung = this.m_sprung * this.l_sprung;
            M_unsprung_r = this.m_unsprung_r * this.L;
            M_fuel = this.fuelMass(V_f) * this.l_fuel;
            l1 = (M_sprung + M_unsprung_r + M_fuel) ./ this.totalMass(V_f);
        end

        % sprungCogLongPosition
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   l1: CoG distance from front axle for sprung mass (including fuel fuel) [m]
        function l1 = sprungCogLongPosition(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            M_sprung = this.m_sprung * this.l_sprung;
            M_fuel = this.fuelMass(V_f) * this.l_fuel;
            l1 = (M_sprung + M_fuel) ./ this.sprungMass(V_f);
        end

        % totalCogLatPosition
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   dy: CoG lateral position of full vehicle (sprung, unsprung, fuel) [m]
        function dy = totalCogLatPosition(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            % Unsprung mass and fuel is assumed to be symmetrical wrt vehicle centreline
            dy =  this.m_sprung * this.dy_sprung ./ this.totalMass(V_f);
        end

        % sprungCogLatPosition
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   dy: CoG lateral position of sprung mass (including fuel) [m]
        function dy = sprungCogLatPosition(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            % Fuel is assumed to be symmetrical wrt vehicle centreline
            dy =  this.m_sprung * this.dy_sprung ./ this.sprungMass(V_f);
        end

        % sprungPropertiesFromTotal
        %
        % Computes the sprung mass and sprung CoG position based on measurements of the
        % full vehicle and the unsprung properties.
        %
        % This requires the unsprung mass and fuel density properties to be set!
        %
        % INPUTS:
        %   FL: Front left corner mass [kg]
        %   FR: Front left corner mass [kg]
        %   RL: Front left corner mass [kg]
        %   RR: Front left corner mass [kg]
        %   V_f: Fuel volume [L]
        %   h_cog: CoG height of the full vehicle [m]
        %   RH_f: Front ride height [m]
        %   RH_r: Rear ride height [m]
        function this = sprungPropertiesFromTotal(this, FL, FR, RL, RR, V_f, h_cog, RH_f, RH_r)
            m_total = FL + FR + RL + RR;
            l1_total = (RL + RR) / m_total * this.L;

            this.m_sprung = m_total - this.fuelMass(V_f) - this.m_unsprung_f - this.m_unsprung_r;

            % Shift the measured CoG height to be defined for a ride height of zero
            h_cog = h_cog - RH_f + (l1_total / this.L) * (RH_r - RH_f);

            % Moment balance about axis along ground plane
            this.h_sprung = (h_cog * this.totalMass() - ...
                this.m_unsprung_f * this.h_unsprung_f - ...
                this.m_unsprung_r * this.h_unsprung_r - ...
                this.fuelMass(V_f) * this.h_fuel) / this.m_sprung;

            % Moment balance about front axle
            this.l_sprung = (m_total * l1_total - this.m_unsprung_r * this.L - ...
                this.fuelMass(V_f) * this.l_fuel) / this.m_sprung;

            % Moment balance about vehicle X axis
            this.dy_sprung = (0.5 * this.track_f * (FR - FL) + 0.5 * this.track_r * (RR - RL)) / this.m_sprung;
        end

        % setSpringPerchFromWheelLoads
        %
        % Computes the spring perch offset settings for all corners based on the measured
        % (static) wheel loads).
        %
        % INPUTS:
        %   x_fl: FL damper position [m]
        %   x_fr: FR damper position [m]
        %   x_rl: RL damper position [m]
        %   x_rr: RR damper position [m]
        %   FL: Measured front left wheel load [N]
        %   FR: Measured front right wheel load [N]
        %   RL: Measured rear left wheel load [N]
        %   RR: Measured rear right wheel load [N]
        function this = setSpringPerchFromWheelLoads(this, x_fl, x_fr, x_rl, x_rr, FL, FR, RL, RR)
            this.FL_corner = this.FL_corner.setPreloadFromMeasuredWheelForce(x_fl, FL);
            this.FR_corner = this.FR_corner.setPreloadFromMeasuredWheelForce(x_fr, FR);
            this.RL_corner = this.RL_corner.setPreloadFromMeasuredWheelForce(x_rl, RL);
            this.RR_corner = this.RR_corner.setPreloadFromMeasuredWheelForce(x_rr, RR);
        end

        % steerToWheelAngles
        %
        % Accounts for toe angles and ackermann geometry
        %
        % INPUTS:
        %   delta: Steering wheel angle (+ve turns right) [rad]
        % OUTPUTS:
        %   delta_wheel_FL: Front left wheel angle (+ve turns right) [rad]
        %   delta_wheel_FR: Front right wheel angle (+ve turns right) [rad]
        function [delta_wheel_FL, delta_wheel_FR] = steerToWheelAngles(this, delta)
            % We'll compute the wheel angles so that each wheel gets an equal
            % offset applied to it for the ackermann geometry. The ackermann ratio is:
            %             theta_i - theta_o
            %     A = ------------------------
            %         theta_i_ack - theta_o_ack
            % Where theta_i is the actual angle and theta_i_ack is the angle for 100%
            % ackermann geometry. We can express the actual angles as the sum of the
            % ackermann angle plus an offset, with the same offset applied to inner and
            % outer wheels (in opposite directions).
            if this.ackermann == 0
                % Have to handle the zero ackermann case separately. Because the
                % difference in wheel angles for non-zero ackerman changes non-linenarly
                % with steer angle, using a single offset for both wheels can introduce
                % error if the ackermann value is zero.
                delta_wheel_FL = delta / this.steer_ratio;
                delta_wheel_FR = delta / this.steer_ratio;
            else
                turn_sign = double(delta >= 0) - double(delta < 0);

                % Wheel angles for perfect ackermann steering
                delta_ack = delta / this.steer_ratio;
                delta_wheel_FL_ack = atan(this.L * tan(delta_ack) ./ (this.L + turn_sign .* 0.5 .* this.track_f .* tan(delta_ack)));
                delta_wheel_FR_ack = atan(this.L * tan(delta_ack) ./ (this.L - turn_sign .* 0.5 .* this.track_f .* tan(delta_ack)));

                % Offset to apply to each wheel
                ack_eps = -0.5 * (1 - this.ackermann) * (delta_wheel_FL_ack - delta_wheel_FR_ack);

                % Compute the actual wheel angles
                delta_wheel_FL = delta_wheel_FL_ack + turn_sign .* ack_eps;
                delta_wheel_FR = delta_wheel_FR_ack - turn_sign .* ack_eps;
            end

            % Add in toe angle
            delta_wheel_FL = delta_wheel_FL - this.FL_corner.toe;
            delta_wheel_FR = delta_wheel_FR + this.FR_corner.toe;
        end

        % cornerWheelSpeedComponents
        %
        % Compute the wheel speeds expected to be measured on each corner
        % accounting for path curvature and wheel angle.
        %
        % For input vectors of size n the output velocities will have size (2, n)
        %
        % INPUTS:
        %   v: Vehicle velocity [m/s]
        %   w: Vehicle yaw rate (+ve turning right) [rad/s]
        %   delta: Steering wheel angle (+ve turns right) [rad]
        % OUTPUTS:
        %   FL: Front left wheel velocity vector [m/s]
        %   FR: Front right wheel velocity vector [m/s]
        %   RL: Rear left wheel velocity vector [m/s]
        %   RR: Rear right wheel velocity vector [m/s]
        function [FL, FR, RL, RR] = cornerWheelSpeedComponents(this, v, w, delta)
            % Relative velocities due to rotation
            vx_f_rel = w * 0.5 * this.track_f;
            vx_r_rel = w * 0.5 * this.track_r;
            vy_rel = w * 0.5 * this.L;

            % Compute the corner velocities from the linear and angular velocity
            FL = [v + vx_f_rel, vy_rel];
            FR = [v - vx_f_rel, vy_rel];
            RL = [v + vx_r_rel, -vy_rel];
            RR = [v - vx_r_rel, -vy_rel];

            % Get the angle of each wheel
            [delta_wheel_FL, delta_wheel_FR] = this.steerToWheelAngles(delta);
            delta_wheel_RL = -this.RL_corner.toe;
            delta_wheel_RR = this.RR_corner.toe;

            % Correct the wheel velocities for the wheel angles
            c_delta = cos(-delta_wheel_FL);
            s_delta = sin(-delta_wheel_FL);
            FL = [c_delta .* FL(:, 1) - s_delta .* FL(:, 2), ...
                s_delta .* FL(:, 1) + c_delta .* FL(:, 2)];

            c_delta = cos(-delta_wheel_FR);
            s_delta = sin(-delta_wheel_FR);
            FR = [c_delta .* FR(:, 1) - s_delta .* FR(:, 2), ...
                s_delta .* FR(:, 1) + c_delta .* FR(:, 2)];

            c_delta = cos(-delta_wheel_RL);
            s_delta = sin(-delta_wheel_RL);
            RL = [c_delta .* RL(:, 1) - s_delta .* RL(:, 2), ...
                s_delta .* RL(:, 1) + c_delta .* RL(:, 2)];

            c_delta = cos(-delta_wheel_RR);
            s_delta = sin(-delta_wheel_RR);
            RR = [c_delta .* RR(:, 1) - s_delta .* RR(:, 2), ...
                s_delta .* RR(:, 1) + c_delta .* RR(:, 2)];
        end

        % rotateImuToGroundPlane
        %
        % For transforming IMU measurements (linear acceleration, angular rates) to be
        % aligned with the ground plane by accounting for the suspension and tire
        % deflection.
        %
        % INPUTS
        %   v: Vector (X, Y, Z) as measured on sprung mass [m/s^2] (size 3xn)
        %   RH_fl: Front left ride height (including any tire deformation) [m]
        %   RH_fr: Front right ride height (including any tire deformation) [m]
        %   RH_rl: Rear left ride height (including any tire deformation) [m]
        %   RH_rr: Rear right ride height (including any tire deformation) [m]
        % OUPUTS:
        %   v_rot: vector rotated to align with the ground plane (size 3xn)
        function v_rot = rotateImuToGroundPlane(this, v, RH_fl, RH_fr, RH_rl, RH_rr)
            pitch = this.pitchFromAvgRideHeights((RH_fl + RH_fr) / 2, (RH_rl + RH_rr) / 2);
            roll = (this.rollFromFrontRideHeights(RH_fl, RH_fr) + this.rollFromRearRideHeights(RH_rl, RH_rr)) / 2;

            % Precompute sines and cosines of roll and pitch angles
            s_r = sin(roll);
            c_r = cos(roll);
            s_p = sin(pitch);
            c_p = cos(pitch);

            % Rotated vector will be result of:
            %   v_rot = (Ry(pitch) * Rx(roll)' * v
            % Since v can have many rows we'll compute the result one coordinate at a time
            v_rot = zeros(size(v));
            v_rot(1, :) = c_p .* v(1, :) + -s_p .* v(3, :);
            v_rot(2, :) = s_r .* s_p .* v(1, :) + c_r .* v(2, :) + s_r .* c_p .* v(3, :);
            v_rot(3, :) = c_r .* s_p .* v(1, :) + - s_r .* v(2, :) + c_r .* c_p .* v(3, :);
        end

        % effectiveFrontTireRadius
        %
        % Radius of the front tire under load and motion
        %
        % INPUTS:
        %   P: Pressure [psi]
        %   w: Angular speed [rad/s]
        %   Fz: Normal load [N]
        % OUTPUTS:
        %   r: Effective radius [m]
        function r = effectiveFrontTireRadius(this, P, w, Fz)
            r = this.r_eff_f(P, w, Fz);
        end

        % frontTireRadiusDelta
        %
        % Change in front tire radius relative to nominal radius.
        %
        % INPUTS:
        %   P: Pressure [psi]
        %   w: Angular speed [rad/s]
        %   Fz: Normal load [N]
        % OUTPUTS:
        %   dr: Change in tire radius [m]
        function r = frontTireRadiusDelta(this, P, w, Fz)
            r = this.effectiveFrontTireRadius(P, w, Fz) - this.r_nominal_f;
        end

        % effectiveRearTireRadius
        %
        % Radius of the rear tire under load and motion
        %
        % INPUTS:
        %   P: Pressure [psi]
        %   w: Angular speed [rad/s]
        %   Fz: Normal load [N]
        % OUTPUTS:
        %   r: Effective radius [m]
        function r = effectiveRearTireRadius(this, P, w, Fz)
            r = this.r_eff_r(P, w, Fz);
        end

        % rearTireRadiusDelta
        %
        % Change in rear tire radius relative to nominal radius.
        %
        % INPUTS:
        %   P: Pressure [psi]
        %   w: Angular speed [rad/s]
        %   Fz: Normal load [N]
        % OUTPUTS:
        %   dr: Change in tire radius [m]
        function r = rearTireRadiusDelta(this, P, w, Fz)
            r = this.effectiveRearTireRadius(P, w, Fz) - this.r_nominal_r;
        end

        % setReferenceRideHeight
        %
        % Provides a baseline measurement for ride height given damper positions, from
        % which the ride height can be computed for any damper position.
        %
        % INPUTS:
        %   RH_f: Front ride height [m]
        %   RH_r: Rear ride height [m]
        %   x_fl: FL damper position [m]
        %   x_fr: FR damper position [m]
        %   x_rl: RL damper position [m]
        %   x_rr: RR damper position [m]
        function this = setReferenceRideHeight(this, RH_f, RH_r, x_fl, x_fr, x_rl, x_rr)
            this.RH_0_fl = RH_f + this.FL_corner.MR_spring * this.FL_corner.correctedDamperPos(x_fl);
            this.RH_0_fr = RH_f + this.FR_corner.MR_spring * this.FR_corner.correctedDamperPos(x_fr);
            this.RH_0_rl = RH_r + this.RL_corner.MR_spring * this.RL_corner.correctedDamperPos(x_rl);
            this.RH_0_rr = RH_r + this.RR_corner.MR_spring * this.RR_corner.correctedDamperPos(x_rr);
        end

        % rideHeightFromDamperPos
        %
        % INPUTS:
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        % OUTPUTS:
        %   RH_fl: Front left ride height [m]
        %   RH_fr: Front right ride height [m]
        %   RH_rl: Rear left ride height [m]
        %   RH_rr: Rear right ride height [m]
        function [RH_fl, RH_fr, RH_rl, RH_rr] = rideHeightFromDamperPos(this, x_fl, x_fr, x_rl, x_rr)
            RH_fl = this.RH_0_fl - this.FL_corner.correctedDamperPos(x_fl) * this.FL_corner.MR_spring;
            RH_fr = this.RH_0_fr - this.FR_corner.correctedDamperPos(x_fr) * this.FR_corner.MR_spring;
            RH_rl = this.RH_0_rl - this.RL_corner.correctedDamperPos(x_rl) * this.RL_corner.MR_spring;
            RH_rr = this.RH_0_rr - this.RR_corner.correctedDamperPos(x_rr) * this.RR_corner.MR_spring;
        end

        % avgRideHeightFromDamperPos
        %
        % INPUTS:
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        % OUTPUTS:
        %   RH_f: Average front ride height [m]
        %   RH_r: Average rear ride height [m]
        function [RH_f, RH_r] = avgRideHeightFromDamperPos(this, x_fl, x_fr, x_rl, x_rr)
            [RH_fl, RH_fr, RH_rl, RH_rr] = rideHeightFromDamperPos(this, x_fl, x_fr, x_rl, x_rr);
            RH_f = (RH_fl + RH_fr) ./ 2;
            RH_r = (RH_rl + RH_rr) ./ 2;
        end

        % damperPosFromRideHeight
        %
        % INPUTS:
        %   RH_fl: Front left ride height [m]
        %   RH_fr: Front right ride height [m]
        %   RH_rl: Rear left ride height [m]
        %   RH_rr: Rear right ride height [m]
        % OUTPUTS:
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        function [x_fl, x_fr, x_rl, x_rr] = damperPosFromRideHeight(this, RH_fl, RH_fr, RH_rl, RH_rr)
            x_fl = (this.RH_0_fl - RH_fl) ./ this.FL_corner.MR_spring + this.FL_corner.x0;
            x_fr = (this.RH_0_fr - RH_fr) ./ this.FR_corner.MR_spring + this.FR_corner.x0;
            x_rl = (this.RH_0_rl - RH_rl) ./ this.RL_corner.MR_spring + this.RL_corner.x0;
            x_rr = (this.RH_0_rr - RH_rr) ./ this.RR_corner.MR_spring + this.RR_corner.x0;
        end

        % damperPosFromAvgRideHeight
        %
        % INPUTS:
        %   RH_f: Front ride height [m]
        %   RH_r: Rear ride height [m]
        % OUTPUTS:
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position mm]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        function [x_fl, x_fr, x_rl, x_rr] = damperPosFromAvgRideHeight(this, RH_f, RH_r)
            [x_fl, x_fr, x_rl, x_rr] = this.damperPosFromRideHeight(RH_f, RH_f, RH_r, RH_r);
        end

        % pitchFromAvgRideHeights
        %
        % INPUTS:
        %   RH_f: Front ride height [m]
        %   RH_r: Rear ride height [m]
        % OUTPUTS:
        %   pitch: Pitch angle (+ve front up) [rad]
        function pitch = pitchFromAvgRideHeights(this, RH_f, RH_r)
            pitch = atan2(RH_f - RH_r, this.L);
        end

        % rollFromFrontRideHeights
        %
        % INPUTS:
        %   RH_fl: Front left ride height [m]
        %   RH_fr: Front right ride height [m]
        % OUTPUTS:
        %   roll: Roll angle (+ve right side down) [rad]
        function roll = rollFromFrontRideHeights(this, RH_fl, RH_fr)
            roll = atan2(RH_fl - RH_fr, this.track_f);
        end

        % rollFromRearRideHeights
        %
        % INPUTS:
        %   RH_rl: Rear left ride height [m]
        %   RH_rr: Rear right ride height [m]
        % OUTPUTS:
        %   roll: Roll angle (+ve right side down) [rad]
        function roll = rollFromRearRideHeights(this, RH_rl, RH_rr)
            roll = atan2(RH_rl - RH_rr, this.track_r);
        end

        % frontArbToWheelMotion
        %
        % INPUTS:
        %   x_arb: Motion at front ARB [m]
        % OUTPUTS:
        %   x_w: Equivalent motion at wheel [m]
        function x_w = frontArbToWheelMotion(this, x_arb)
            x_w = this.MR_arb_f * x_arb;
        end

        % frontWheelToArbMotion
        %
        % INPUTS:
        %   x_w: Motion at front wheel [m]
        % OUTPUTS:
        %   x_arb: Equivalent motion at ARB [m]
        function x_arb = frontWheelToArbMotion(this, x_w)
            x_arb = x_w / this.MR_arb_f;
        end

        % frontArbToWheelForce
        %
        % INPUTS:
        %   F_arb: Force at front ARB [N]
        % OUTPUTS:
        %   F_w: Equivalent force at wheel [N]
        function F_w = frontArbToWheelForce(this, F_arb)
            F_w = this.MR_arb_f^2 * F_arb;
        end

        % frontWheelToArbForce
        %
        % INPUTS:
        %   F_w: Force at front wheel [N]
        % OUTPUTS:
        %   F_arb: Equivalent force at ARB [N]
        function F_arb = frontWheelToArbForce(this, F_w)
            F_arb = F_w / this.MR_arb_f^2;
        end

        % rearArbToWheelMotion
        %
        % INPUTS:
        %   x_arb: Motion at rear ARB [m]
        % OUTPUTS:
        %   x_w: Equivalent motion at wheel [m]
        function x_w = rearArbToWheelMotion(this, x_arb)
            x_w = this.MR_arb_r * x_arb;
        end

        % rearWheelToArbMotion
        %
        % INPUTS:
        %   x_w: Motion at rear wheel [m]
        % OUTPUTS:
        %   x_arb: Equivalent motion at ARB [m]
        function x_arb = rearWheelToArbMotion(this, x_w)
            x_arb = x_w / this.MR_arb_r;
        end

        % rearArbToWheelForce
        %
        % INPUTS:
        %   F_arb: Force at rear ARB [N]
        % OUTPUTS:
        %   F_w: Equivalent force at wheel [N]
        function F_w = rearArbToWheelForce(this, F_arb)
            F_w = this.MR_arb_r^2 * F_arb;
        end

        % rearWheelToArbForce
        %
        % INPUTS:
        %   F_w: Force at rear wheel [N]
        % OUTPUTS:
        %   F_arb: Equivalent force at ARB [N]
        function F_arb = rearWheelToArbForce(this, F_w)
            F_arb = F_w / this.MR_arb_r^2;
        end

        % rollStiffnessDistribution
        %
        % OUTPUTS:
        %   q: Fraction of total roll stiffness across front axle [0, 1]
        function q = rollStiffnessDistribution(this)
            k_roll_f = this.MR_arb_r^2 * this.karb_f + ...
                0.5 * this.FL_corner.MR_spring^2 * this.FL_corner.kspring + ...
                0.5 * this.FR_corner.MR_spring^2 * this.FR_corner.kspring;

            k_roll_r = this.MR_arb_r^2 * this.karb_r + ...
                0.5 * this.RL_corner.MR_spring^2 * this.RL_corner.kspring + ...
                0.5 * this.RR_corner.MR_spring^2 * this.RR_corner.kspring;

            q = k_roll_f / (k_roll_f + k_roll_r);
        end

        % arbForces
        %
        % INPUTS:
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        % OUTPUTS:
        %   FL: Force at front left ARB (+ve downwards) [N]
        %   FR: Force at front right ARB (+ve downwards) [N]
        %   RL: Force at rear left ARB (+ve downwards) [N]
        %   RR: Force at rear right ARB (+ve downwards) [N]
        function [FL, FR, RL, RR] = arbForces(this, x_fl, x_fr, x_rl, x_rr)
            % We need to consider two offsets when evaluating damper positions:
            %   - Sensor offset: A bias in the sensor reading, this is the x0 parameter
            %     in each Corner object
            %   - Geometry offset: A bias in the vehicle geometry that results in
            %     identical damper positions corresponding to different ride heights,
            %     this is captured with the RH_0_xx member variable within this class
            %
            % We'll also assume that when ride heights are equal there is zero torsion in
            % the ARB.

            % First convert damper positions to ride heights, this will account for both
            % offsets
            [RH_fl, RH_fr, RH_rl, RH_rr] = this.rideHeightFromDamperPos(x_fl, x_fr, x_rl, x_rr);

            % Convert wheel position delta to bar position deltas then compute force
            arb_F_f = this.karb_f * this.frontWheelToArbMotion((RH_fl - RH_fr));
            FL = -0.5 * arb_F_f;
            FR = 0.5 * arb_F_f;

            arb_F_r = this.karb_r * this.rearWheelToArbMotion((RH_rl - RH_rr));
            RL = -0.5 * arb_F_r;
            RR = 0.5 * arb_F_r;
        end

        % arbWheelForces
        %
        % INPUTS:
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        % OUTPUTS:
        %   FL: Force on front left wheel (+ve downwards) [N]
        %   FR: Force on front right wheel (+ve downwards) [N]
        %   RL: Force on rear left wheel (+ve downwards) [N]
        %   RR: Force on rear right wheel (+ve downwards) [N]
        function [FL, FR, RL, RR] = arbWheelForces(this, x_fl, x_fr, x_rl, x_rr)
            [FL, FR, RL, RR] = this.arbForces(x_fl, x_fr, x_rl, x_rr);
            FL = this.frontArbToWheelForce(FL);
            FR = this.frontArbToWheelForce(FR);
            RL = this.rearArbToWheelForce(RL);
            RR = this.rearArbToWheelForce(RR);
        end

        % staticWheelLoads
        %
        % Computes the static wheel loads based on total mass and CoG position
        %
        % INPUTS:
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   FL: Front left wheel load [N]
        %   FR: Front right wheel load [N]
        %   RL: Rear left wheel load [N]
        %   RR: Rear right wheel load [N]
        function [FL, FR, RL, RR] = staticWheelLoads(this, varargin)
            V_f = this.V_fuel;
            if length(varargin) == 1 && isa(varargin{1}, 'double')
                V_f = varargin{1};
            end

            l_cog = this.totalCogLongPosition(V_f);
            dy_cog = this.totalCogLatPosition(V_f);
            m_total = this.totalMass(V_f);
            [FL, FR, RL, RR] = this.staticWheelLoadsFromCoG(m_total, l_cog, dy_cog);
        end

        % unsprungCentrifugalWheelLoads
        %
        % Effect of centrifugal force from dips or crests on unsprung weight transfer.
        %
        % INPUTS
        %   v: Linear velocity [m/s]
        %   w_sprung: Pitch rate of sprung mass (+ve pitch up) [rad/s]
        %   v_damp_fl: Front left damper velocity [m/s]
        %   v_damp_fr: Front right damper velocity [m/s]
        %   v_damp_rl: Rear left damper velocity [m/s]
        %   v_damp_rr: Rear right damper velocity [m/s]
        % OUTPUTS:
        %   FL: Front left wheel load delta [N]
        %   FR: Front right wheel load delta [N]
        %   RL: Rear left wheel load delta [N]
        %   RR: Rear right wheel load delta [N]
        function [FL, FR, RL, RR] = unsprungCentrifugalWheelLoads(this, v, w_sprung, v_damp_fl, v_damp_fr, v_damp_rl, v_damp_rr)
            % Compute the pitch rate of the vehicle from the sprung pitch rate and
            % suspension pitch rate
            v_susp_f = (this.FL_corner.springToWheelMotion(v_damp_fl) + this.FR_corner.springToWheelMotion(v_damp_fr)) / 2;
            v_susp_r = (this.RL_corner.springToWheelMotion(v_damp_rl) + this.RR_corner.springToWheelMotion(v_damp_rr)) / 2;
            w_susp = (v_susp_r - v_susp_f) / this.L;
            w_eff = w_sprung + w_susp;

            FL = this.m_unsprung_f * v .* w_eff / 2;
            FR = FL;
            RL = this.m_unsprung_r * v .* w_eff / 2;
            RR = RL;
        end

        % latWeightTransferSuspension
        %
        % Compute the amount of weight transfer due to the sprung mass transmitted
        % through the shocks.
        %
        % INPUTS:
        %   a: Lateral acceleration [m/s^2]
        %   RH_f: Front ride height (not including tire deformation) [m]
        %   RH_r: Rear ride height (not including tire deformation) [m]
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   FL: Front left wheel load delta [N]
        %   FR: Front right wheel load delta [N]
        %   RL: Rear left wheel load delta [N]
        %   RR: Rear right wheel load delta [N]
        function [FL, FR, RL, RR] = latWeightTransferSuspension(this, varargin)
            a = varargin{1};
            RH_r = varargin{2};
            RH_f = varargin{3};

            V_f = this.V_fuel;
            if length(varargin) == 4 && isa(varargin{4}, 'double')
                V_f = varargin{4};
            end

            % Sprung mass CoG position
            l_sprung_cog = this.sprungCogLongPosition(V_f);
            h_sprung_cog = this.sprungCogHeight(RH_f, RH_r, V_f);

            % Distance between the sprung CoG and the roll axis
            h_roll = h_sprung_cog - this.h_roll_f(RH_f) - ...
                (l_sprung_cog / this.L) .* (this.h_roll_r(RH_r) - this.h_roll_f(RH_f));

            % Compute the weight transfer across each axle
            q = this.rollStiffnessDistribution();
            dW_f = this.sprungMass(V_f) .* a .* h_roll .* q / this.track_f;
            dW_r = this.sprungMass(V_f) .* a .* h_roll .* (1 - q) / this.track_r;

            FL = 0.5 * dW_f;
            FR = -0.5 * dW_f;
            RL = 0.5 * dW_r;
            RR = -0.5 * dW_r;
        end

        % longWeightTransferSuspension
        %
        % Computes the amount of longitudinal weight transfer to the wheels due to the
        % sprung mass forces transmitted through the shocks.
        %
        % INPUTS:
        %   a: Longitudinal acceleration [m/s^2]
        %   RH_f: Front ride height (including tire deformation) [m]
        %   RH_r: Rear ride height (including tire deformation) [m]
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   FL: Front left wheel load delta [N]
        %   FR: Front right wheel load delta [N]
        %   RL: Rear left wheel load delta [N]
        %   RR: Rear right wheel load delta [N]
        function [FL, FR, RL, RR] = longWeightTransferSuspension(this, varargin)
            a = varargin{1};
            RH_r = varargin{2};
            RH_f = varargin{3};

            V_f = this.V_fuel;
            if length(varargin) == 4 && isa(varargin{4}, 'double')
                V_f = varargin{4};
            end

            % Compute total longitudinal load transfer and the anti rates to get the
            % amount of sprung weight transfer
            h_cog = this.totalCogHeight(RH_f, RH_r, V_f);
            dW_total = this.sprungMass(V_f) .* a .* h_cog / this.L;

            % anti_f_perc = this.brake_bias * tan(this.SVIC_ang_f) * h_cog / this.L;
            % anti_r_perc = (1 - this.brake_bias) * tan(this.SVIC_ang_r) * h_cog / this.L;

            anti_f_perc = tan(this.SVIC_ang_f) * h_cog / this.L;
            anti_r_perc = tan(this.SVIC_ang_r) * h_cog / this.L;

            anti_f_perc(a < 0) = this.brake_bias * anti_f_perc(a < 0);
            anti_r_perc(a < 0) = (1 - this.brake_bias) * anti_r_perc(a < 0);

            if ~this.FWD
                anti_f_perc(a > 0) = 0;
                anti_r_perc = 2 * anti_r_perc;
            end
            if ~this.RWD
                anti_r_perc(a > 0) = 0;
                anti_f_perc = 2 * anti_f_perc;
            end

            % FL = -0.5 * dW_total .* (1 - anti_f_perc);
            % FR = -0.5 * dW_total .* (1 - anti_f_perc);
            % RL = 0.5 * dW_total .* (1 - anti_r_perc);
            % RR = 0.5 * dW_total .* (1 - anti_r_perc);
            FL = -0.5 * dW_total .* anti_f_perc;
            FR = -0.5 * dW_total .* anti_f_perc;
            RL = 0.5 * dW_total .* anti_r_perc;
            RR = 0.5 * dW_total .* anti_r_perc;
        end

        % latWeightTransferGeometric
        %
        % Computes the amount of lateral weight transfer to the wheels due to the
        % sprung mass forces transmitted through the suspension geometry.
        %
        % INPUTS:
        %   a: Lateral acceleration [m/s^2]
        %   RH_f: Front ride height (not including tire deformation) [m]
        %   RH_r: Rear ride height (not including tire deformation) [m]
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   FL: Front left wheel load delta [N]
        %   FR: Front right wheel load delta [N]
        %   RL: Rear left wheel load delta [N]
        %   RR: Rear right wheel load delta [N]
        function [FL, FR, RL, RR] = latWeightTransferGeometric(this, varargin)
            a = varargin{1};
            RH_r = varargin{2};
            RH_f = varargin{3};

            V_f = this.V_fuel;
            if length(varargin) == 4 && isa(varargin{4}, 'double')
                V_f = varargin{4};
            end

            l_sprung_eff = this.sprungCogLongPosition(V_f);
            m_sprung_f = this.sprungMass(V_f) .* (1 - l_sprung_eff / this.L);
            m_sprung_r = this.sprungMass(V_f) .* (l_sprung_eff / this.L);

            % Compute the weight transfer on each axle via moment balance about the roll
            % axis
            dW_f = m_sprung_f .* a .* this.h_roll_f(RH_f) / this.track_f;
            dW_r = m_sprung_r .* a .* this.h_roll_r(RH_r) / this.track_r;

            FL = 0.5 * dW_f;
            FR = -0.5 * dW_f;
            RL = 0.5 * dW_r;
            RR = -0.5 * dW_r;
        end

        % longWeightTransferGeometric
        %
        % Computes the amount of longitudinal weight transfer to the wheels due to the
        % sprung mass forces transmitted through the suspension geometry.
        %
        % INPUTS:
        %   a: Longitudinal acceleration [m/s^2]
        %   RH_f: Front ride height (including tire deformation) [m]
        %   RH_r: Rear ride height (including tire deformation) [m]
        %   V_f: OPTIONAL fuel volume [L], when not present member variable V_fuel is used
        % OUTPUTS:
        %   FL: Front left wheel load delta [N]
        %   FR: Front right wheel load delta [N]
        %   RL: Rear left wheel load delta [N]
        %   RR: Rear right wheel load delta [N]
        function [FL, FR, RL, RR] = longWeightTransferGeometric(this, varargin)
            a = varargin{1};
            RH_r = varargin{2};
            RH_f = varargin{3};

            V_f = this.V_fuel;
            if length(varargin) == 4 && isa(varargin{4}, 'double')
                V_f = varargin{4};
            end

            % Compute total longitudinal load transfer and the anti rates to get the
            % amount of geometric transfer
            h_cog = this.totalCogHeight(RH_f, RH_r, V_f);
            dW_total = this.sprungMass(V_f) .* a .* h_cog / this.L;
            anti_f_perc = this.brake_bias * tan(this.SVIC_ang_f) * h_cog / this.L;
            anti_r_perc = (1 - this.brake_bias) * tan(this.SVIC_ang_r) * h_cog / this.L;

            FL = -0.25 * dW_total .* anti_f_perc;
            FR = -0.25 * dW_total .* anti_f_perc;
            RL = 0.25 * dW_total .* anti_r_perc;
            RR = 0.25 * dW_total .* anti_r_perc;
        end

        % latWeightTransferUnsprung
        %
        % Computes the amount of weight transfer due to the unsprung mass. This does
        % not account for changes in unsprung CoG height due to tire compression.
        %
        % INPUTS:
        %   a: Lateral acceleration [m/s^2]
        % OUTPUTS:
        %   FL: Front left wheel load delta [N]
        %   FR: Front right wheel load delta [N]
        %   RL: Rear left wheel load delta [N]
        %   RR: Rear right wheel load delta [N]
        function [FL, FR, RL, RR] = latWeightTransferUnsprung(this, a)
            dW_f = a * this.m_unsprung_f * this.h_unsprung_f / this.track_f;
            dW_r = a * this.m_unsprung_r * this.h_unsprung_r / this.track_r;

            FL = 0.5 * dW_f;
            FR = -0.5 * dW_f;
            RL = 0.5 * dW_r;
            RR = -0.5 * dW_r;
        end

        % totalWheelLoads
        %
        % INPUTS:
        %   v: Vehicle velocity [m/s]
        %   a: Acceleration (X, Y, Z) as measured on sprung mass (gravity included) [m/s^2]
        %   w: Pitch rate as measured on sprung mass [rad/s]
        %   V_f: Fuel level [L]
        %   RH_fl: Front left ride height (including any tire deformation) [m]
        %   RH_fr: Front right ride height (including any tire deformation) [m]
        %   RH_rl: Rear left ride height (including any tire deformation) [m]
        %   RH_rr: Rear right ride height (including any tire deformation) [m]
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        %   v_fl: Front left damper velocity [m/s]
        %   v_fr: Front right damper velocity [m/s]
        %   v_rl: Rear left damper velocity [m/s]
        %   v_rr: Rear right damper velocity [m/s]
        % OUTPUTS:
        %   FL: Front left wheel load [N]
        %   FR: Front right wheel load [N]
        %   RL: Rear left wheel load [N]
        %   RR: Rear right wheel load [N]
        function [FL, FR, RL, RR] = totalWheelLoads(this, v, a, w, V_f, ...
            RH_fl, RH_fr, RH_rl, RH_rr, ...
            x_fl, x_fr, x_rl, x_rr, ...
            v_fl, v_fr, v_rl, v_rr)

            % Total wheel loads is the sum of shock (spring and damper), ARB forces,
            % geometric weight transfer, unsprung weight transfer, and aerodynamic forces
            %
            % F = F_shock + F_arb + F_us_wt + F_geo_wt + F_aero

            % Rotate acceleration and pitch rate to align with ground frame
            a = this.rotateImuToGroundPlane(a, RH_fl, RH_fr, RH_rl, RH_rr);

            w_vec = zeros(3, length(w));
            w_vec(2, :) = w;
            w_vec = this.rotateImuToGroundPlane(w_vec, RH_fl, RH_fr, RH_rl, RH_rr);
            w = w_vec(2, :);

            % Spring, damper, and bumpstop forces
            FL.spring = this.FL_corner.springForceAtWheel(x_fl) + this.FL_corner.bumpstopForceAtWheel(x_fl);
            FL.damper = this.FL_corner.damperForceAtWheel(v_fl);

            FR.spring = this.FR_corner.springForceAtWheel(x_fr) + this.FR_corner.bumpstopForceAtWheel(x_fr);
            FR.damper = this.FR_corner.damperForceAtWheel(v_fr);

            RL.spring = this.RL_corner.springForceAtWheel(x_rl) + this.RL_corner.bumpstopForceAtWheel(x_rl);
            RL.damper = this.RL_corner.damperForceAtWheel(v_rl);

            RR.spring = this.RR_corner.springForceAtWheel(x_rr) + this.RR_corner.bumpstopForceAtWheel(x_rr);
            RR.damper = this.RR_corner.damperForceAtWheel(v_rr);

            % ARB forces
            [FL.arb, FR.arb, RL.arb, RR.arb] = this.arbWheelForces(x_fl, x_fr, x_rl, x_rr);

            % Unsprung lateral weight transfer and centrifugal effects
            [FL_us_wt, FR_us_wt, RL_us_wt, RR_us_wt] = this.latWeightTransferUnsprung(a(2, :));
            [FL_us_cent, FR_us_cent, RL_us_cent, RR_us_cent] = ...
                this.unsprungCentrifugalWheelLoads(v, w, v_fl, v_fr, v_rl, v_rr);

            FL.unsprung = (this.m_unsprung_f * this.gravity / 2) + FL_us_wt + FL_us_cent;
            FR.unsprung = (this.m_unsprung_f * this.gravity / 2) + FR_us_wt + FR_us_cent;
            RL.unsprung = (this.m_unsprung_r * this.gravity / 2) + RL_us_wt + RL_us_cent;
            RR.unsprung = (this.m_unsprung_r * this.gravity / 2) + RR_us_wt + RR_us_cent;

            % Geometric weight transfer
            %
            % Need to use ride heights only considering suspension movement, because
            % roll centres are defined for constant tire radius.
            [RH_sus_f, RH_sus_r] = this.avgRideHeightFromDamperPos(x_fl, x_fr, x_rl, x_rr);
            [FL_geo_lat, FR_geo_lat, RL_geo_lat, RR_geo_lat] = ...
                this.latWeightTransferGeometric(a(2, :), RH_sus_f, RH_sus_r, V_f);

            % For longitudinal geometric weight transfer we only want the consider the
            % amount of acceleration caused by wheel forces and ignore the effect of
            % aero drag. So compute the aero drag force, then use that to adjust the
            % longitudinal acceleration.
            RH_f = (RH_fl + RH_fr) / 2;
            RH_r = (RH_rl + RH_rr) / 2;

            F_drag = this.drag(v, RH_f, this.pitchFromAvgRideHeights(RH_f, RH_r));
            ax_wheels = a(1, :) + F_drag .* this.totalMass(V_f);
            [FL_geo_long, FR_geo_long, RL_geo_long, RR_geo_long] = ...
                this.longWeightTransferGeometric(ax_wheels, RH_f, RH_r, V_f);

            FL.geometric = FL_geo_lat + 0 * FL_geo_long;
            FR.geometric = FR_geo_lat + 0 * FR_geo_long;
            RL.geometric = RL_geo_lat + 0 * RL_geo_long;
            RR.geometric = RR_geo_lat + 0 * RR_geo_long;

            % Aerodynamic forces
            F_aero_f = this.DF_f(v, RH_f, this.pitchFromAvgRideHeights(RH_f, RH_r));
            F_aero_r = this.DF_r(v, RH_f, this.pitchFromAvgRideHeights(RH_f, RH_r));
            FL.aero = 0.5 * F_aero_f;
            FR.aero = 0.5 * F_aero_f;
            RL.aero = 0.5 * F_aero_r;
            RR.aero = 0.5 * F_aero_r;

            % Sum everything up
            FL.total = FL.spring + FL.damper + FL.arb + FL.unsprung + FL.geometric + FL.aero;
            FR.total = FR.spring + FR.damper + FR.arb + FR.unsprung + FR.geometric + FR.aero;
            RL.total = RL.spring + RL.damper + RL.arb + RL.unsprung + RL.geometric + RL.aero;
            RR.total = RR.spring + RR.damper + RR.arb + RR.unsprung + RR.geometric + RR.aero;
        end

        % estimatedAeroWheelLoad
        %
        % INPUTS:
        %   a: Acceleration (X, Y, Z) as measured on sprung mass (gravity included) [m/s^2]
        %   V_f: Fuel level [L]
        %   RH_fl: Front left ride height (including any tire deformation) [m]
        %   RH_fr: Front right ride height (including any tire deformation) [m]
        %   RH_rl: Rear left ride height (including any tire deformation) [m]
        %   RH_rr: Rear right ride height (including any tire deformation) [m]
        %   x_fl: Front left damper position [m]
        %   x_fr: Front right damper position [m]
        %   x_rl: Rear left damper position [m]
        %   x_rr: Rear right damper position [m]
        %   v_fl: Front left damper velocity [m/s]
        %   v_fr: Front right damper velocity [m/s]
        %   v_rl: Rear left damper velocity [m/s]
        %   v_rr: Rear right damper velocity [m/s]
        % OUTPUTS:
        %   FL: Front left wheel load [N]
        %   FR: Front right wheel load [N]
        %   RL: Rear left wheel load [N]
        %   RR: Rear right wheel load [N]
        function [FL, FR, RL, RR] = estimatedAeroWheelLoad(this, a, V_f, ...
            RH_fl, RH_fr, RH_rl, RH_rr, ...
            x_fl, x_fr, x_rl, x_rr, ...
            v_fl, v_fr, v_rl, v_rr)

            % Force balance in the vertical direction when considering forces transmitted
            % through the suspension
            %   F_shocks = F_static_sprung + F_anti + F_aero
            %
            % We're only interested in the front-rear distribution so the ARB forces and
            % lateral geometric weight transfer can be ignored since they ear equal and
            % opposite between left and right sides.

            % Rotate acceleration to align with ground frame
            a = this.rotateImuToGroundPlane(a, RH_fl, RH_fr, RH_rl, RH_rr);

            % Spring, damper, and bumpstop forces
            FL.spring = this.FL_corner.springForceAtWheel(x_fl) + this.FL_corner.bumpstopForceAtWheel(x_fl);
            FL.damper = this.FL_corner.damperForceAtWheel(v_fl);

            FR.spring = this.FR_corner.springForceAtWheel(x_fr) + this.FR_corner.bumpstopForceAtWheel(x_fr);
            FR.damper = this.FR_corner.damperForceAtWheel(v_fr);

            RL.spring = this.RL_corner.springForceAtWheel(x_rl) + this.RL_corner.bumpstopForceAtWheel(x_rl);
            RL.damper = this.RL_corner.damperForceAtWheel(v_rl);

            RR.spring = this.RR_corner.springForceAtWheel(x_rr) + this.RR_corner.bumpstopForceAtWheel(x_rr);
            RR.damper = this.RR_corner.damperForceAtWheel(v_rr);

            % Static forces for sprung mass
            % This assumes zero acceleration, so only gravity, but we me be going over
            % a crest or through a bump, so scale this by the actual measured acceleration
            % in the vertical direction
            l_sprung_eff = this.sprungCogLongPosition(V_f);
            dy_sprung_eff = this.sprungCogLatPosition(V_f);

            [FL_static, FR_static, RL_static, RR_static] = ...
                this.staticWheelLoadsFromCoG(this.sprungMass(V_f), l_sprung_eff, dy_sprung_eff);
            FL.static = FL_static .* -a(3, :) / this.gravity;
            FR.static = FR_static .* -a(3, :) / this.gravity;
            RL.static = RL_static .* -a(3, :) / this.gravity;
            RR.static = RR_static .* -a(3, :) / this.gravity;

            % Weight transfer through suspension
            RH_f =  (RH_fl + RH_fr) / 2;
            RH_r =  (RH_rl + RH_rr) / 2;
            [FL_anti, FR_anti, RL_anti, RR_anti] = longWeightTransferSuspension(this, a(1, :), RH_f, RH_r, V_f);
            FL.anti = FL_anti;
            FR.anti = FR_anti;
            RL.anti = RL_anti;
            RR.anti = RR_anti;

            % Sum everything up
            FL.aero = FL.spring + 0*FL.damper - FL.static - 0*FL.anti;
            FR.aero = FR.spring + 0*FR.damper - FR.static - 0*FR.anti;
            RL.aero = RL.spring + 0*RL.damper - RL.static - 0*RL.anti;
            RR.aero = RR.spring + 0*RR.damper - RR.static - 0*RR.anti;
        end

        %
        % % rideHeightAtVelocity
        % %
        % % Esimates the steady state ride heights, rake angle, and damper
        % % positions at a specific velocity based on the current spring
        % % rates and aerodynamic downforce functions.
        % %
        % % INPUTS:
        % %   v: Velocity [m/s]
        % % OUTPUTS:
        % %   RH_f: Front ride height [mm]
        % %   RH_r: Rear ride height [mm]
        % %   pitch: Pitch angle (+ve nose up) [deg]
        % %   xf: Front damper position [mm]
        % %   xr: Rear damper position [mm]
        % function [RH_f, RH_r, pitch, xf, xr] = rideHeightAtVelocity(this, v)
        %     func = @(x) this.rideHeightAtVelocityOptFunction(v, x(1), x(2));
        %
        %     [xf, xr] = this.damperPosFromRideHeight(this.RH_f_static, this.RH_r_static);
        %     x0 = [xf, xr];
        %     x = fsolve(func, x0);
        %
        %     xf = x(1);
        %     xr = x(2);
        %     [RH_f, RH_r] = this.rideHeightFromDamperPos(xf, xr);
        %     pitch = this.pitchFromAvgRideHeights(RH_f, RH_r);
        % end

        % aeroForceCoefficient
        %
        % INPUTS:
        %   v: Measured velocity [m/s]
        %   F: Measured force (drag or downforce) [N]
        %   rho: Air density [kg/m^3]
        % OUTPUTS:
        %   C: Coefficient of force (per unit area)
        function C = aeroForceCoefficient(this, v, F, rho)
            C = F ./ (0.5 * rho .* this.A_ref .* v.^2);
        end

    end

    methods (Access = protected)

        % staticWheelLoadsFromCoG
        %
        % Computes the static wheel loads based on some mass and CoG position
        %
        % INPUTS:
        %   m: Mass [kg]
        %   l: CoG position from front axle (+ve towards rear) [m]
        %   dy: CoG offset from vehicle centre line (+ve towards right) [m]
        % OUTPUTS:
        %   FL: Front left wheel load [N]
        %   FR: Front right wheel load [N]
        %   RL: Rear left wheel load [N]
        %   RR: Rear right wheel load [N]
        function [FL, FR, RL, RR] = staticWheelLoadsFromCoG(this, m, l, dy)
            % Can form a system of equations for the loads at each wheel:
            %   - Moment balance about rear axle
            %   - Moment balance about front axle
            %   - Moment balance across front axle
            %   - Moment balance across rear axle
            %
            % MATLAB's symbolic solver was used to solve for each wheel load:
            %
            % syms FL FR RL RR W L l1 dy bf br;
            % eq1 = FL * L + FR * L == W * (L - l1);
            % eq2 = RL * L + RR * L == W * l1;
            % eq3 = -FL * bf / 2 + FR * bf / 2 == W * (L - l1) / L * dy;
            % eq4 = -RL * br / 2 + RR * br / 2 == W * l1 / L * dy;
            % S = solve([eq1, eq2, eq3, eq4], [FL, FR, RL, RR]);

            % Convert mass to force
            W = m * this.gravity;

            FL = (this.L * this.track_f * W - 2 * this.L * dy .* W ...
                - this.track_f * l .* W + 2 * dy .* l .* W) ...
                / (2 * this.L * this.track_f);
            FR = (this.L * this.track_f * W + 2 * this.L * dy .* W ...
                - this.track_f * l .* W - 2 * dy .* l .* W) ...
                / (2 * this.L * this.track_f);
            RL = (this.track_r * l .* W - 2 * dy .* l .* W) / (2 * this.L * this.track_r);
            RR = (this.track_r * l .* W + 2 * dy .* l .* W) / (2 * this.L * this.track_r);
        end

        % % rideHeightAtVelocityOptFunction
        % %
        % % Optimiation function to use for solving for the ride height at a
        % % velocity.
        % %
        % % INPUTS:
        % %   v: Velocity [m/s]
        % %   xf: Front damper position [mm]
        % %   xr: Rear damper position [mm]
        % % OUTPUTS:
        % %   F: Result of the force balance F_sprung + F_aero - F_springs
        % function F = rideHeightAtVelocityOptFunction(this, v, xf, xr)
        %     % Setup a force balance between the front/rear springs and downforce
        %     % functions
        %     F_static = this.gravity * this.totalMass();
        %     F_kf = 2 * this.damperToWheelForce(this.springForce(xf, true), true);
        %     F_kr = 2 * this.damperToWheelForce(this.springForce(xr, false), false);
        %
        %     [RH_f_opt, RH_r_opt] = this.rideHeightFromDamperPos(xf, xr);
        %     pitch = this.pitchFromAvgRideHeights(RH_f_opt, RH_r_opt);
        %
        %     F_aero_f = this.DF_f(v, RH_f_opt, pitch) + ...
        %         this.DF_f_front_element(v, this.wing_front) + ...
        %         this.DF_f_rear_element(v, this.wing_front);
        %
        %     F_aero_r = this.DF_r(v, RH_f_opt, pitch) + ...
        %         this.DF_r_front_element(v, this.wing_front) + ...
        %         this.DF_r_rear_element(v, this.wing_front);
        %
        %     F = F_static + F_aero_f + F_aero_r - F_kf - F_kr;
        % end
    end
end
