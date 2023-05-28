% Setup
%
% Defines all the properties for a vehicle setup, as well as utilities for
% querying vehicle properties (e.g. wheel force from damper position).
classdef Setup
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Chassis

        % Wheelbase [m]
        L;
        % Front track width [m]
        b_f;
        % Rear track width [m]
        b_r;
        % Sprung mass at  zero fuel load [kg]
        m_sprung = 0;
        % Unsprung mass for front axle [kg]
        m_unsprung_f = 0;
        % Unsprung mass for rear axle [kg]
        m_unsprung_r = 0;
        % Sprung mass CoG distance from front axle [m]
        lm1 = 0;
        % Sprung mass CoG height for a ride height of zero [m]
        h_sprung;
        % Rotational inertia about the Z axis
        Iz = 0;
        % Fuel density [kg/L]
        p_fuel = 0;
        % Fuel level [L]
        V_fuel = 0;
        % Fuel tank CoG distance from front axle [m]
        lf1 = 0;
        % Fuel tank CoG height for a ride height of zero [m]
        h_fuel;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Suspension and Wheel Geometry

        % Steering ratio (input/output)
        steer_ratio = 1;
        % Ackermann ratio [0, 1]
        % (0.0 --> wheel angles are equal, 1.0 --> coincident IC's)
        ackermann = 0;

        % Suspension and geometry for individual corners
        FL_corner = Corner();
        FR_corner = Corner();
        RL_corner = Corner();
        RR_corner = Corner();

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Aerodynamics

        % Functions for front downforce, rear downforce, and drag force of the
        % chassis/floor (no additional aero elements) as a function of velocity
        % [m/s], front ride height [mm], and rake angle[deg].
        %
        % Can be evaluated as:
        %   F = DF_f(v, RH_f, rake)
        DF_f = @(v, RH_f, rake) 0;
        DF_r = @(v, RH_f, rake) 0;
        drag = @(v, RH_f, rake) 0;

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
    end

    properties (Access = private)
        % Ride height at zero damper positions [mm]
        RH_0_fl = 0;
        RH_0_fr = 0;
        RH_0_rl = 0;
        RH_0_rr = 0;
    end

    properties (Constant)
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Environment

        % Air density [kg/m^3]
        p_air = 1.225;
        % Gravitational acceleration [m/s^2]
        g = 9.8067;
    end

    methods
        % steerToWheelAngles
        %
        % INPUTS:
        %   delta: Steering wheel angle (+ve turns right) [rad]
        % OUTPUTS:
        %   delta_wheel_FL: Front left wheel angle (+ve turns right) [rad]
        %   delta_wheel_FR: Front right wheel angle (+ve turns right) [rad]
        function [delta_wheel_FL, delta_wheel_FR] = steerToWheelAngles(this, delta)
            delta_wheel_FL = delta / this.steer_ratio - this.FL_corner.toe;
            delta_wheel_FR = delta / this.steer_ratio + this.FR_corner.toe;
        end

        % totalMass
        %
        % OUTPUTS:
        %   m: Total vehicle mass including fuel
        function m = totalMass(this)
            m = this.m_sprung + this.m_unsprung_f + this.m_unsprung_r + ...
                this.fuelMass(this.V_fuel);
        end

        % fuelMass
        %
        % INPUTS:
        %   V_fuel: Volume of fuel [L]
        % OUTPUTS:
        %   m: Mass of fuel load
        function m = fuelMass(this, V_fuel)
            m = this.p_fuel * V_fuel;
        end

        % setReferenceRideHeight
        %
        % Provides a baseline measurement for ride height given damper positions, from
        % which the ride height can be computed for any damper position.
        %
        % INPUTS:
        %   RH_f: Front ride height
        %   RH_r: Rear ride height
        %   x_fl: FL damper position
        %   x_fr: FR damper position
        %   x_rl: RL damper position
        %   x_rr: RR damper position
        function this = setReferenceRideHeight(RH_f, RH_r, x_fl, x_fr, x_rl, x_rr)
            RH_0_fl = RH_f + this.FL_corner.MR_spring * (x_fl - this.FL_corner.x0);
            RH_0_fr = RH_f + this.FR_corner.MR_spring * (x_fr - this.FR_corner.x0);
            RH_0_rl = RH_r + this.RL_corner.MR_spring * (x_rl - this.RL_corner.x0);
            RH_0_rr = RH_r + this.RR_corner.MR_spring * (x_rr - this.RR_corner.x0);
        end

        % rideHeightFromDamperPos
        %
        % INPUTS:
        %   x_fl: Front left damper position
        %   x_fr: Front right damper position
        %   x_rl: Rear left damper position
        %   x_rr: Rear right damper position
        % OUTPUTS:
        %   RH_fl: Front left ride height
        %   RH_fr: Front right ride height
        %   RH_rl: Rear left ride height
        %   RH_rr: Rear right ride height
        function [RH_fl, RH_fr, RH_rl, RH_rr] = rideHeightFromDamperPos(this, x_fl, x_fr, x_rl, x_rr)
            RH_fl = this.RH_0_fl - (x_fl - this.FL_corner.x0) * this.FL_corner.MR_spring;
            RH_fr = this.RH_0_fr - (x_fr - this.FR_corner.x0) * this.FR_corner.MR_spring;
            RH_rl = this.RH_0_rl - (x_rl - this.RL_corner.x0) * this.RL_corner.MR_spring;
            RH_rr = this.RH_0_rr - (x_rr - this.RR_corner.x0) * this.RR_corner.MR_spring;
        end

        % avgRideHeightFromDamperPos
        %
        % INPUTS:
        %   x_fl: Front left damper position
        %   x_fr: Front right damper position
        %   x_rl: Rear left damper position
        %   x_rr: Rear right damper position
        % OUTPUTS:
        %   RH_f: Average front ride height
        %   RH_r: Average rear ride height
        function [RH_f, RH_r] = avgRideHeightFromDamperPos(this, x_fl, x_fr, x_rl, x_rr)
            [RH_fl, RH_fr, RH_rl, RH_rr] = rideHeightFromDamperPos(this, x_fl, x_fr, x_rl, x_rr);
            RH_f = (RH_fl + RH_fr) ./ 2;
            RH_r = (RH_rl + RH_rr) ./ 2;
        end

        % damperPosFromRideHeight
        %
        % INPUTS:
        %   RH_fl: Front left ride height
        %   RH_fr: Front right ride height
        %   RH_rl: Rear left ride height
        %   RH_rr: Rear right ride height
        % OUTPUTS:
        %   x_fl: Front left damper position
        %   x_fr: Front right damper position
        %   x_rl: Rear left damper position
        %   x_rr: Rear right damper position
        function [x_fl, x_fr, x_rl, x_rr] = damperPosFromRideHeight(this, RH_fl, RH_fr, RH_rl, RH_rr)
            x_fl = (this.RH_0_fl - RH_fl) ./ this.FL_corner.MR_spring + this.FL_corner.x0;
            x_fr = (this.RH_0_fr - RH_fr) ./ this.FR_corner.MR_spring + this.FR_corner.x0;
            x_rl = (this.RH_0_rl - RH_rl) ./ this.RL_corner.MR_spring + this.RL_corner.x0;
            x_rr = (this.RH_0_rr - RH_rr) ./ this.RR_corner.MR_spring + this.RR_corner.x0;
        end

        % rakeFromRideHeights
        %
        % INPUTS:
        %   RH_f: Front ride height [mm]
        %   RH_r: Rear ride height [mm]
        % OUTPUTS:
        %   rake: Rake angle [deg]
        function rake = rakeFromRideHeights(this, RH_f, RH_r)
            rake = atan2d(1e-3 * (RH_r - RH_f), this.L);
        end

        % rollFromFrontRideHeights
        %
        % INPUTS:
        %   RH_fl: Front left ride height [mm]
        %   RH_fr: Front right ride height [mm]
        % OUTPUTS:
        %   roll: Roll angle [deg]
        function roll = rollFromFrontRideHeights(this, RH_fl, RH_fr)
            roll = atan2d(1e-3 * (RH_fl - RH_fr), this.b_f);
        end

        % rollFromRearRideHeights
        %
        % INPUTS:
        %   RH_rl: Rear left ride height [mm]
        %   RH_rr: Rear right ride height [mm]
        % OUTPUTS:
        %   roll: Roll angle [deg]
        function roll = rollFromRearRideHeights(this, RH_rl, RH_rr)
            roll = atan2d(1e-3 * (RH_rl - RH_rr), this.b_r);
        end

        % downforceFromDamperPos
        %
        % Computes the downforce on the front and rear axles based on the damper
        % positions. This will account for the force from the sprung mass and
        % fuel level. It will not account for any weight transfer.
        %
        % INPUTS:
        %   x_fl: Front left damper position
        %   x_fr: Front right damper position
        %   x_rl: Rear left damper position
        %   x_rr: Rear right damper position
        % OUTPUTS:
        %   F: Total downforce [N]
        %   balance: Fraction of total downforce on front axle
        function [F, balance] = downforceFromDamperPos(this, x_fl, x_fr, x_rl, x_rr)
            % Compute the total force on the springs
            F_k_FL = this.FL_corner.springToWheelForce(this.FL_corner.damperPosToSpringForce(x_fl));
            F_k_FR = this.FR_corner.springToWheelForce(this.FR_corner.damperPosToSpringForce(x_fr));
            F_k_RL = this.RL_corner.springToWheelForce(this.RL_corner.damperPosToSpringForce(x_rl));
            F_k_RR = this.RR_corner.springToWheelForce(this.RR_corner.damperPosToSpringForce(x_rr));

            F_total_f = F_k_FL + F_k_FR;
            F_total_r = F_k_RL + F_k_RR;

            % Subtract the sprung mass and fuel level
            F_sprung_f = this.m_sprung * this.g * this.lm1 / this.L;
            F_sprung_r = this.m_sprung * this.g * (this.L - this.lm1) / this.L;

            F_fuel_f = this.p_fuel * this.V_fuel * this.g * (this.L - this.lf1) / this.L;
            F_fuel_r = this.p_fuel * this.V_fuel * this.g * this.lf1 / this.L;

            D_f = F_total_f - F_sprung_f - F_fuel_f;
            D_r = F_total_r - F_sprung_r - F_fuel_r;
            F = D_f + D_r;
            balance = D_f ./ (D_f + D_r);
        end

        % aeroForceCoefficient
        %
        % INPUTS:
        %   v: Measured velocity [m/s]
        %   F: Measured force (drag or downforce) [N]
        % OUTPUTS:
        %   C: Coefficient of force (per unit area)
        function C = aeroForceCoefficient(this, v, F)
            C = F ./ (0.5 * this.p_air * v.^2);
        end

        % expectedWheelSpeed
        %
        % Compute the wheel speeds expected to be measured on each corner
        % accounting for path curvature and wheel angle.
        %
        % INPUTS:
        %   v: Vehicle velocity [m/s]
        %   w: Vehicle yaw rate (+ve turning right) [rad/s]
        %   delta: Steering wheel angle (+ve turns right) [rad]
        % OUTPUTS:
        %   FL: Front left wheel velocity vector
        %   FR: Front right wheel velocity vector
        %   RL: Rear left wheel velocity vector
        %   RR: Rear right wheel velocity vector
        function [FL, FR, RL, RR] = expectedWheelSpeed(this, v, w, delta)
            % Relative velocities due to rotation
            vx_f_rel = w * 0.5 * this.b_f;
            vx_r_rel = w * 0.5 * this.b_r;
            vy_rel = w * 0.5 * this.L;

            % Compute the corner velocities from the linear and angular velocity
            FL = [v + vx_f_rel; vy_rel];
            FR = [v - vx_f_rel; vy_rel];
            RL = [v + vx_r_rel; -vy_rel];
            RR = [v - vx_r_rel; -vy_rel];

            % Get the angle of each wheel
            [delta_wheel_FL, delta_wheel_FR] = this.steerToWheelAngles(delta);
            delta_wheel_RL = -this.RL_corner.toe;
            delta_wheel_RR = this.RR_corner.toe;

            % Correct the wheel velocities for the wheel angles
            c_delta = cos(-delta_wheel_FL);
            s_delta = sin(-delta_wheel_FL);
            FL = [c_delta .* FL(1, :) - s_delta .* FL(2, :); ...
                s_delta .* FL(1, :) + c_delta .* FL(2, :)];

            c_delta = cos(-delta_wheel_FR);
            s_delta = sin(-delta_wheel_FR);
            FR = [c_delta .* FR(1, :) - s_delta .* FR(2, :); ...
                s_delta .* FR(1, :) + c_delta .* FR(2, :)];

            c_delta = cos(-delta_wheel_RL);
            s_delta = sin(-delta_wheel_RL);
            RL = [c_delta .* RL(1, :) - s_delta .* RL(2, :); ...
                s_delta .* RL(1, :) + c_delta .* RL(2, :)];

            c_delta = cos(-delta_wheel_RR);
            s_delta = sin(-delta_wheel_RR);
            RR = [c_delta .* RR(1, :) - s_delta .* RR(2, :); ...
                s_delta .* RR(1, :) + c_delta .* RR(2, :)];
        end

        % rideHeightAtVelocity
        %
        % Esimates the steady state ride heights, rake angle, and damper
        % positions at a specific velocity based on the current spring
        % rates and aerodynamic downforce functions.
        %
        % INPUTS:
        %   v: Velocity [m/s]
        % OUTPUTS:
        %   RH_f: Front ride height [mm]
        %   RH_r: Rear ride height [mm]
        %   rake: Rake angle [deg]
        %   xf: Front damper position [mm]
        %   xr: Rear damper position [mm]
        function [RH_f, RH_r, rake, xf, xr] = rideHeightAtVelocity(this, v)
            func = @(x) this.rideHeightAtVelocityOptFunction(v, x(1), x(2));

            [xf, xr] = this.damperPosFromRideHeight(this.RH_f_static, this.RH_r_static);
            x0 = [xf, xr];
            x = fsolve(func, x0);

            xf = x(1);
            xr = x(2);
            [RH_f, RH_r] = this.rideHeightFromDamperPos(xf, xr);
            rake = this.rakeFromRideHeights(RH_f, RH_r);
        end
    end

    methods (Access = protected)

        % rideHeightAtVelocityOptFunction
        %
        % Optimiation function to use for solving for the ride height at a
        % velocity.
        %
        % INPUTS:
        %   v: Velocity [m/s]
        %   xf: Front damper position [mm]
        %   xr: Rear damper position [mm]
        % OUTPUTS:
        %   F: Result of the force balance F_sprung + F_aero - F_springs
        function F = rideHeightAtVelocityOptFunction(this, v, xf, xr)
            % Setup a force balance between the front/rear springs and downforce
            % functions
            F_static = this.g * this.totalMass();
            F_kf = 2 * this.damperToWheelForce(this.springForce(xf, true), true);
            F_kr = 2 * this.damperToWheelForce(this.springForce(xr, false), false);

            [RH_f_opt, RH_r_opt] = this.rideHeightFromDamperPos(xf, xr);
            rake = this.rakeFromRideHeights(RH_f_opt, RH_r_opt);

            F_aero_f = this.DF_f(v, RH_f_opt, rake) + ...
                this.DF_f_front_element(v, this.wing_front) + ...
                this.DF_f_rear_element(v, this.wing_front);

            F_aero_r = this.DF_r(v, RH_f_opt, rake) + ...
                this.DF_r_front_element(v, this.wing_front) + ...
                this.DF_r_rear_element(v, this.wing_front);

            F = F_static + F_aero_f + F_aero_r - F_kf - F_kr;
        end
    end
end
