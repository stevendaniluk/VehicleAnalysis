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
        % Sprung mass CoG distance from rear axle [m]
        lm2 = 0;
        % Sprung mass CoG height above the ground [m]
        h_sprung;
        % Rotational inertia about the Z axis
        Iz = 0;
        % Fuel density [kg/L]
        p_fuel = 0;
        % Fuel level [L]
        V_fuel = 0;
        % Fuel tank CoG distance from front axle [m]
        lf1 = 0;
        % Fuel tank CoG distance from rear axle [m]
        lf2 = 0;
        % Fuel tank CoG height above the ground [m]
        h_fuel;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Wheel Geometry

        % Toe angle [rad] (-ve is toe in)
        toe_f = 0;
        toe_r = 0;
        % Camber angle [rad] (-ve is angled in)
        camber_f = 0;
        camber_r = 0;
        % Caster angle [rad] (+ve is inclined rearwards)
        caster = 0;
        % Steering ratio (input/output)
        steer_ratio = 1;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Suspension

        % Static ride heights [mm]
        RH_f_static = 0;
        RH_r_static  = 0;
        % Ride height at the neutral damper position [mm]
        RH0_f = 0;
        RH0_r = 0;
        % Neutral damper positions [mm]
        xf0 = 0;
        xr0 = 0;
        % Damper/spring motion ratios
        MR_spring_f = 1;
        MR_spring_r = 1;
        % ARB motion ratios
        MR_arb_f = 1;
        MR_arb_r = 1;
        % Suspension spring rates [N/m]
        kspring_f = 0;
        kspring_r = 0;
        % Wheel spring rates [N/m]
        kspring_wheel_f = 0;
        kspring_wheel_r = 0;
        % ARB rates [N/m]
        karb_f = 0;
        karb_r = 0;
        % Wheel ARB rates [N/m]
        karb_wheel_f = 0;
        karb_wheel_r = 0;

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
            delta_wheel_FL = delta / this.steer_ratio - this.toe_f;
            delta_wheel_FR = delta / this.steer_ratio + this.toe_f;
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

        % setRideHeightOffset
        %
        % Computes the ride height offset for the current ride height and
        % estimated damper position from the sprung mass and fuel load.
        function this = setRideHeightOffset(this)
            F_fuel_f = this.V_fuel * this.p_fuel * this.g * (this.lf2 / this.L);
            F_fuel_r = this.V_fuel * this.p_fuel * this.g * (this.lf1 / this.L);

            F_sprung_f = this.m_sprung * this.g * (this.lm2 / this.L);
            F_sprung_r = this.m_sprung * this.g * (this.lm1 / this.L);

            xf_pred = 1e3 * this.wheelToDamperForce(0.5 * (F_fuel_f + F_sprung_f), true) / ...
                this.kspring_f;
            xr_pred = 1e3 * this.wheelToDamperForce(0.5 * (F_fuel_r + F_sprung_r), false) / ...
                this.kspring_r;

            this.RH0_f = this.RH_f_static + this.MR_spring_f  * xf_pred;
            this.RH0_r = this.RH_r_static + this.MR_spring_r * xr_pred;
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

        % rollFromRideHeights
        %
        % INPUTS:
        %   RH_l: Left ride height [mm]
        %   RH_r: Right ride height [mm]
        %   front: True for front axle, false for rear axle
        % OUTPUTS:
        %   roll: Roll angle [deg]
        function roll = rollFromRideHeights(this, RH_l, RH_r, front)
            if front
                roll = atan2d(1e-3 * (RH_l - RH_r), this.b_f);
            else
                roll = atan2d(1e-3 * (RH_l - RH_r), this.b_r);
            end
        end

        % rideHeightFromDamperPos
        %
        % INPUTS:
        %   xf: Front damper position
        %   xr: Rear damper position
        % OUTPUTS:
        %   RH_f: Front ride height
        %   RH_r: Rear ride height
        function [RH_f, RH_r] = rideHeightFromDamperPos(this, xf, xr)
            RH_f = this.RH0_f - this.MR_spring_f * xf;
            RH_r = this.RH0_r - this.MR_spring_r * xr;
        end

        % damperPosFromRideHeight
        %
        % INPUTS:
        %   RH_f: Front ride height
        %   RH_r: Rear ride height
        % OUTPUTS:
        %   xf: Front damper position
        %   xr: Rear damper position
        function [xf, xr] = damperPosFromRideHeight(this, RH_f, RH_r)
            xf = (this.RH0_f - RH_f) / this.MR_spring_f;
            xr = (this.RH0_r - RH_r) / this.MR_spring_r;
        end

        % downforceFromDamperPos
        %
        % Computes the downforce on the front and rear axles based on the damper
        % positions. This will account for the force from the sprung mass and
        % fuel level. It will not account for any weight transfer.
        %
        % INPUTS:
        %   xf: Front damper position
        %   xr: Rear damper position
        % OUTPUTS:
        %   F: Total downforce [N]
        %   balance: Fraction of total downforce on front axle
        function [F, balance] = downforceFromDamperPos(this, xf, xr)
            % Compute the total force on the springs
            F_total_f = 2 * this.damperToWheelForce(this.springForce(xf, true), true);
            F_total_r = 2 * this.damperToWheelForce(this.springForce(xr, false), false);

            % Subtract the sprung mass and fuel level
            F_sprung_f = this.m_sprung * this.g * this.lm1 / this.L;
            F_sprung_r = this.m_sprung * this.g * this.lm2 / this.L;

            F_fuel_f = this.p_fuel * this.V_fuel * this.g * this.lf2 / this.L;
            F_fuel_r = this.p_fuel * this.V_fuel * this.g * this.lf1 / this.L;

            D_f = F_total_f - F_sprung_f - F_fuel_f;
            D_r = F_total_r - F_sprung_r - F_fuel_r;
            F = D_f + D_r;
            balance = D_f ./ (D_f + D_r);
        end

        % setWheelRatesFromSprings
        %
        % Sets the wheel rate (kspring_wheel_f/r) based on the spring rate the
        % the damper (kspring_f/r) and the motion rate (MR_spring_f/r).
        function this = setWheelRatesFromSprings(this)
            this.kspring_wheel_f = this.MR_spring_f^2 * this.kspring_f;
            this.kspring_wheel_r = this.MR_spring_r^2 * this.kspring_r;
        end

        % setSpringRatesFromWheel
        %
        % Sets the spring rate at the damper (kspring_f/r) based on the spring
        % rate at the wheel(kspring_wheel_f/r) and the motion rate (MR_spring_f/r).
        function this = setSpringRatesFromWheel(this)
            this.kspring_f = this.kspring_wheel_f / this.MR_spring_f^2;
            this.kspring_r = this.kspring_wheel_r / this.MR_spring_r^2;
        end

        % springForce
        %
        % INPUTS:
        %   x: Damper position [mm]
        %   front: True for front springs, false for rear springs
        % OUTPUTS:
        %   F: Force at damper due to srping
        function F = springForce(this, x, front)
            if front
                F = 1e-3 * (x - this.xf0) * this.kspring_f;
            else
                F = 1e-3 * (x - this.xr0) * this.kspring_r;
            end
        end

        % damperToWheelForce
        %
        % INPUTS:
        %   F: Force at damper
        %   front: True for front springs, false for rear springs
        % OUTPUTS:
        %   F: Equivalent force at wheel
        function F = damperToWheelForce(this, F, front)
            if front
                F = this.MR_spring_f^2 * F;
            else
                F = this.MR_spring_r^2 * F;
            end
        end

        % wheelToDamperForce
        %
        % INPUTS:
        %   F: Force at wheel
        %   front: True for front springs, false for rear springs
        % OUTPUTS:
        %   F: Equivalent force at damper
        function F = wheelToDamperForce(this, F, front)
            if front
                F = F / this.MR_spring_f^2;
            else
                F = F / this.MR_spring_r^2;
            end
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
            delta_wheel_RL = -this.toe_r;
            delta_wheel_RR = this.toe_r;

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
