% Corner
%
% Defines all the suspension properties for a single corner of a vehicle
classdef Corner
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Wheel Geometry

        % Toe angle [rad] (-ve is toe in)
        toe = 0;
        % Camber angle [rad] (-ve is angled in)
        camber = 0;
        % Caster angle [rad] (+ve is inclined rearwards)
        caster = 0;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Shocks

        % Spring and damper motion ratio (Wheel/Damper)
        MR_spring = 1;
        % Suspension spring rate [N/m]
        kspring = 0;
        % Bumpstop spring rate [N/m]
        kbump = 0;
        % Bumpstop thickness [m]
        bumpstop_thickness = 0;
        % Fully extended shock length [m]
        shock_length = 0;
        % Available shock travel [m]
        shock_travel = 0;
        % Spring perch offset (distance from rod end to spring perch) [m]
        spring_perch_offset = 0;
        % Spring cup height (distance from rod end to spring cup)[m]
        spring_cup_height = 0;
        % Spring length [m]
        spring_length = 0;
        % Damper position when shock fully extended (optional offset) [m]
        x0 = 0;
        % Damper force function (result is [N], input is [m/s])
        F_damper = @(v) 0;
    end

    methods
        % correctedDamperPos
        %
        % INPUTS:
        %   x: Damper position reading [m]
        % OUTPUTS:
        %   x: Adjusted damper position such that 0 corresponds to full extension [m]
        function x = correctedDamperPos(this, x)
            x = x - this.x0;
        end

        % springToWheelMotion
        %
        % INPUTS:
        %   x_d: Motion at spring/damper [m]
        % OUTPUTS:
        %   x_w: Equivalent motion at wheel [m]
        function x_w = springToWheelMotion(this, x_d)
            x_w = this.MR_spring * x_d;
        end

        % wheelToSpringMotion
        %
        % INPUTS:
        %   x_w: Motion at wheel [m]
        % OUTPUTS:
        %   x_d: Equivalent motion at spring/damper [m]
        function x_d = wheelToSpringMotion(this, x_w)
            x_d = x_w / this.MR_spring;
        end

        % springToWheelForce
        %
        % INPUTS:
        %   F_d: Force at spring/damper [N]
        % OUTPUTS:
        %   F_w: Equivalent force at wheel [N]
        function F_w = springToWheelForce(this, F_d)
            F_w = F_d / this.MR_spring^2;
        end

        % wheelToSpringForce
        %
        % INPUTS:
        %   F_w: Force at wheel [N]
        % OUTPUTS:
        %   F_d: Equivalent force at spring/damper [N]
        function F_d = wheelToSpringForce(this, F_w)
            F_d = F_w * this.MR_spring^2;
        end

        % totalShockForce
        %
        % INPUTS:
        %   x: Damper position [m]
        %   v: Damper velocity [m/s]
        % OUTPUTS:
        %   F: Force at shock due to spring, bumpstop, and damper [N]
        function F = totalShockForce(this, x, v)
            F = this.springForce(x) + this.bumpstopForce(x) ...
                + this.damperForce(v);
        end

        % totalWheelForce
        %
        % INPUTS:
        %   x: Damper position [m]
        %   v: Damper velocity [m/s]
        % OUTPUTS:
        %   F: Force at wheel due to spring, bumpstop, and damper [N]
        function F = totalWheelForce(this, x, v)
            F = this.springToWheelForce(this.totalShockForce(x, v));
        end

        % springForce
        %
        % INPUTS:
        %   x: Damper position [m]
        % OUTPUTS:
        %   F: Force at shock due to spring [N]
        function F = springForce(this, x)
            F = zeros(size(x));

            % Apply offset to damper position measurement
            x = this.correctedDamperPos(x);

            % Compute how much the spring is compressed accounting for spring perch
            % position (spring may not always be compressed)
            spring_length_on_shock = this.shock_length - this.spring_perch_offset ...
                - this.spring_cup_height;
            spring_compressed = (spring_length_on_shock - x) < this.spring_length;

            % We only compute force when the damper is compressed, at full extension
            % all force is restrained by the shock body
            kx_spring = this.spring_length - (spring_length_on_shock - x);
            F_spring = kx_spring * this.kspring;

            compressed_pts = and(x > 0, spring_compressed);
            F(compressed_pts) = F(compressed_pts) + F_spring(compressed_pts);
        end

        % springForceAtWheel
        %
        % INPUTS:
        %   x: Damper position [m]
        % OUTPUTS:
        %   F: Force at shock due to spring [N]
        function F = springForceAtWheel(this, x)
            F = this.springToWheelForce(this.springForce(x));
        end

        % bumpstopForce
        %
        % INPUTS:
        %   x: Damper position [m]
        % OUTPUTS:
        %   F: Force at shock due to bumpstop [N]
        function F = bumpstopForce(this, x)
            F = zeros(size(x));

            % See if we've reached the bumpstop
            travel_remaining = max(0, this.shock_travel - x);
            F_bump = (this.bumpstop_thickness - travel_remaining) * this.kbump;

            touching = travel_remaining < this.bumpstop_thickness;
            F(touching) = F(touching) + F_bump(touching);
        end

        % bumpstopForceAtWheel
        %
        % INPUTS:
        %   x: Damper position [m]
        % OUTPUTS:
        %   F: Force at wheel due to bumpstop [N]
        function F = bumpstopForceAtWheel(this, x)
            F = this.springToWheelForce(this.bumpstopForce(x));
        end

        % damperForce
        %
        % INPUTS:
        %   v: Damper velocity [m/s]
        % OUTPUTS:
        %   F: Force at shock due to damper [N]
        function F = damperForce(this, v)
            F = this.F_damper(v);
        end

        % damperForceAtWheel
        %
        % INPUTS:
        %   v: Damper velocity [m/s]
        % OUTPUTS:
        %   F: Force at wheel due to damper [N]
        function F = damperForceAtWheel(this, v)
            F = this.springToWheelForce(this.damperForce(v));
        end

        % bumpStopEngaged
        %
        % INPUTS:
        %   x: Damper position [m]
        % OUTPUTS:
        %   engaged: True when the bumpstop is depressed, false otherwise
        function engaged = bumpStopEngaged(this, x)
            % Apply offset to damper position measurement
            x = this.correctedDamperPos(x);

            travel_remaining = max(0, this.shock_travel - x);
            engaged = travel_remaining < this.bumpstop_thickness;
        end

        % setSpringRateFromWheelRate
        %
        % Sets the spring rate at the shock based on the motion ratio and the provided
        % spring rate at the wheel.
        %
        % INPUTS:
        %   k_wheel: Effective spring rate at wheel [N/m]
        function this = setSpringRatesFromWheel(this, k_wheel)
            this.kspring = k_wheel / this.MR_spring^2;
        end

        % setPreloadFromMeasuredWheelForce
        %
        % Computes the amount of preload present on the shock based on measured position
        % and force.
        %
        % INPUTS:
        %   x: Damper position [m]
        %   F: Measured force at the wheel [N]
        function this = setPreloadFromMeasuredWheelForce(this, x, F)
            F = this.wheelToSpringForce(F);

            % Apply offset to damper position measurement
            x = this.correctedDamperPos(x);

            % Subtract any bumpstop forces to give only spring force
            F = F - this.bumpstopForce(x);

            if F <= 0
                % Cannot determine perch position, default to zero
                this.spring_perch_offset = 0;
                return;
            end

            dx_spring = F / this.kspring;
            eff_spring_length = this.spring_length - dx_spring;

            % Solve for spring perch position based on shock dimensions and spring
            % compression
            this.spring_perch_offset =  (this.shock_length - x) - ...
                this.spring_cup_height - eff_spring_length;
        end

    end
end
