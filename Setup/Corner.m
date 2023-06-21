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
        % Bumpstop thickness [mm]
        bumpstop_thickness = 0;
        % Fully extended shock length [mm]
        shock_length = 0;
        % Available shock travel [mm]
        shock_travel = 0;
        % Spring perch offset (distance from rod end to spring perch) [mm]
        spring_perch_offset = 0;
        % Spring cup height (distance from rod end to spring cup)[mm]
        spring_cup_height = 0;
        % Spring length [mm]
        spring_length = 0;
        % Damper position when shock fully extended (optional offset) [mm]
        x0 = 0;

        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Anti-Roll Bars

        % ARB motion ratio (Wheel/Bar)
        MR_arb = 1;
        % ARB rate [N/m]
        karb = 0;
    end

    methods
        % springToWheelMotion
        %
        % INPUTS:
        %   x_d: Motion at spring/damper
        % OUTPUTS:
        %   x_w: Equivalent motion at wheel
        function x_w = springToWheelMotion(this, x_d)
            x_w = this.MR_spring * x_d;
        end

        % wheelToSpringMotion
        %
        % INPUTS:
        %   x_w: Motion at wheel
        % OUTPUTS:
        %   x_d: Equivalent motion at spring/damper
        function x_d = wheelToSpringMotion(this, x_w)
            x_d = x_w / this.MR_spring;
        end

        % springToWheelForce
        %
        % INPUTS:
        %   F_d: Force at spring/damper
        % OUTPUTS:
        %   F_w: Equivalent force at wheel
        function F_w = springToWheelForce(this, F_d)
            F_w = this.MR_spring^2 * F_d;
        end

        % wheelToSpringForce
        %
        % INPUTS:
        %   F_w: Force at wheel
        % OUTPUTS:
        %   F_d: Equivalent force at spring/damper
        function F_d = wheelToSpringForce(this, F_w)
            F_d = F_w / this.MR_spring^2;
        end

        % arbToWheelMotion
        %
        % INPUTS:
        %   x_arb: Motion at ARB
        % OUTPUTS:
        %   x_w: Equivalent motion at wheel
        function x_w = arbToWheelMotion(this, x_arb)
            x_w = this.MR_arb * x_arb;
        end

        % wheelToArbMotion
        %
        % INPUTS:
        %   x_w: Motion at wheel
        % OUTPUTS:
        %   x_arb: Equivalent motion at ARB
        function x_arb = wheelToArbMotion(this, x_w)
            x_arb = x_w / this.MR_arb;
        end

        % arbToWheelForce
        %
        % INPUTS:
        %   F_arb: Force at ARB
        % OUTPUTS:
        %   F_w: Equivalent force at wheel
        function F_w = arbToWheelForce(this, F_arb)
            F_w = this.MR_arb^2 * F_arb;
        end

        % wheelToArbForce
        %
        % INPUTS:
        %   F_w: Force at wheel
        % OUTPUTS:
        %   F_arb: Equivalent force at ARB
        function F_arb = wheelToArbForce(this, F_w)
            F_arb = F_w / this.MR_arb^2;
        end

        % damperPosToSpringForce
        %
        % INPUTS:
        %   x: Damper position [mm]
        % OUTPUTS:
        %   F: Force at shock due to spring and bumpstops
        function F = damperPosToSpringForce(this, x)
            F = 0;

            % Apply offset to damper position measurement
            x = x - this.x0;

            % Compute how much the spring is compressed accounting for spring perch
            % position (spring may not always be compressed)
            spring_height = this.shock_length - this.spring_perch_offset - this.spring_cup_height;
            spring_compressed = (spring_height - x) < this.spring_length;

            % We only compute force when the damper is compressed, at full extension
            % all force is restrained by the shock body
            if x > 0 && spring_compressed
                kx_spring = this.spring_length - (spring_height - x);
                F = F + 1e-3 * kx_spring * this.kspring;
            end

            % Determine how much the bumpstop is compressed
            travel_remaining = max(0, this.shock_travel - x);
            if travel_remaining < this.bumpstop_thickness
                F = F + 1e-3 * (this.bumpstop_thickness - travel_remaining) * this.kbump;
            end
        end

        % bumpStopEngaged
        %
        % INPUTS:
        %   x: Damper position [mm]
        % OUTPUTS:
        %   engaged: True when the bumpstop is depressed, false otherwise
        function engaged = bumpStopEngaged(this, x)
            % Apply offset to damper position measurement
            x = x - this.x0;

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
        function this = setSpringRatesFromWheel(k_wheel)
            this.kspring = k_wheel / this.MR_spring^2;
        end

    end
end
