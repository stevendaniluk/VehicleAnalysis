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
        bump_thickness = 0;
        % Spring perch offset (distance from rod end to spring perch) [mm]
        spring_perch_offset = 0;
        % Spring cup height (distance from rod end to spring cup)[mm]
        spring_cup_height = 0;
        % Spring length [mm]
        spring_length = 0;
        % Fully extended shock length [mm]
        shock_length = 0;
        % Available shock travel [mm]
        shock_travel = 0;
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

        % damperPosToSpringForce
        %
        % INPUTS:
        %   x: Damper position [mm]
        % OUTPUTS:
        %   F: Force at shock due to srping
        function F = damperPosToSpringForce(this, x)
            % Apply offset to damper position measurement
            x_eff = x - this.x0;

            % Compute how much the spring is compressed accounting for spring perch
            % position (spring may not always be compressed)
            kx = this.spring_length - ...
                (this.shock_length - this.spring_perch - this.spring_cup_height) ...
                - x_eff;

            if kx > 0
                F = 1e-3 * kx * this.kspring;
            else
                F = 0;
            end
        end

        % springToWheelForce
        %
        % INPUTS:
        %   F: Force at front damper
        % OUTPUTS:
        %   F: Equivalent force at wheel
        function F = springToWheelForce(this, F)
            F = this.MR_spring^2 * F;
        end

        % wheelToSpringForce
        %
        % INPUTS:
        %   F: Force at front wheel
        % OUTPUTS:
        %   F: Equivalent force at damper
        function F = wheelToSpringForce(this, F)
            F = F / this.MR_spring^2;
        end
    end
end
