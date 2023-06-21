classdef CornerTest < matlab.unittest.TestCase
    properties
        corner = Corner;
    end

    methods (TestMethodSetup)
        function createSampleCorner(this)
            this.corner.MR_spring = 1.5;
            this.corner.kspring = 8e4;
            this.corner.kbump = 1e5;
            this.corner.bumpstop_thickness = 20.0;

            % Make a shock that has the spring engaged but with zero preload
            this.corner.shock_travel = 100.0;
            this.corner.shock_length = 350.0;
            this.corner.spring_perch_offset = 80.0;
            this.corner.spring_cup_height = 50.0;
            this.corner.spring_length = 220.0;
            this.corner.x0 = 0;

            this.corner.MR_arb = 1.2;
            this.corner.karb = 1e4;
        end
    end

    methods (Test)
        function wheelSpringMotionConversions(this)
            % Wheel motion is scaled by the motion ratio
            x_spring = 1.0;
            x_wheel = this.corner.MR_spring * x_spring;

            x_spring_calc = this.corner.wheelToSpringMotion(x_wheel);
            x_wheel_calc = this.corner.springToWheelMotion(x_spring);

            this.verifyEqual(x_wheel_calc, x_wheel, 'AbsTol', 1e-6);
            this.verifyEqual(x_spring_calc, x_spring, 'AbsTol', 1e-6);
        end

        function wheelSpringForceConversions(this)
            % Wheel force is scaled by the square of the motion ratio
            F_spring = 1e5;
            F_wheel = this.corner.MR_spring ^2 * F_spring;

            F_spring_calc = this.corner.wheelToSpringForce(F_wheel);
            F_wheel_calc = this.corner.springToWheelForce(F_spring);

            this.verifyEqual(F_wheel_calc, F_wheel, 'AbsTol', 1e-6);
            this.verifyEqual(F_spring_calc, F_spring, 'AbsTol', 1e-6);
        end

        function wheelArbMotionConversions(this)
            % ARB motion is scaled by the motion ratio
            x_arb = 1.0;
            x_wheel = this.corner.MR_arb * x_arb;

            x_arb_calc = this.corner.wheelToArbMotion(x_wheel);
            x_wheel_calc = this.corner.arbToWheelMotion(x_arb);

            this.verifyEqual(x_wheel_calc, x_wheel, 'AbsTol', 1e-6);
            this.verifyEqual(x_arb_calc, x_arb, 'AbsTol', 1e-6);
        end

        function wheelArbForceConversions(this)
            % Wheel force is scaled by the square of the motion ratio
            F_arb = 1e5;
            F_wheel = this.corner.MR_arb ^2 * F_arb;

            F_arb_calc = this.corner.wheelToArbForce(F_wheel);
            F_wheel_calc = this.corner.arbToWheelForce(F_arb);

            this.verifyEqual(F_wheel_calc, F_wheel, 'AbsTol', 1e-6);
            this.verifyEqual(F_arb_calc, F_arb, 'AbsTol', 1e-6);
        end

        function springForceZeroAtZeroTravel(this)
            F = this.corner.damperPosToSpringForce(0.0);
            this.verifyEqual(F, 0);
        end

        function springForceAtNonZeroTravel(this)
            % Out default shock has zero preload, so the force is simply the damper
            % displacement
            x = 1.23;
            F_target = x * 1e-3 * this.corner.kspring;

            F = this.corner.damperPosToSpringForce(x);
            this.verifyEqual(F, F_target, 'AbsTol', 1e-6);
        end

        function springForceAccountsForDamperOffset(this)
            % Compute a reference value
            x = 1.23;
            F_target = this.corner.damperPosToSpringForce(x);

            % Add some offset to the corner as well as to the input position
            delta = -32.489;
            this.corner.x0 = -32.489;
            x = x + this.corner.x0;

            F = this.corner.damperPosToSpringForce(x);

            this.verifyEqual(F, F_target, 'AbsTol', 1e-6);
        end

        function springForceWithPreload(this)
            % Compute some reference position
            x = 1.23;

            % Apply some delta to that position and compute the force
            delta = 4.56;
            x_offset = x + delta;
            F_offset_travel = this.corner.damperPosToSpringForce(x_offset);

            % If we increase the spring perch offset by the same delta but evaluated at
            % the original position we should get the same force
            this.corner.spring_perch_offset = this.corner.spring_perch_offset + delta;
            F_preload = this.corner.damperPosToSpringForce(x);

            this.verifyEqual(F_preload, F_offset_travel, 'AbsTol', 1e-6);
        end

        function springForceWithBumpstopEngaged(this)
            % Compute a damper position that will have the bumpstop engaged and evaluate
            % the force at that point
            x_bumpstop = 0.8 * this.corner.bumpstop_thickness;
            x = this.corner.shock_travel - (this.corner.bumpstop_thickness - x_bumpstop);
            F_bumpstop =  this.corner.damperPosToSpringForce(x);

            % Now eliminate the bumpstop and reevaluate the force
            this.corner.bumpstop_thickness = 0;
            F_no_bumpstop =  this.corner.damperPosToSpringForce(x);

            % Compute what we expect the bumpstop contribution to be and add that to the
            % no bumpstop case, this should be what we computed originally
            F_bumpstop_target = F_no_bumpstop + 1e-3 * x_bumpstop * this.corner.kbump;

            this.verifyEqual(F_bumpstop, F_bumpstop_target, 'AbsTol', 1e-6);
        end
    end
end
