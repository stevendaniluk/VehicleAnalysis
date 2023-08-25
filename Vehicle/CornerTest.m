classdef CornerTest < matlab.unittest.TestCase
    properties
        corner = Corner;
    end

    methods (TestMethodSetup)
        function createSampleCorner(this)
            this.corner.MR_spring = 1.5;
            this.corner.kspring = 8e4;
            this.corner.kbump = 1e5;
            this.corner.bumpstop_thickness = 0.02;

            % Make a shock that has the spring engaged but with zero preload
            this.corner.shock_travel = 0.10;
            this.corner.shock_length = 0.35;
            this.corner.spring_perch_offset = 0.08;
            this.corner.spring_cup_height = 0.05;
            this.corner.spring_length = 0.220;
            this.corner.x0 = 0;
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
            F_wheel =  F_spring / this.corner.MR_spring ^2;

            F_spring_calc = this.corner.wheelToSpringForce(F_wheel);
            F_wheel_calc = this.corner.springToWheelForce(F_spring);

            this.verifyEqual(F_wheel_calc, F_wheel, 'AbsTol', 1e-6);
            this.verifyEqual(F_spring_calc, F_spring, 'AbsTol', 1e-6);
        end

        function springForceZeroAtZeroTravel(this)
            F = this.corner.springForce(0.0);
            this.verifyEqual(F, 0);
        end

        function springForceAtNonZeroTravel(this)
            % Out default shock has zero preload, so the force is simply the damper
            % displacement
            x = 1.23;
            F_target = x * this.corner.kspring;

            F = this.corner.springForce(x);
            this.verifyEqual(F, F_target, 'AbsTol', 1e-6);
        end

        function springForceAccountsForDamperOffset(this)
            % Compute a reference value
            x = 1.23;
            F_target = this.corner.springForce(x);

            % Add some offset to the corner as well as to the input position
            this.corner.x0 = -32.489;
            x = x + this.corner.x0;

            F = this.corner.springForce(x);

            this.verifyEqual(F, F_target, 'AbsTol', 1e-6);
        end

        function springForceWithPreload(this)
            % Compute some reference position
            x = 1.23;

            % Apply some delta to that position and compute the force
            delta = 4.56;
            x_offset = x + delta;
            F_offset_travel = this.corner.springForce(x_offset);

            % If we increase the spring perch offset by the same delta but evaluated at
            % the original position we should get the same force
            this.corner.spring_perch_offset = this.corner.spring_perch_offset + delta;
            F_preload = this.corner.springForce(x);

            this.verifyEqual(F_preload, F_offset_travel, 'AbsTol', 1e-6);
        end

        function bumpstopForceWhenEngaged(this)
            % Compute a damper position that will have the bumpstop engaged and evaluate
            % the force at that point
            x_bumpstop = 0.8 * this.corner.bumpstop_thickness;
            x = this.corner.shock_travel - (this.corner.bumpstop_thickness - x_bumpstop);
            F_bumpstop =  this.corner.bumpstopForce(x);

            F_bumpstop_target = x_bumpstop * this.corner.kbump;

            this.verifyEqual(F_bumpstop, F_bumpstop_target, 'AbsTol', 1e-6);
        end

        function bumpstopForceZeroWhenNotEngaged(this)
            % Compute a damper position that will not have the bumpstop engaged
            x = this.corner.shock_travel - this.corner.bumpstop_thickness - 1e-3;
            F_bumpstop =  this.corner.bumpstopForce(x);

            this.verifyEqual(F_bumpstop, 0, 'AbsTol', 1e-6);
        end

        function setPreloadFromMeasuredWheelForce(this)
            % Query the force at some non-zero position
            x = 0.03;
            F = this.corner.totalWheelForce(x, 0);

            % Now aletr the perch offset and recompute, should get the original value
            og_val = this.corner.spring_perch_offset;
            this.corner.spring_perch_offset = this.corner.spring_perch_offset - 0.2;
            this.corner = this.corner.setPreloadFromMeasuredWheelForce(x, F);

            this.verifyEqual(this.corner.spring_perch_offset, og_val, 'AbsTol', 1e-6);
        end
    end
end
