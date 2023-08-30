classdef VehicleTest < matlab.unittest.TestCase
    properties
        vehicle = Vehicle;
    end

    methods (TestMethodSetup)
        function createSampleVehicle(this)
            % Create a vehicle that has non-zero (and reasonable) values for its
            % properties. No offsets for damper positions of ride height will be set.

            this.vehicle.L = 2.0;
            this.vehicle.track_f = 1.2;
            this.vehicle.track_r = 1.2;

            this.vehicle.m_sprung = 1e3;
            this.vehicle.l_sprung = 0.5 * this.vehicle.L;
            this.vehicle.dy_sprung = 0;
            this.vehicle.h_sprung = 0.8;

            this.vehicle.m_unsprung_f = 100;
            this.vehicle.h_unsprung_f = 0.3;

            this.vehicle.m_unsprung_r = 100;
            this.vehicle.h_unsprung_r = 0.3;

            this.vehicle.V_fuel = 60.0;
            this.vehicle.l_fuel = 0.5 * this.vehicle.L;
            this.vehicle.h_fuel = 0.5;

            this.vehicle.steer_ratio = 10.0;
            this.vehicle.ackermann = 0;

            this.vehicle.brake_bias = 0.5;

            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FL_corner.camber = 0;
            this.vehicle.FL_corner.caster = 0;
            this.vehicle.FL_corner.MR_spring = 1.2;
            this.vehicle.FL_corner.kspring = 120e3;
            this.vehicle.FL_corner.kbump = 0;
            this.vehicle.FL_corner.bumpstop_thickness = 0;
            this.vehicle.FL_corner.shock_length = 0.3;
            this.vehicle.FL_corner.shock_travel = 0.3;
            this.vehicle.FL_corner.spring_perch_offset = 0;
            this.vehicle.FL_corner.spring_cup_height = 0;
            this.vehicle.FL_corner.spring_length = 0.3;
            this.vehicle.FL_corner.x0 = 0;

            this.vehicle.FR_corner = this.vehicle.FL_corner;
            this.vehicle.RL_corner = this.vehicle.FL_corner;
            this.vehicle.RR_corner = this.vehicle.FL_corner;

            this.vehicle.MR_arb_f = 1.5;
            this.vehicle.MR_arb_r = 1.5;

            this.vehicle.karb_f = 2e4;
            this.vehicle.karb_r = 1e4;

            this.vehicle.h_roll_f = @(RH) 0.1;
            this.vehicle.h_roll_r = @(RH) 0.3;

            this.vehicle.SVIC_ang_f = 0.0;
            this.vehicle.SVIC_ang_r = 0.0;

            this.vehicle.r_nominal_f = 0.35;
            this.vehicle.r_nominal_r = 0.35;
            this.vehicle.r_eff_f = @(P, w, Fz) 0.35;
            this.vehicle.r_eff_r = @(P, w, Fz) 0.35;
        end
    end

    methods (Test)
        function staticWheelLoadsEqualDistribution(this)
            % Make unsprung weights equal and all CoG positions at vehicle centre, the
            % resulting corner weights should all be equal
            this.vehicle.m_unsprung_f = 100;
            this.vehicle.m_unsprung_r = 100;
            this.vehicle.l_sprung = 0.5 * this.vehicle.L;
            this.vehicle.dy_sprung = 0;
            this.vehicle.l_fuel = 0.5 * this.vehicle.L;
            this.vehicle.V_fuel = 100.0;

            [FL, FR, RL, RR] = this.vehicle.staticWheelLoads();

            this.assertGreaterThan(FL, 0);

            F_target = this.vehicle.gravity * this.vehicle.totalMass() / 4;
            this.verifyEqual(FL, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(FR, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(RL, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_target, 'AbsTol', 1e-6);
        end

        function staticWheelLoadsVaryingSprungMassPosition(this)
            % Make sure track widths are equal for all checks
            this.vehicle.track_r = this.vehicle.track_f;

            % Only sprung mass coincident with some pair of wheels
            this.vehicle.m_unsprung_f = 0;
            this.vehicle.m_unsprung_r = 0;
            this.vehicle.V_fuel = 0;
            this.vehicle.dy_sprung = 0;
            F_target = this.vehicle.gravity * this.vehicle.totalMass() / 2;

            % Positioned at front axle
            this.vehicle.l_sprung = 0;

            [FL, FR, RL, RR] = this.vehicle.staticWheelLoads();
            this.verifyEqual(FL, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(FR, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Positioned at rear axle
            this.vehicle.l_sprung = this.vehicle.L;

            [FL, FR, RL, RR] = this.vehicle.staticWheelLoads();
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_target, 'AbsTol', 1e-6);

            % Positioned at right side at mid point
            this.vehicle.l_sprung = 0.5 * this.vehicle.L;
            this.vehicle.dy_sprung = 0.5 * this.vehicle.track_f;

            [FL, FR, RL, RR] = this.vehicle.staticWheelLoads();
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, F_target, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_target, 'AbsTol', 1e-6);
        end

        function staticWheelLoadsVaryingUnsprungMass(this)
            % Set sprung mass and fuel level to zero so only unsprung mass is present
            this.vehicle.m_sprung = 0;
            this.vehicle.V_fuel = 0;

            % Unsprung mass should  be equally split between wheels
            [FL, FR, RL, RR] = this.vehicle.staticWheelLoads();
            F_target_f = 0.5 * this.vehicle.m_unsprung_f * this.vehicle.gravity;
            F_target_r = 0.5 * this.vehicle.m_unsprung_r * this.vehicle.gravity;
            this.verifyEqual(FL, F_target_f, 'AbsTol', 1e-6);
            this.verifyEqual(FR, F_target_f, 'AbsTol', 1e-6);
            this.verifyEqual(RL, F_target_r, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_target_r, 'AbsTol', 1e-6);
        end

        function staticWheelLoadsVaryingFuelPosition(this)
            % Set fuel load to zero to get a reference point
            this.vehicle.V_fuel = 0;
            [FL_ref, FR_ref, RL_ref, RR_ref] = this.vehicle.staticWheelLoads();

            % Add fuel at some fraction of the wheelbase, should see that fractor of the
            % fuel weight added to each corner
            this.vehicle.V_fuel = 80.0;
            F_fuel = this.vehicle.fuelMass() * this.vehicle.gravity;
            L_frac = 0.2;
            this.vehicle.l_fuel = L_frac * this.vehicle.L;

            F_delta_f = (1 - L_frac) * F_fuel;
            F_delta_r = L_frac * F_fuel;

            [FL, FR, RL, RR] = this.vehicle.staticWheelLoads();
            this.verifyEqual(FL, FL_ref + 0.5 * F_delta_f, 'AbsTol', 1e-6);
            this.verifyEqual(FR, FR_ref + 0.5 * F_delta_f, 'AbsTol', 1e-6);
            this.verifyEqual(RL, RL_ref + 0.5 * F_delta_r, 'AbsTol', 1e-6);
            this.verifyEqual(RR, RR_ref + 0.5 * F_delta_r, 'AbsTol', 1e-6);
        end

        function totalCogHeightFromComponents(this)
            RH_f = 0.0;
            RH_r = 0.0;

            % With zero unsprung mass and fuel load the CoG height should match the
            % sprung mass CoG height
            this.vehicle.m_unsprung_f = 0;
            this.vehicle.m_unsprung_r = 0;
            this.vehicle.V_fuel = 0;

            h_cog = this.vehicle.totalCogHeight(RH_f, RH_r);
            this.verifyEqual(h_cog, this.vehicle.h_sprung, 'AbsTol', 1e-6);

            % Add other masses individually and set them to be equal to the sprung mass,
            % the resulting CoG height should be the average of the two heights

            % Front unsprung
            this.vehicle.m_unsprung_f = this.vehicle.m_sprung;
            this.vehicle.h_unsprung_f = this.vehicle.h_sprung - 0.2;

            h_cog = this.vehicle.totalCogHeight(RH_f, RH_r);
            h_target = 0.5 * (this.vehicle.h_sprung + this.vehicle.h_unsprung_f);
            this.verifyEqual(h_cog, h_target, 'AbsTol', 1e-6);

            this.vehicle.m_unsprung_f = 0;

            % Rear unsprung
            this.vehicle.m_unsprung_r = this.vehicle.m_sprung;
            this.vehicle.h_unsprung_r = this.vehicle.h_sprung - 0.2;

            h_cog = this.vehicle.totalCogHeight(RH_f, RH_r);
            h_target = 0.5 * (this.vehicle.h_sprung + this.vehicle.h_unsprung_r);
            this.verifyEqual(h_cog, h_target, 'AbsTol', 1e-6);

            this.vehicle.m_unsprung_r = 0;

            % Fuel
            this.vehicle.V_fuel = this.vehicle.m_sprung / this.vehicle.rho_fuel;
            this.vehicle.h_fuel = this.vehicle.h_sprung - 0.2;

            h_cog = this.vehicle.totalCogHeight(RH_f, RH_r);
            h_target = 0.5 * (this.vehicle.h_sprung + this.vehicle.h_fuel);
            this.verifyEqual(h_cog, h_target, 'AbsTol', 1e-6);
        end

        function totalCogLongPositionFromComponents(this)
            % With zero unsprung mass and fuel load the CoG position should match the
            % sprung mass CoG position
            this.vehicle.m_unsprung_f = 0;
            this.vehicle.m_unsprung_r = 0;
            this.vehicle.V_fuel = 0;

            l_cog = this.vehicle.totalCogLongPosition();
            this.verifyEqual(l_cog, this.vehicle.l_sprung, 'AbsTol', 1e-6);

            % Add other masses individually and set them to be equal to the sprung mass,
            % the resulting CoG position should be the average of the two

            % Front unsprung
            this.vehicle.m_unsprung_f = this.vehicle.m_sprung;

            l_cog = this.vehicle.totalCogLongPosition();
            l_target = 0.5 * this.vehicle.l_sprung;
            this.verifyEqual(l_cog, l_target, 'AbsTol', 1e-6);

            this.vehicle.m_unsprung_f = 0;

            % Rear unsprung
            this.vehicle.m_unsprung_r = this.vehicle.m_sprung;

            l_cog = this.vehicle.totalCogLongPosition();
            l_target = 0.5 * (this.vehicle.l_sprung + this.vehicle.L);
            this.verifyEqual(l_cog, l_target, 'AbsTol', 1e-6);

            this.vehicle.m_unsprung_r = 0;

            % Fuel
            this.vehicle.V_fuel = this.vehicle.m_sprung / this.vehicle.rho_fuel;
            this.vehicle.l_fuel = this.vehicle.l_sprung + 0.2;

            l_cog = this.vehicle.totalCogLongPosition();
            l_target = 0.5 * (this.vehicle.l_sprung + this.vehicle.l_fuel);
            this.verifyEqual(l_cog, l_target, 'AbsTol', 1e-6);
        end

        function totalCogLatPositionFromComponents(this)
            % With zero unsprung mass and fuel load the CoG position should match the
            % sprung mass CoG position
            this.vehicle.m_unsprung_f = 0;
            this.vehicle.m_unsprung_r = 0;
            this.vehicle.V_fuel = 0;
            this.vehicle.dy_sprung = 0.123;

            dy_cog = this.vehicle.totalCogLatPosition();
            this.verifyEqual(dy_cog, this.vehicle.dy_sprung, 'AbsTol', 1e-6);

            % Add other masses individually and set them to be equal to the sprung mass,
            % each time we should see the lateral position get halved
            dy_target = 0.5 * this.vehicle.dy_sprung;

            % Front unsprung
            this.vehicle.m_unsprung_f = this.vehicle.m_sprung;

            dy_cog = this.vehicle.totalCogLatPosition();
            this.verifyEqual(dy_cog, dy_target, 'AbsTol', 1e-6);

            this.vehicle.m_unsprung_f = 0;

            % Rear unsprung
            this.vehicle.m_unsprung_r = this.vehicle.m_sprung;

            dy_cog = this.vehicle.totalCogLatPosition();
            this.verifyEqual(dy_cog, dy_target, 'AbsTol', 1e-6);

            this.vehicle.m_unsprung_r = 0;

            % Fuel
            this.vehicle.V_fuel = this.vehicle.m_sprung / this.vehicle.rho_fuel;
            this.vehicle.l_fuel = this.vehicle.l_sprung + 0.2;

            dy_cog = this.vehicle.totalCogLatPosition();
            this.verifyEqual(dy_cog, dy_target, 'AbsTol', 1e-6);
        end

        function steerToWheelAnglesNoToeNoAckermann(this)
            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;
            this.vehicle.ackermann = 0;

            delta_steer = deg2rad(35);
            delta_target = delta_steer / this.vehicle.steer_ratio;

            % Steer right
            [delta_l, delta_r] = this.vehicle.steerToWheelAngles(delta_steer);
            this.verifyEqual(delta_l, delta_target, 'AbsTol', 1e-6);
            this.verifyEqual(delta_r, delta_target, 'AbsTol', 1e-6);

            % Steer left
            [delta_l, delta_r] = this.vehicle.steerToWheelAngles(-delta_steer);
            this.verifyEqual(delta_l, -delta_target, 'AbsTol', 1e-6);
            this.verifyEqual(delta_r, -delta_target, 'AbsTol', 1e-6);
        end

        function steerToWheelAnglesWithToe(this)
            % Get some reference wheel angles with zero toe
            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;

            delta_steer = deg2rad(35);
            [delta_l_ref, delta_r_ref] = this.vehicle.steerToWheelAngles(delta_steer);

            % Now add some toe out
            toe = deg2rad(0.3);
            this.vehicle.FL_corner.toe = toe;
            this.vehicle.FR_corner.toe = toe;

            [delta_l, delta_r] = this.vehicle.steerToWheelAngles(delta_steer);
            this.verifyEqual(delta_l, delta_l_ref - toe, 'AbsTol', 1e-6);
            this.verifyEqual(delta_r, delta_r_ref + toe, 'AbsTol', 1e-6);
        end

        function steerToWheelAnglesWith100Ackermann(this)
            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;

            % At 100% ackermann the difference in turning radius of each wheel should be
            % equal to the track width
            this.vehicle.ackermann = 1.0;
            delta_steer = deg2rad(35);
            [delta_l, delta_r] = this.vehicle.steerToWheelAngles(delta_steer);

            r_l = this.vehicle.L / tan(delta_l);
            r_r = this.vehicle.L / tan(delta_r);
            r_diff = r_l - r_r;

            this.verifyEqual(r_diff, this.vehicle.track_f, 'AbsTol', 1e-3);
        end

        function steerToWheelAngleMidAckermann(this)
            % The ackermann ratio is:
            %             theta_i - theta_o
            %     A = ------------------------
            %         theta_i_ack - theta_o_ack
            % Where theta_i is the actual angle and theta_i_ack is the angle for 100%
            % ackermann geometry.
            %
            % Compute some wheel angles for 100% ackermann and some other value, the
            % ratio of angle differences should produce the ackermann value

            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;
            delta_steer = deg2rad(35);

            this.vehicle.ackermann = 1.0;
            [delta_l_ack, delta_r_ack] = this.vehicle.steerToWheelAngles(delta_steer);

            this.vehicle.ackermann = 0.345;
            [delta_l, delta_r] = this.vehicle.steerToWheelAngles(delta_steer);

            ack = (delta_r - delta_l) / (delta_r_ack - delta_l_ack);

            this.verifyEqual(ack, this.vehicle.ackermann, 'AbsTol', 1e-3);
        end

        function wheelSpeedComponentsEqualForStraightMotion(this)
            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;
            this.vehicle.RL_corner.toe = 0;
            this.vehicle.RR_corner.toe = 0;

            v = 12.34;
            w = 0;
            delta = 0;
            [FL, FR, RL, RR] = this.vehicle.cornerWheelSpeedComponents(v, w, delta);

            v_target = [v, 0];
            this.verifyEqual(FL, v_target, 'AbsTol', 1e-6);
            this.verifyEqual(FR, v_target, 'AbsTol', 1e-6);
            this.verifyEqual(RL, v_target, 'AbsTol', 1e-6);
            this.verifyEqual(RR, v_target, 'AbsTol', 1e-6);
        end

        function wheelSpeedComponentsWithSteering(this)
            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;
            this.vehicle.RL_corner.toe = 0;
            this.vehicle.RR_corner.toe = 0;

            % Use steer ratio of 1 for convenien
            this.vehicle.steer_ratio = 1;

            v = 12.34;
            w = 0;

            % Set to 45 degrees, should see equal magnitudes in X and Y directions on front
            delta = deg2rad(45);
            [FL, FR, RL, RR] = this.vehicle.cornerWheelSpeedComponents(v, w, delta);

            v_target_f = [v / sqrt(2), -v / sqrt(2)];
            v_target_r = [v, 0];
            this.verifyEqual(FL, v_target_f, 'AbsTol', 1e-6);
            this.verifyEqual(FR, v_target_f, 'AbsTol', 1e-6);
            this.verifyEqual(RL, v_target_r, 'AbsTol', 1e-6);
            this.verifyEqual(RR, v_target_r, 'AbsTol', 1e-6);

            % Set to 90 degrees, should only see Y componnent on fronts
            delta = deg2rad(90);
            [FL, FR, RL, RR] = this.vehicle.cornerWheelSpeedComponents(v, w, delta);

            v_target_f = [0, -v];
            v_target_r = [v, 0];
            this.verifyEqual(FL, v_target_f, 'AbsTol', 1e-6);
            this.verifyEqual(FR, v_target_f, 'AbsTol', 1e-6);
            this.verifyEqual(RL, v_target_r, 'AbsTol', 1e-6);
            this.verifyEqual(RR, v_target_r, 'AbsTol', 1e-6);
        end

        function wheelSpeedComponentsWithAngularVelocity(this)
            this.vehicle.FL_corner.toe = 0;
            this.vehicle.FR_corner.toe = 0;
            this.vehicle.RL_corner.toe = 0;
            this.vehicle.RR_corner.toe = 0;

            % Make front and rear track widths different to make sure both are accounted
            % for
            this.vehicle.track_r = this.vehicle.track_f + 0.1;

            v = 0;
            w = 0.123;
            delta = 0;

            [FL, FR, RL, RR] = this.vehicle.cornerWheelSpeedComponents(v, w, delta);

            this.verifyEqual([FL, 0], cross([0, 0, w], [0.5 * this.vehicle.L, -0.5 * this.vehicle.track_f, 0]), 'AbsTol', 1e-6);
            this.verifyEqual([FR, 0], cross([0, 0, w], [0.5 * this.vehicle.L, 0.5 * this.vehicle.track_f, 0]), 'AbsTol', 1e-6);
            this.verifyEqual([RL, 0], cross([0, 0, w], [-0.5 * this.vehicle.L, -0.5 * this.vehicle.track_r, 0]), 'AbsTol', 1e-6);
            this.verifyEqual([RR, 0], cross([0, 0, w], [-0.5 * this.vehicle.L, 0.5 * this.vehicle.track_r, 0]), 'AbsTol', 1e-6);
        end

        function wheelSpeedComponentsWithToe(this)
            % Give a different toe agle to each corner
            this.vehicle.FL_corner.toe = deg2rad(5);
            this.vehicle.FR_corner.toe = deg2rad(10);
            this.vehicle.RL_corner.toe = deg2rad(15);
            this.vehicle.RR_corner.toe = deg2rad(20);

            v = 10;
            w = 0;
            delta = 0;
            [FL, FR, RL, RR] = this.vehicle.cornerWheelSpeedComponents(v, w, delta);

            % Form a velocity vector and multiply it by a rotation matrix to rotate it
            % by the toe angle (+ve angles are toe out)
            v_vec = [v; 0; 0];
            this.verifyEqual([FL, 0]', rotz(rad2deg(this.vehicle.FL_corner.toe)) * v_vec, 'AbsTol', 1e-6);
            this.verifyEqual([FR, 0]', rotz(-rad2deg(this.vehicle.FR_corner.toe)) * v_vec, 'AbsTol', 1e-6);
            this.verifyEqual([RL, 0]', rotz(rad2deg(this.vehicle.RL_corner.toe)) * v_vec, 'AbsTol', 1e-6);
            this.verifyEqual([RR, 0]', rotz(-rad2deg(this.vehicle.RR_corner.toe)) * v_vec, 'AbsTol', 1e-6);
        end

        function rotateImuToGroundPlaneNoChangeForEqualRideHeight(this)
            a = [1.11; 2.22; 3.33];
            a_rot = this.vehicle.rotateImuToGroundPlane(a, 0.1, 0.1, 0.1, 0.1);
            this.verifyEqual(a_rot, a, 'AbsTol', 1e-6);
        end

        function rotateImuToGroundPlaneUnequalRideHeight(this)
            % Make an acceleration vector only in the Z direction like gravity so that
            % when we roll or pitch the car there will be some component in other
            % directions
            a = [0; 0; -10];

            % Vehicle pitched up by 45 degrees should result in equal components in the
            % X and Z directions, and appear to be accelerating forward in the X direction
            RH_rear = 0;
            RH_front = this.vehicle.L * tand(45);

            a_rot = this.vehicle.rotateImuToGroundPlane(a, RH_front, RH_front, RH_rear, RH_rear);
            this.verifyEqual(a_rot(1), -a(3) * sqrt(2) / 2, 'AbsTol', 1e-6);
            this.verifyEqual(a_rot(3), a(3) * sqrt(2) / 2, 'AbsTol', 1e-6);

            % Vehicle rolled right side up by 45 degrees should result in equal
            % components in the Y and Z directions, and appear to be accelerating in the
            % Y direction
            this.vehicle.track_r = this.vehicle.track_f;
            RH_left = 0;
            RH_right = this.vehicle.track_f * tand(45);

            a_rot = this.vehicle.rotateImuToGroundPlane(a, RH_left, RH_right, RH_left, RH_right);
            this.verifyEqual(a_rot(2), -a(3) * sqrt(2) / 2, 'AbsTol', 1e-6);
            this.verifyEqual(a_rot(3), a(3) * sqrt(2) / 2, 'AbsTol', 1e-6);
        end

        function rideHeightFromDamperPosZeroForZeroDamperPosition(this)
            % With no ride height offset set a damper position of zero will correspond
            % to the chassis bottom touching the ground
            [RH_fl, RH_fr, RH_rl, RH_rr] = this.vehicle.rideHeightFromDamperPos(0, 0, 0, 0);
            this.verifyEqual(RH_fl, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_fr, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rl, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rr, 0, 'AbsTol', 1e-6);
        end

        function rideHeightFromDamperPosConversion(this)
            % Only check changes in ride height and not individual values so that the
            % effects of offsets don't have to be considered

            % Get some initial ride height
            x = [0.111, 0.222, 0.333, 0.444];
            [RH_fl_ref, RH_fr_ref, RH_rl_ref, RH_rr_ref] = this.vehicle.rideHeightFromDamperPos(x(1), x(2), x(3), x(4));

            % Perturb the damper positions
            x_delta = 0.123;
            x = x + x_delta * ones(size(x));
            [RH_fl, RH_fr, RH_rl, RH_rr] = this.vehicle.rideHeightFromDamperPos(x(1), x(2), x(3), x(4));

            % New ride height should be the reference height reduced by the perturbation
            this.verifyEqual(RH_fl, RH_fl_ref - x_delta * this.vehicle.FL_corner.MR_spring, 'AbsTol', 1e-6);
            this.verifyEqual(RH_fr, RH_fr_ref - x_delta * this.vehicle.FR_corner.MR_spring, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rl, RH_rl_ref - x_delta * this.vehicle.RL_corner.MR_spring, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rr, RH_rr_ref - x_delta * this.vehicle.RR_corner.MR_spring, 'AbsTol', 1e-6);
        end

        function rideHeightFromDamperPosAccountsForRideHeightOffsets(this)
            % With no ride height offset set a damper position of zero will correspond
            % to the chassis bottom touching the ground
            this.vehicle.FL_corner.x0 = 0;
            this.vehicle.FR_corner.x0 = 0;
            this.vehicle.RL_corner.x0 = 0;
            this.vehicle.RR_corner.x0 = 0;

            [RH_fl, RH_fr, RH_rl, RH_rr] = this.vehicle.rideHeightFromDamperPos(0, 0, 0, 0);
            this.verifyEqual(RH_fl, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_fr, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rl, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rr, 0, 'AbsTol', 1e-6);

            % Now apply some offset
            RH_f_target = 0.80;
            RH_r_target = 0.110;
            x = [0.111, 0.222, 0.333, 0.444];
            this.vehicle = this.vehicle.setReferenceRideHeight(RH_f_target, RH_r_target, x(1), x(2), x(3), x(4));

            % When we evaluate at those same damper positions we should get the reference
            % ride height
            [RH_fl, RH_fr, RH_rl, RH_rr] = this.vehicle.rideHeightFromDamperPos(x(1), x(2), x(3), x(4));
            this.verifyEqual(RH_fl, RH_f_target, 'AbsTol', 1e-6);
            this.verifyEqual(RH_fr, RH_f_target, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rl, RH_r_target, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rr, RH_r_target, 'AbsTol', 1e-6);
        end

        function rideHeightFromDamperPosAccountsForDamperOffsets(this)
            % Set all offsets to some value, then evaluate the ride height at those values,
            % the resulting ride height should be zero
            x = [0.111, 0.222, 0.333, 0.444];

            this.vehicle.FL_corner.x0 = x(1);
            this.vehicle.FR_corner.x0 = x(2);
            this.vehicle.RL_corner.x0 = x(3);
            this.vehicle.RR_corner.x0 = x(4);

            [RH_fl, RH_fr, RH_rl, RH_rr] = this.vehicle.rideHeightFromDamperPos(x(1), x(2), x(3), x(4));
            this.verifyEqual(RH_fl, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_fr, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rl, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RH_rr, 0, 'AbsTol', 1e-6);
        end

        function damperPosFromRideHeightMatchesInverseCalculation(this)
            % Set some damper and ride height offsets
            this.vehicle.FL_corner.x0 = 0.12;
            this.vehicle.FR_corner.x0 = 0.345;
            this.vehicle.RL_corner.x0 = 0.679;
            this.vehicle.RR_corner.x0 = 0.901;

            RH_f_target = 0.80;
            RH_r_target = 0.110;
            x = [0.111, 0.222, 0.333, 0.444];
            this.vehicle = this.vehicle.setReferenceRideHeight(RH_f_target, RH_r_target, x(1), x(2), x(3), x(4));

            % Compute some new ride height
            x = 2 * x;
            [RH_fl, RH_fr, RH_rl, RH_rr] = this.vehicle.rideHeightFromDamperPos(x(1), x(2), x(3), x(4));

            % Now compute the damper positions from this ride height, they should match
            % the input damper positions
            [x_fl, x_fr, x_rl, x_rr] = this.vehicle.damperPosFromRideHeight(RH_fl, RH_fr, RH_rl, RH_rr);
            this.verifyEqual(x_fl, x(1), 'AbsTol', 1e-6);
            this.verifyEqual(x_fr, x(2), 'AbsTol', 1e-6);
            this.verifyEqual(x_rl, x(3), 'AbsTol', 1e-6);
            this.verifyEqual(x_rr, x(4), 'AbsTol', 1e-6);
        end

        function arbWheelMotionConversions(this)
            % ARB motion is scaled by the motion ratio
            x_arb = 1.0;

            x_wheel_f = this.vehicle.MR_arb_f * x_arb;
            x_arb_calc_f = this.vehicle.frontWheelToArbMotion(x_wheel_f);
            x_wheel_calc_f = this.vehicle.frontArbToWheelMotion(x_arb);

            this.verifyEqual(x_wheel_calc_f, x_wheel_f, 'AbsTol', 1e-6);
            this.verifyEqual(x_arb_calc_f, x_arb, 'AbsTol', 1e-6);

            x_wheel_r = this.vehicle.MR_arb_r * x_arb;
            x_arb_calc_r = this.vehicle.rearWheelToArbMotion(x_wheel_r);
            x_wheel_calc_r = this.vehicle.rearArbToWheelMotion(x_arb);

            this.verifyEqual(x_wheel_calc_r, x_wheel_r, 'AbsTol', 1e-6);
            this.verifyEqual(x_arb_calc_r, x_arb, 'AbsTol', 1e-6);
        end

        function arbWheelForceConversions(this)
            % Wheel force is scaled by the square of the motion ratio
            F_arb = 1e5;

            F_wheel_f = this.vehicle.MR_arb_f ^2 * F_arb;
            F_arb_calc_f = this.vehicle.frontWheelToArbForce(F_wheel_f);
            F_wheel_calc_f = this.vehicle.frontArbToWheelForce(F_arb);

            this.verifyEqual(F_wheel_calc_f, F_wheel_f, 'AbsTol', 1e-6);
            this.verifyEqual(F_arb_calc_f, F_arb, 'AbsTol', 1e-6);

            F_wheel_r = this.vehicle.MR_arb_r ^2 * F_arb;
            F_arb_calc_r = this.vehicle.rearWheelToArbForce(F_wheel_r);
            F_wheel_calc_r = this.vehicle.rearArbToWheelForce(F_arb);

            this.verifyEqual(F_wheel_calc_r, F_wheel_r, 'AbsTol', 1e-6);
            this.verifyEqual(F_arb_calc_r, F_arb, 'AbsTol', 1e-6);
        end

        function arbForcesZeroForEqualSuspPositions(this)
            x = 0.1234;
            [FL, FR, RL, RR] = this.vehicle.arbForces(x, x, x, x);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function arbWheelForcesForNonEqualSuspPositions(this)
            % Make sure front and rear motion ratios and spring rates are different
            this.vehicle.MR_arb_f = 1.5 * this.vehicle.MR_arb_r;
            this.vehicle.karb_f = 1.5 * this.vehicle.karb_r;

            % For some amount of ride height perturbation compute the expected wheel loads
            % (arbs force is halved to give the individual wheel loads, not the sum)
            %   F_wheel = MR^2 * F_arb
            %           = MR^2 * dx_arb * k_arb
            %           = MR^2 * (dx_wheel / MR) * k_arb
            %           = MR * dx_wheel * k_arb
            dh_RH = 0.1234;
            F_wheel_f = 0.5 * this.vehicle.MR_arb_f * dh_RH * this.vehicle.karb_f;
            F_wheel_r = 0.5 * this.vehicle.MR_arb_r * dh_RH * this.vehicle.karb_r;

            % Compute damper positions from the perturbed ride height, then compute the
            % arb forces. FR wheel extends so force is -ve, RR wheel compresses so force is +ve.
            RH_ref = 0.08;
            [x_fl, x_fr, x_rl, x_rr] = this.vehicle.damperPosFromRideHeight(RH_ref, RH_ref + dh_RH, RH_ref, RH_ref - dh_RH);

            [FL, FR, RL, RR] = this.vehicle.arbWheelForces(x_fl, x_fr, x_rl, x_rr);

            this.verifyEqual(FL, F_wheel_f, 'AbsTol', 1e-6);
            this.verifyEqual(FR, -F_wheel_f, 'AbsTol', 1e-6);
            this.verifyEqual(RL, -F_wheel_r, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_wheel_r, 'AbsTol', 1e-6);
        end

        function arbWheelForcesAccountForRideAndDamperOffsets(this)
            % Select some non zero (unequal) damper positions, should produce non-zero
            % damper forces
            x = [0.111, 0.222, 0.333, 0.444];

            [FL, FR, RL, RR] = this.vehicle.arbWheelForces(x(1), x(2), x(3), x(4));
            this.assertGreaterThan(abs(FL), 0);
            this.assertGreaterThan(abs(FR), 0);
            this.assertGreaterThan(abs(RL), 0);
            this.assertGreaterThan(abs(RR), 0);

            % Set the offsets to those same values then re evaluate, should get zero forces
            this.vehicle.FL_corner.x0 = x(1);
            this.vehicle.FR_corner.x0 = x(2);
            this.vehicle.RL_corner.x0 = x(3);
            this.vehicle.RR_corner.x0 = x(4);

            [FL, FR, RL, RR] = this.vehicle.arbWheelForces(x(1), x(2), x(3), x(4));

            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function unsprungCentrifugalWheelLoadsZeroForZeroRotation(this)
            v = 100.0;
            w = 0.0;
            [FL, FR, RL, RR] = this.vehicle.unsprungCentrifugalWheelLoads(v, w, 0, 0, 0, 0);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function unsprungCentrifugalWheelLoadsMagnitude(this)
            v = 100.0;

            % Pitching down as if going over a crest of a hill
            w = -1.23;
            az = v * w;
            [FL, FR, RL, RR] = this.vehicle.unsprungCentrifugalWheelLoads(v, w, 0, 0, 0, 0);

            F_us_f = this.vehicle.m_unsprung_f * az;
            F_us_r = this.vehicle.m_unsprung_f * az;
            this.verifyEqual(FL, F_us_f / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FR, F_us_f / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RL, F_us_r / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_us_r / 2, 'AbsTol', 1e-6);

            % Pitching up as if going through a dip and compressing the suspension
            w = 4.56;
            az = v * w;
            [FL, FR, RL, RR] = this.vehicle.unsprungCentrifugalWheelLoads(v, w, 0, 0, 0, 0);

            F_us_f = this.vehicle.m_unsprung_f * az;
            F_us_r = this.vehicle.m_unsprung_f * az;
            this.verifyEqual(FL, F_us_f / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FR, F_us_f / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RL, F_us_r / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RR, F_us_r / 2, 'AbsTol', 1e-6);
        end

        function unsprungCentrifugalWheelLoadsAccountsForSuspensionMotion(this)
            % Compute a rate of motion for the dampers that will cancel out the motion
            % of the sprung mass. This should produce zero change in wheel loads.

            % Sprung mass pitching up, so make the suspension motion pitch down (front
            % extended, rear compressing)
            v = 100.0;
            w = 4.56;
            v_susp = 0.5 * this.vehicle.L * w;
            v_fl = this.vehicle.FL_corner.wheelToSpringMotion(v_susp);
            v_fr = this.vehicle.FR_corner.wheelToSpringMotion(v_susp);
            v_rl = this.vehicle.RL_corner.wheelToSpringMotion(-v_susp);
            v_rr = this.vehicle.RR_corner.wheelToSpringMotion(-v_susp);

            [FL, FR, RL, RR] = this.vehicle.unsprungCentrifugalWheelLoads(v, w, v_fl, v_fr, v_rl, v_rr);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function latWeightTransferSuspensionZeroForZeroAccelOrArmHeight(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % No acceleration, non zero distance between CoG and roll axis
            a = 0;
            this.vehicle.h_sprung = 0.5;
            this.vehicle.h_fuel = 0.5;
            this.vehicle.h_roll_f = @(RH) RH + 0.1;
            this.vehicle.h_roll_r = @(RH) RH + 0.1;

            [FL, FR, RL, RR] = this.vehicle.latWeightTransferSuspension(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Non zero acceleration, roll axis coincident with CoG
            a = 12.34;
            this.vehicle.h_roll_f = @(RH) RH + this.vehicle.h_sprung;
            this.vehicle.h_roll_r = @(RH) RH + this.vehicle.h_sprung;

            [FL, FR, RL, RR] = this.vehicle.latWeightTransferSuspension(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function latWeightTransferSuspensionShiftsWithRollDistribution(this)
            % Put all CoG's at vehicle mid point
            this.vehicle.l_sprung = 0.5 * this.vehicle.L;
            this.vehicle.l_fuel = 0.5 * this.vehicle.L;

            this.vehicle.h_roll_f = @(RH) RH + 0.1;
            this.vehicle.h_roll_r = @(RH) RH + 0.1;

            % Get a baseline value for weight transfer
            RH_f = 0.08;
            RH_r = 0.08;
            a = 12.34;

            [FL_ref, FR_ref, RL_ref, RR_ref] = this.vehicle.latWeightTransferSuspension(a, RH_f, RH_r);
            dW_total = abs(FL_ref) + abs(FR_ref) + abs(RL_ref) + abs(RR_ref);
            this.assertGreaterThan(abs(FL_ref), 0);
            this.assertGreaterThan(abs(FR_ref), 0);
            this.assertGreaterThan(abs(RL_ref), 0);
            this.assertGreaterThan(abs(RR_ref), 0);

            % Make motion ratios equal front to rear for easier adjustments
            this.vehicle.FL_corner.MR_spring = 1.2;
            this.vehicle.FR_corner.MR_spring = 1.2;
            this.vehicle.RL_corner.MR_spring = 1.2;
            this.vehicle.RR_corner.MR_spring = 1.2;
            this.vehicle.MR_arb_f = 1.5;
            this.vehicle.MR_arb_r = 1.5;

            % Record initial spring rates
            kspring_front_og = this.vehicle.FL_corner.kspring;
            kspring_rear_og = this.vehicle.RL_corner.kspring;
            karb_front_og = this.vehicle.karb_f;
            karb_rear_og = this.vehicle.karb_r;

            % Shift all the roll stiffness to the front axle
            this.vehicle.FL_corner.kspring = 1 / (1 / kspring_front_og + 1 / kspring_rear_og);
            this.vehicle.FR_corner.kspring = this.vehicle.FL_corner.kspring;
            this.vehicle.RL_corner.kspring = 0;
            this.vehicle.RR_corner.kspring = 0;
            this.vehicle.karb_f = 1 / (1 / karb_front_og + 1 / karb_rear_og);
            this.vehicle.karb_r = 0;

            [FL, FR, RL, RR] = this.vehicle.latWeightTransferSuspension(a, RH_f, RH_r);
            this.verifyEqual(FL, 0.5 * dW_total, 'AbsTol', 1e-6);
            this.verifyEqual(FR, -0.5 * dW_total, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Do the same for the rear
            this.vehicle.RL_corner.kspring = this.vehicle.FL_corner.kspring;
            this.vehicle.RR_corner.kspring = this.vehicle.FL_corner.kspring;
            this.vehicle.FL_corner.kspring = 0;
            this.vehicle.FR_corner.kspring = 0;
            this.vehicle.karb_r = this.vehicle.karb_f;
            this.vehicle.karb_f = 0;

            [FL, FR, RL, RR] = this.vehicle.latWeightTransferSuspension(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0.5 * dW_total, 'AbsTol', 1e-6);
            this.verifyEqual(RR, -0.5 * dW_total, 'AbsTol', 1e-6);
        end

        function latWeightTransferSuspensionMagnitude(this)
            % Make all spring rates symmetric front to rear and CoG position at mid point
            % to create a simple configuration to manually calculate
            RH_f = 0;
            RH_r = 0;

            this.vehicle.FR_corner = this.vehicle.FL_corner;
            this.vehicle.RL_corner = this.vehicle.FL_corner;
            this.vehicle.RR_corner = this.vehicle.FL_corner;
            this.vehicle.MR_arb_f = this.vehicle.MR_arb_r;
            this.vehicle.karb_f = this.vehicle.karb_r;

            this.vehicle.h_sprung = 0.5;
            this.vehicle.h_fuel = this.vehicle.h_sprung;
            this.vehicle.l_sprung = 0.5 * this.vehicle.L;
            this.vehicle.l_fuel = 0.5 * this.vehicle.L;

            % Make roll axis half the CoG height, so the sprung weight transfer will be
            % half of the total weight transfer
            this.vehicle.h_roll_f = @(RH) RH + 0.5 * this.vehicle.h_sprung;
            this.vehicle.h_roll_r = @(RH) RH + 0.5 * this.vehicle.h_sprung;

            % Total weight transfer
            a = 12.34;
            WT_total = this.vehicle.sprungMass() * a * ...
                (RH_f + this.vehicle.h_sprung) / this.vehicle.track_f;

            % Sprung weight transfer should be half of the total, evenly split
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferSuspension(a, RH_f, RH_r);

            this.verifyEqual(abs(FL) + abs(FR) + abs(RL) + abs(RR), WT_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FL, WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(FR, -WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(RL, WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(RR, -WT_total / 8, 'AbsTol', 1e-6);
        end

        function longWeightTransferSuspensionZeroForZeroAccel(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % Zero acceleration, non zero anti
            a = 0;
            this.vehicle.SVIC_ang_f = 0.1;
            this.vehicle.SVIC_ang_r = 0.1;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferSuspension(a, RH_f, RH_r);

            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function longWeightTransferSuspensionForZeroAnti(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % With no anti effects the sprung mass weight transfer should be entirely
            % transmitted through the springs. With equal brake bias the weight
            % transfer will be equally split front to rear
            a = 100;
            this.vehicle.SVIC_ang_f = 0.0;
            this.vehicle.SVIC_ang_r = 0.0;

            dW_total = this.vehicle.sprungMass() * a * ...
                this.vehicle.totalCogHeight(RH_f, RH_r) / this.vehicle.L;

            [FL, FR, RL, RR] = this.vehicle.longWeightTransferSuspension(a, RH_f, RH_r);

            this.verifyEqual(FL, -dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FR, -dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RL, dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RR, dW_total / 2, 'AbsTol', 1e-6);
        end

        function longWeightTransferSuspensionScalesWithBrakeBias(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % Decelerate
            a = -12.34;

            % Set some non zero anti
            this.vehicle.SVIC_ang_f = deg2rad(10.0);
            this.vehicle.SVIC_ang_r = deg2rad(10.0);

            % Total expected weight transfer due to sprung mass
            dW_total = this.vehicle.sprungMass() * a * ...
                this.vehicle.totalCogHeight(RH_f, RH_r) / this.vehicle.L;

            % Full front brake bias should result in all rear weight transfer being
            % transmitted through the springs
            this.vehicle.brake_bias = 1.0;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferSuspension(a, RH_f, RH_r);
            this.verifyGreaterThan(FL, 0);
            this.verifyGreaterThan(FR, 0);
            this.verifyEqual(RL, dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RR, dW_total / 2, 'AbsTol', 1e-6);

            % Opposite for rear
            this.vehicle.brake_bias = 0.0;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferSuspension(a, RH_f, RH_r);
            this.verifyEqual(FL, -dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FR, -dW_total / 2, 'AbsTol', 1e-6);
            this.verifyLessThan(RL, 0);
            this.verifyLessThan(RR, 0);
        end

        function latWeightTransferGeometricZeroForZeroAccelOrRcHeight(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % No acceleration, non zero RC height
            a = 0;
            this.vehicle.h_roll_f = @(RH) 0.20;
            this.vehicle.h_roll_r = @(RH) 0.40;
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Acceleration, but zero RC height one axle at a time
            a = 100;

            this.vehicle.h_roll_f = @(RH) 0;
            this.vehicle.h_roll_r = @(RH) 0.2;
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyGreaterThan(abs(RL), 0);
            this.verifyGreaterThan(abs(RR), 0);

            this.vehicle.h_roll_f = @(RH) 0.2;
            this.vehicle.h_roll_r = @(RH) 0;
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyGreaterThan(abs(FL), 0);
            this.verifyGreaterThan(abs(FR), 0);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function latWeightTransferGeometricProportionalForLongCogPosition(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % Set the front and rear RC heights to be equal so that shifting the CoG
            % positions longitudinally results in equal changes in the front and rear
            this.vehicle.h_roll_f = @(RH) 0.3;
            this.vehicle.h_roll_r = @(RH) 0.3;

            % Compute the weight transfer for some non-zero acceleration
            a = 12.34;
            [FL_ref, FR_ref, RL_ref, RR_ref] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.assertGreaterThan(abs(FL_ref), 0);
            this.assertGreaterThan(abs(FR_ref), 0);
            this.assertGreaterThan(abs(RL_ref), 0);
            this.assertGreaterThan(abs(RR_ref), 0);

            % Shift all sprung mass CoG positions to front axle, all geometric weight
            % transfer should be on the front axle only
            this.vehicle.l_sprung = 0;
            this.vehicle.l_fuel = 0;
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyEqual(FL, FL_ref + RL_ref, 'AbsTol', 1e-6);
            this.verifyEqual(FR, FR_ref + RR_ref, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Do the same for the rear axle
            this.vehicle.l_sprung = this.vehicle.L;
            this.vehicle.l_fuel = this.vehicle.L;
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, FL_ref + RL_ref, 'AbsTol', 1e-6);
            this.verifyEqual(RR, FR_ref + RR_ref, 'AbsTol', 1e-6);
        end

        function latWeightTransferGeometricMagnitude(this)
            % Put front and rear roll centres at equal heights, half way between ground
            % and CoG height, put sprung mass CoG at wheelbase mid point, and make the
            % track widths equal. We can easily calculate what the weight transfer should
            % be in this case.
            RH_f = 0;
            RH_r = 0;

            this.vehicle.l_sprung = 0.5 * this.vehicle.L;
            this.vehicle.l_fuel = 0.5 * this.vehicle.L;
            this.vehicle.h_sprung = 0.8;
            this.vehicle.h_fuel = this.vehicle.h_sprung;
            h_roll = 0.5 * this.vehicle.h_sprung;
            this.vehicle.h_roll_f = @(RH) RH + h_roll;
            this.vehicle.h_roll_r = @(RH) RH + h_roll;
            this.vehicle.track_r = this.vehicle.track_f;

            % Total weight transfer
            a = 12.34;
            WT_total = this.vehicle.sprungMass() * a * (RH_f + this.vehicle.h_sprung) / this.vehicle.track_f;

            % Net geometric weight transfer should be half the total, evenly split
            % across all wheels
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyEqual(abs(FL) + abs(FR) + abs(RL) + abs(RR), WT_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FL, WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(FR, -WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(RL, WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(RR, -WT_total / 8, 'AbsTol', 1e-6);

            % Now shift the front and rear RC heights up and down by some %. The total
            % weight transfer should be the same but the distribution should scale with
            % the perturbation
            dh = 0.2 * h_roll;
            this.vehicle.h_roll_f = @(RH) (h_roll - dh);
            this.vehicle.h_roll_r = @(RH) (h_roll + dh);
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferGeometric(a, RH_f, RH_r);

            this.verifyEqual(abs(FL) + abs(FR) + abs(RL) + abs(RR), WT_total / 2, 'AbsTol', 1e-6);

            front_factor = (h_roll - dh) / h_roll;
            rear_factor = (h_roll + dh) / h_roll;

            this.verifyEqual(FL, front_factor * WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(FR, front_factor * -WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(RL, rear_factor * WT_total / 8, 'AbsTol', 1e-6);
            this.verifyEqual(RR, rear_factor * -WT_total / 8, 'AbsTol', 1e-6);
        end

        function longWeightTransferGeometricZeroForZeroAccelOrAnti(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % Zero acceleration, non zero anti
            a = 0;
            this.vehicle.SVIC_ang_f = 0.1;
            this.vehicle.SVIC_ang_r = 0.1;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferGeometric(a, RH_f, RH_r);

            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Non zero acceleration, zero anti
            a = 100;
            this.vehicle.SVIC_ang_f = 0.0;
            this.vehicle.SVIC_ang_r = 0.0;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferGeometric(a, RH_f, RH_r);

            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function longWeightTransferGeometricScalesWithBrakeBias(this)
            RH_f = 0.08;
            RH_r = 0.10;

            % Decelerate
            a = -12.34;

            % Set some non zero anti
            this.vehicle.SVIC_ang_f = deg2rad(10.0);
            this.vehicle.SVIC_ang_r = deg2rad(10.0);

            % Full front brake bias should have zero rear geometric weight transfer
            this.vehicle.brake_bias = 1.0;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyGreaterThan(FL, 0);
            this.verifyGreaterThan(FR, 0);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            % Opposite for rear
            this.vehicle.brake_bias = 0.0;
            [FL, FR, RL, RR] = this.vehicle.longWeightTransferGeometric(a, RH_f, RH_r);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyLessThan(RL, 0);
            this.verifyLessThan(RR, 0);
        end

        function latWeightTransferUnsprungZeroForZeroAccelOrHeight(this)
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferUnsprung(0);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);

            this.vehicle.h_unsprung_f = 0.0;
            this.vehicle.h_unsprung_r = 0.0;

            [FL, FR, RL, RR] = this.vehicle.latWeightTransferUnsprung(100);
            this.verifyEqual(FL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RR, 0, 'AbsTol', 1e-6);
        end

        function latWeightTransferUnsprungMagnitude(this)
            [FL, FR, RL, RR] = this.vehicle.latWeightTransferUnsprung(1.23);

            % Everything should be non zero
            this.assertGreaterThan(abs(FL), 0);
            this.assertGreaterThan(abs(FR), 0);
            this.assertGreaterThan(abs(RL), 0);
            this.assertGreaterThan(abs(RR), 0);

            % Sum on axles should be zero
            this.verifyEqual(FL + FR, 0, 'AbsTol', 1e-6);
            this.verifyEqual(RL + RR, 0, 'AbsTol', 1e-6);

            % For right hand turn (a > 0) left side weight should increase
            this.verifyGreaterThan(FL, FR);
            this.verifyGreaterThan(RL, RR);
        end

        function longWeightTransferGeometricAndSuspensionTotal(this)
            % The sum of the load transfer through the springs and the suspension geometry
            % should match the load transfer computed from the sprung mass and CoG height
            RH_f = 0.08;
            RH_r = 0.10;

            % Decelerate
            a = -12.34;

            % Set some non zero anti, and unequal brake bias
            this.vehicle.SVIC_ang_f = deg2rad(15.0);
            this.vehicle.SVIC_ang_r = deg2rad(10.0);
            this.vehicle.brake_bias = 0.63;

            % Compute the total expected load transfer of the sprung mass
            dW_total = this.vehicle.sprungMass() * a * ...
                this.vehicle.totalCogHeight(RH_f, RH_r) / this.vehicle.L;

            % Full front brake bias should have zero rear geometric weight transfer
            [FL_geo, FR_geo, RL_geo, RR_geo] = this.vehicle.longWeightTransferGeometric(a, RH_f, RH_r);
            [FL_susp, FR_susp, RL_susp, RR_susp] = this.vehicle.longWeightTransferSuspension(a, RH_f, RH_r);

            this.verifyEqual(FL_geo + FL_susp, -dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(FR_geo + FR_susp, -dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RL_geo + RL_susp, dW_total / 2, 'AbsTol', 1e-6);
            this.verifyEqual(RR_geo + RR_susp, dW_total / 2, 'AbsTol', 1e-6);
        end

        function totalWheelLoads(this)
            % For all test cases we'll use the same damper positions. In reality under
            % some acceleration there would be some change in damper positions/ride
            % height, however calculating what that ride height should be involves
            % the same calculations we're testing. So instead we'll rely on some
            % comparisons we'll know should be true. We'll also always use zero damper
            % velocity to ensure we can properly predict the outcomes.
            v = 34.5;
            w = 0;
            V_f = 60.0;
            RH_f = 0.08;
            RH_r = 0.08;
            this.vehicle.SVIC_ang_f = deg2rad(6);
            this.vehicle.SVIC_ang_r = deg2rad(3);

            % We'll need to set and compute some damper properties to make sure the damper
            % positions and ride heights we provide result in proper spring forces

            % Set origin for all dampers to be a little higher than the input ride height
            this.vehicle = this.vehicle.setReferenceRideHeight(RH_f + 0.05, RH_r + 0.05, 0, 0, 0, 0);

            % Compute the spring perch settings to make the shock forces match the weight
            [FL_static, FR_static, RL_static, RR_static] = this.vehicle.staticWheelLoads();
            [x_fl, x_fr, x_rl, x_rr] = this.vehicle.damperPosFromAvgRideHeight(RH_f, RH_r);
            this.vehicle = this.vehicle.setSpringPerchFromWheelLoads(x_fl, x_fr, x_rl, x_rr, ...
                FL_static, FR_static, RL_static, RR_static);

            % Zero acceleration should result in the static wheel loads
            a = [0; 0; -this.vehicle.gravity];
            [FL, FR, RL, RR] = this.vehicle.totalWheelLoads(v, a, w, V_f, ...
                RH_f, RH_f, RH_r, RH_r, ...
                x_fl, x_fr, x_rl, x_rr, 0, 0, 0, 0);

            this.verifyEqual(FL.total, FL_static, 'AbsTol', 1e-3);
            this.verifyEqual(FR.total, FR_static, 'AbsTol', 1e-3);
            this.verifyEqual(RL.total, RL_static, 'AbsTol', 1e-3);
            this.verifyEqual(RR.total, RR_static, 'AbsTol', 1e-3);

            % Decelerating should increase front wheel loads and reduce the rear
            a = [-12.34; 0; -this.vehicle.gravity];
            [FL, FR, RL, RR] = this.vehicle.totalWheelLoads(v, a, w, V_f, ...
                RH_f, RH_f, RH_r, RH_r, ...
                x_fl, x_fr, x_rl, x_rr, 0, 0, 0, 0);

            this.verifyGreaterThan(FL.total, FL_static);
            this.verifyGreaterThan(FR.total, FR_static);
            this.verifyLessThan(RL.total, RL_static);
            this.verifyLessThan(RR.total, RR_static);

            % Turning right should increase the left wheel loads and decrease the right
            a = [0; 3.45; -this.vehicle.gravity];
            [FL, FR, RL, RR] = this.vehicle.totalWheelLoads(v, a, w, V_f, ...
                RH_f, RH_f, RH_r, RH_r, ...
                x_fl, x_fr, x_rl, x_rr, 0, 0, 0, 0);

            this.verifyGreaterThan(FL.total, FL_static);
            this.verifyLessThan(FR.total, FR_static);
            this.verifyGreaterThan(RL.total, RL_static);
            this.verifyLessThan(RR.total, RR_static);

            % Combined lateral and longitudinal acceleration (forward and left)
            a = [12.34; -3.45; -this.vehicle.gravity];
            [FL, FR, RL, RR] = this.vehicle.totalWheelLoads(v, a, w, V_f, ...
                RH_f, RH_f, RH_r, RH_r, ...
                x_fl, x_fr, x_rl, x_rr, 0, 0, 0, 0);

            this.verifyLessThan(FL.total, FL_static);
            this.verifyGreaterThan(FR.total, FL.total);
            this.verifyLessThan(RL.total, RR.total);
            this.verifyGreaterThan(RR.total, RR_static);
        end

    end
end
