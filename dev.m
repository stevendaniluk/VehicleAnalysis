clear;

% Define the regions of the track where we want to collect data
data_zones = {};
data_zones{1} = [590, 1290]; % Before T3, to before T5
% data_zones{2} = [1755, 1900]; % Between T5 and T6
% data_zones{2} = [2055, 2175]; % Between T6 and T7
% data_zones{2} = [2055, 2175]; % Between T6 and T7
% data_zones{3} = [3040, inf]; % T11 to finish

% % Session 1 data
% filename = 'exige_session_1.mat';
% rho_air = 1.173;
% V_i = 42.0;
% V_f = 31.2;
% ballast = 0;
%
% t_trim = [201, 1077]; % All flying laps

% Session 2 data
filename = 'exige_session_2.mat';
rho_air = 1.136;
V_i = 31.2;
V_f = 24.2;
ballast = 0;

% t_trim = [66.0, 976.0]; % Trim entry/exit
% t_trim = [185, 842]; % All flying laps
% t_trim = [0, 70]; % Track entry
t_trim = [292, 414]; % Full fast lap
% t_trim = [292, 519]; % Two fast laps
% t_trim = [292, 310]; % Front straight
% t_trim = [390, 403]; % T11
% t_trim = [390, 423]; % T11 - T2
% t_trim = [317, 353]; % T3-T5
% t_trim = [325, 330]; % Curbs, exit of T3
% t_trim = [364, 381]; % Corkscrew
% t_trim = [80, 100]; % Clutch press
% t_trim = [650, 700]; % Clutch press
% t_trim = [800, 960]; % Clutch press

% % Session 3 data
% filename = 'exige_session_3.mat';
% rho_air = 1.132;
% V_i = 24.3;
% V_f = 14.8;
% % ballast = 100;
% ballast = 0;
% 
% % t_trim = [0, 170]; % Entry
% % t_trim = [170, 1194]; % Trim entry/exit
% % t_trim = [285, 1048]; % All flying laps
% t_trim = [390, 520]; % Lap

% % Session 4 data
% filename = 'exige_session_4.mat';
% rho_air = 1.126;
% V_i = 34.5;
% V_f = 18.3;
% ballast = 0;

% t_trim = [106, 2070]; % Trim entry/exit
% t_trim = [333, 447]; % Lap 4
% t_trim = [322, 350]; % Finish straight

handler = MotecHandlerExige(filename);

% Compute and set the fuel level
t = handler.getTimestamps();
v_ground = handler.getChannel('GROUND_SPEED');
V_fuel_full = fuelLevelDuringRun(V_i, V_f, t, v_ground);
handler = handler.addChannelData('FUEL_LEVEL', V_fuel_full, 'L');

handler = handler.trimTimeRange(t_trim(1), t_trim(2));

exige = createExige();

exige.gravity = 9.92;
g = exige.gravity;

t = handler.getTimestamps();
dt = t(2) - t(1);
t_min = t(1);
t_max = t(end);

% Fetch all the data channels
lap_distance = handler.getChannel('LAP_DISTANCE');
V_fuel = handler.getChannel('FUEL_LEVEL');
throttle = handler.getChannel('THROTTLE');
brake = handler.getChannel('BRAKE');
clutch = handler.getChannel('CLUTCH');
steer = handler.getChannel('STEERING_WHEEL');
v_ground = handler.getChannel('GROUND_SPEED');
P_air = handler.getChannel('AIR_DYN_PRESSURE');
elevation = handler.getChannelNoRemap('GPS_Altitude');

% Lightly smooth ground speed and pressure for air speed
n_v_ground = 0.2 / dt + 1;
v_ground = smoothdata(v_ground, 'movmean', n_v_ground);

dt_shift_v_ground = -0.5;
v_ground = handler.shiftData(v_ground, dt, dt_shift_v_ground);

n_P_air = 0.5 / dt + 1;
P_air = smoothdata(P_air, 'movmean', n_P_air);

v_air = sqrt(2 * P_air / rho_air);

dt_shift_v_air = -0.3;
v_air = handler.shiftData(v_air, dt, dt_shift_v_air);

% Grade angle computed from GNSS elevation (heavily smooth both elevation and
% groundn speed for angle computation)
n_elevation = 3.0 / dt + 1;
dt_shift_elevation = -0.5;
elevation = smoothdata(elevation, 'movmean', n_elevation);
elevation = handler.shiftData(elevation, dt, dt_shift_elevation);

Vz = -diff(elevation) / dt;
Vz = [Vz, Vz(end)];
n_Vz = 2.5 / dt + 1;
Vz = smoothdata(Vz, 'movmean', n_Vz);

az_elevation = diff(Vz) / dt;
az_elevation = [az_elevation, az_elevation(end)];
n_az_elevation = 1.0 / dt + 1;
az_elevation = smoothdata(az_elevation, 'movmean', n_az_elevation);

n_v_ground = 1.0 / dt + 1;
v_ground_smooth = smoothdata(v_ground, 'movmean', n_v_ground);

road_grade = atan2(Vz, v_ground_smooth);

grade_v_min = 20;
road_grade(v_ground_smooth < grade_v_min) = 0;

% For shocks compute the velocity from the raw data, then apply any smoothing to
% the position and velocity data individually
n_damp_p = 1.5 / dt + 1;
n_damp_pv = 1.0 / dt + 1;
n_damp_v = 1.0 / dt + 1;
n_damp_a = 1.0 / dt + 1;
dt_shift_damp_p_f = -0.1;
dt_shift_damp_p_r = -0.1;
dt_shift_damp_v_f = 0.3;
dt_shift_damp_v_r = 0.2;

x_fl = handler.getChannel('DAMPER_FL');
x_fl = handler.shiftData(x_fl, dt, dt_shift_damp_p_f);

xv_fl = smoothdata(x_fl, 'movmean', n_damp_pv);
v_fl = [diff(xv_fl), xv_fl(end) - xv_fl(end - 1)] / dt;
v_fl = handler.shiftData(v_fl, dt, dt_shift_damp_v_f);

x_fl = smoothdata(x_fl, 'movmean', n_damp_p);
v_fl = smoothdata(v_fl, 'movmean', n_damp_v);

x_fr = handler.getChannel('DAMPER_FR');
x_fr = handler.shiftData(x_fr, dt, dt_shift_damp_p_f);

xv_fr = smoothdata(x_fr, 'movmean', n_damp_pv);
v_fr = [diff(xv_fr), xv_fr(end) - xv_fr(end - 1)] / dt;
v_fr = handler.shiftData(v_fr, dt, dt_shift_damp_v_f);

x_fr = smoothdata(x_fr, 'movmean', n_damp_p);
v_fr = smoothdata(v_fr, 'movmean', n_damp_v);

x_rl = handler.getChannel('DAMPER_RL');
x_rl = handler.shiftData(x_rl, dt, dt_shift_damp_p_r);

xv_rl = smoothdata(x_rl, 'movmean', n_damp_pv);
v_rl = [diff(xv_rl), xv_rl(end) - xv_rl(end - 1)] / dt;
v_rl = handler.shiftData(v_rl, dt, dt_shift_damp_v_r);

x_rl = smoothdata(x_rl, 'movmean', n_damp_p);
v_rl = smoothdata(v_rl, 'movmean', n_damp_v);

x_rr = handler.getChannel('DAMPER_RR');
x_rr = handler.shiftData(x_rr, dt, dt_shift_damp_p_r);

xv_rr = smoothdata(x_rr, 'movmean', n_damp_pv);
v_rr = [diff(xv_rr), xv_rr(end) - xv_rr(end - 1)] / dt;
v_rr = handler.shiftData(v_rr, dt, dt_shift_damp_v_r);

x_rr = smoothdata(x_rr, 'movmean', n_damp_p);
v_rr = smoothdata(v_rr, 'movmean', n_damp_v);

% Pitch motion
wy_unsmooth = handler.getChannel('WY');

n_wy = 2.0 / dt + 1;
handler = handler.smoothChannel('WY', n_wy, 'movmean');
wy = handler.getChannel('WY');

% Adjust IMU pitch rate to account for suspension pitch motion
[RH_f, RH_r] = exige.avgRideHeightFromDamperPos(x_fl, x_fr, x_rl, x_rr);
pitch_susp = exige.pitchFromAvgRideHeights(RH_f, RH_r);

n_pitch_susp = 0.5 / dt + 1;
pitch_susp = smoothdata(pitch_susp, 'movmean', n_pitch_susp);

wy_susp = diff(pitch_susp) / dt;
wy_susp = [wy_susp, wy_susp(end)];

wy_uncorrected = wy;
wy = wy - wy_susp;

wy_dot = diff(wy) / dt;
wy_dot = [wy_dot, wy_dot(end)];

n_wy_dot = 0.5 / dt + 1;
wy_dot =  smoothdata(wy_dot, 'movmean', n_wy_dot);

% Linear acceleration
n_axy = 0.8 / dt + 1;

handler = handler.smoothChannel('AX', n_axy, 'movmean');
ax = handler.getChannel('AX');

handler = handler.smoothChannel('AY', n_axy, 'movmean');
ay = handler.getChannel('AY');

% Compute vertical acceleration from gyro data
% az = -wy .* v_ground;
az = az_elevation;

% Add in gravity component, accounting for estimated road grade angle
g_road = g * cos(road_grade);
az = az - g_road;

n_az = 1.0 / dt + 1;
az =  smoothdata(az, 'movmean', n_az);

n_wy_heavy = 1.0 / dt + 1;
wy_heavy_mean =  smoothdata(wy_unsmooth, 'movmean', n_wy_heavy);
az_heavy_mean = -wy_heavy_mean .* v_ground - g;

n_wy_heavy = 1.0 / dt + 1;
wy_heavy_med =  smoothdata(wy_unsmooth, 'movmedian', n_wy_heavy);
az_heavy_med = -wy_heavy_med .* v_ground - g;

% Found regions of the track where we want to process data
zone_pts = false(size(t));
for i=1:length(data_zones)
    d = data_zones{i};
    zone_pts_new = handler.thresholdIndices(lap_distance, dt, d(1), d(2), -inf, inf);
    zone_pts = or(zone_pts_new, zone_pts);
end

% Find periods of sufficient speed
v_min = 80 / 3.6;
v_min_reached_pts = v_ground >= v_min;

% Find periods when the clutch has been depressed for at least some duration,
% then trim either side of those by some buffer
dt_clutch = 2.0;
clutch_buff = 0.5;

clutch_depressed_pts = clutch == 100;
clutch_depressed_pts = handler.detectConditionPeriod(clutch_depressed_pts, dt, dt_clutch);
clutch_depressed_pts = handler.trimLogicalEdges(clutch_depressed_pts, dt, clutch_buff);

% Find when the brakes are not applied
brake_lim = 20.0;
brake_trim_dt = 0.2;
brakes_released_pts = handler.thresholdIndices(brake, dt, 0, brake_lim, ...
    -inf, inf);
brakes_released_pts = handler.trimLogicalEdges(brakes_released_pts, dt, brake_trim_dt);

% Find period of low steering angle input
wheel_angle = steer / exige.steer_ratio;
wheel_angle_lim = deg2rad(5.0);
wheel_angle_pts = handler.thresholdIndices(wheel_angle, dt, -wheel_angle_lim, wheel_angle_lim, ...
    -inf, inf);

% Find when longitudinal acceleration is low
ax_lim = 3.0;
ax_trim_dt = 0.4;

ax_low_pts = handler.thresholdIndices(ax, dt, -ax_lim, ax_lim, -inf, inf);
ax_low_pts = handler.trimLogicalEdges(ax_low_pts, dt, ax_trim_dt);

% Find when lateral acceleration is low
ay_lim = 6.0;
ay_trim_dt = 0.4;

ay_low_pts = handler.thresholdIndices(ay, dt, -ay_lim, ay_lim, -inf, inf);
ay_low_pts = handler.trimLogicalEdges(ay_low_pts, dt, ay_trim_dt);

% Find when lateral acceleration is low
az_lim = 10.0;
az_trim_dt = 0.4;

az_low_pts = handler.thresholdIndices(az + g, dt, -az_lim, az_lim, -inf, inf);
az_low_pts = handler.trimLogicalEdges(az_low_pts, dt, az_trim_dt);

% Find when suspension pitch rate is low
wy_susp_min = deg2rad(-1.0);
wy_susp_max = deg2rad(0.08);
wy_susp_trim_dt = 1.0;

wy_susp_low_pts = handler.thresholdIndices(wy_susp, dt, wy_susp_min, wy_susp_max, -inf, inf);
wy_susp_low_pts = handler.trimLogicalEdges(wy_susp_low_pts, dt, wy_susp_trim_dt);

% Trim based on pitch rate
wy_lim = deg2rad(4.0);
wy_trim_dt = 0.2;
wy_low_pts = handler.thresholdIndices(wy, dt, -wy_lim, wy_lim, ...
    -inf, inf);
wy_low_pts = handler.trimLogicalEdges(wy_low_pts, dt, wy_trim_dt);

% Trim based on pitch acceleration
wy_dot_lim = deg2rad(5.0);
wy_trim_dt = 0.2;
wy_dot_low_pts = handler.thresholdIndices(wy_dot, dt, -wy_dot_lim, wy_dot_lim, ...
    -inf, inf);
wy_dot_low_pts = handler.trimLogicalEdges(wy_dot_low_pts, dt, wy_trim_dt);

valid_data_pts = true(size(t));
valid_data_pts = and(zone_pts, valid_data_pts);
% valid_data_pts = and(v_min_reached_pts, valid_data_pts);
% valid_data_pts = and(clutch_depressed_pts, valid_data_pts);
% valid_data_pts = and(brakes_released_pts, valid_data_pts);
valid_data_pts = and(wheel_angle_pts, valid_data_pts);
valid_data_pts = and(wy_susp_low_pts, valid_data_pts);
% valid_data_pts = and(ax_low_pts, valid_data_pts);
% valid_data_pts = and(ay_low_pts, valid_data_pts);
% valid_data_pts = and(az_low_pts, valid_data_pts);
% valid_data_pts = and(wy_low_pts, valid_data_pts);
% valid_data_pts = and(wy_dot_low_pts, valid_data_pts);
% Plot input data
figure(1);
clf;

n = 0;
n_max = 4;

% Ground speed
n = n + 1;
subplot(n_max, 1, n);
hold on;
plot(t, 3.6 * v_ground, '-b', 'DisplayName', 'Ground');
plot(t, 3.6 * v_air, '--m', 'DisplayName', 'Air');
plot(t(valid_data_pts), 3.6 * v_ground(valid_data_pts), '.r', 'DisplayName', 'Valid');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Speed [km/h]');
ylim([0, 180]);

% Pedals
n = n + 1;
subplot(n_max, 1, n);
hold on
plot(t, throttle, 'b', 'DisplayName', 'Throttle');
plot(t, brake, 'r', 'DisplayName', 'Brake');
plot(t, clutch, 'y', 'DisplayName', 'Clutch');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Pedals [%]');

% % Steering
% n = n + 1;
% subplot(n_max, 1, n);
% plot(t, rad2deg(steer), '');
% grid on;
% xlabel('Time [s]');
% ylabel('Steering Wheel [deg]');

% % Elevation
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% plot(t, elevation, '-b', 'DisplayName', 'Z');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Elevation [m]');

% % Grade angle
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% % plot(t, rad2deg(road_grade), '-b', 'DisplayName', 'Z');
% plot(t, Vz, '-b', 'DisplayName', 'Vz');
% plot(t, az_elevation, '-r', 'DisplayName', 'az');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Grade Angle [deg]');

% % Pitch rate
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% plot(t, rad2deg(wy_uncorrected), '--k', 'DisplayName', 'OG');
% plot(t, rad2deg(wy), '-b', 'DisplayName', 'Wy');
% plot(t, rad2deg(wy_susp), '-r', 'DisplayName', 'Susp');
% % plot(t, rad2deg(wy_heavy_mean), 'r', 'DisplayName', 'Hvy Mean');
% % plot(t, rad2deg(wy_heavy_med), 'g', 'DisplayName', 'Hvy Med');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Pitch Rate [deg/s]');
% ylim([-1, 1]);

% % Pitch  acceleration
% n = n + 1;
% subplot(n_max, 1, n);
% hold on;
% plot(t, rad2deg(wy_dot), '-k');
% grid on;
% xlabel('Time [s]');
% ylabel('Pitch Accel [deg/s^2]');
% ylim([-3, 3]);

% Linear Acceleration
n = n + 1;
subplot(n_max, 1, n);
hold on
plot(t, ax, 'g', 'DisplayName', 'X');
plot(t, ay, 'b', 'DisplayName', 'Y');
plot(t, (az + g), '-k', 'DisplayName', 'Z');
plot(t, az_elevation, '-r', 'DisplayName', 'Z elev');
% plot(t, (az_heavy_mean + g), '-r', 'DisplayName', 'Z-Mean');
% plot(t, (az_heavy_med + g), '-g', 'DisplayName', 'Z-Med');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
ylim([-1, 1]);

% Damper positions
n = n + 1;
subplot(n_max, 1, n);
hold on
plot(t, 1e3 * x_fl, 'b', 'DisplayName', 'FL');
plot(t, 1e3 * x_fr, 'g', 'DisplayName', 'FR');
plot(t, 1e3 * x_rl, 'r', 'DisplayName', 'RL');
plot(t, 1e3 * x_rr, 'y', 'DisplayName', 'RR');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Damper Position [mm]');
ylim([0, 80]);

% % Damper velocities
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% plot(t, 1e3 * v_fl, 'b', 'DisplayName', 'FL');
% plot(t, 1e3 * v_fr, 'g', 'DisplayName', 'FR');
% plot(t, 1e3 * v_rl, 'r', 'DisplayName', 'RL');
% plot(t, 1e3 * v_rr, 'y', 'DisplayName', 'RR');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Damper Velocity [mm/s]');
% ylim([-50, 50]);

%% Computed data

% Compute ride height from damper positions and tire radius
[RH_fl, RH_fr, RH_rl, RH_rr] = exige.rideHeightFromDamperPos(x_fl, x_fr, x_rl, x_rr);

% Static wheel loads
[FL_static, FR_static, RL_static, RR_static] = exige.staticWheelLoads(V_i);
F_static = (FL_static + FR_static) / 2;
R_static = (RL_static + RR_static) / 2;

% Concatenate accelerations
a = [ax; ay; az];

% Estimate individual wheel loads
[FL, FR, RL, RR] = exige.totalWheelLoads(v_ground, a, wy, V_fuel, ...
            RH_fl, RH_fr, RH_rl, RH_rr, ...
            x_fl, x_fr, x_rl, x_rr, ...
            v_fl, v_fr, v_rl, v_rr);

% Estimate aero loads
[FL_aero, FR_aero, RL_aero, RR_aero] = exige.estimatedAeroWheelLoad(a, V_fuel, ...
            RH_fl, RH_fr, RH_rl, RH_rr, ...
            x_fl, x_fr, x_rl, x_rr, ...
            v_fl, v_fr, v_rl, v_rr);

F_aero_front = FL_aero.aero + FR_aero.aero;
F_aero_rear = RL_aero.aero + RR_aero.aero;
F_aero = F_aero_front + F_aero_rear;

n_aero = 0.5 / dt + 1;
F_aero_front_smooth = smoothdata(F_aero_front, 'movmean', n_aero);
F_aero_smooth = smoothdata(F_aero, 'movmean', n_aero);

aero_bal = F_aero_front_smooth ./ F_aero_smooth;

% Account for ballast
F_aero = F_aero - ballast / 0.2248;

Cl = exige.aeroForceCoefficient(v_air, F_aero, rho_air);

% Get all indices where the estimated downforce is negative and we're not
% pruning the data
neg_DF_indices = F_aero < 0;
neg_DF_indices = and(neg_DF_indices, valid_data_pts);

low_DF_indices = and(F_aero_front > 100, F_aero_rear > 100);
low_DF_indices = and(low_DF_indices, valid_data_pts);

%% Aero plots
figure(2);
clf;

n_max = 5;
n = 0;

% Ground speed
n = n + 1;
subplot(n_max, 1, n);
hold on;
plot(t, 3.6 * v_ground, '-b', 'DisplayName', 'Ground');
plot(t, 3.6 * v_air, '--m', 'DisplayName', 'Air');
plot(t(valid_data_pts), 3.6 * v_ground(valid_data_pts), '.r', 'DisplayName', 'Valid');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Speed [km/h]');

% % Pedals
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% plot(t, throttle, 'b', 'DisplayName', 'Throttle');
% plot(t, brake, 'r', 'DisplayName', 'Brake');
% % plot(t, clutch, 'y', 'DisplayName', 'Clutch');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Pedals [%]');

% Linear Acceleration
n = n + 1;
subplot(n_max, 1, n);
hold on
plot(t, ax, 'g', 'DisplayName', 'X');
plot(t, ay, 'b', 'DisplayName', 'Y');
plot(t, (az + g), 'k', 'DisplayName', 'Z');
% plot(t, (az_heavy_mean + g), 'k', 'DisplayName', 'Z-Mean');
% plot(t, (az_heavy_med + g), 'k', 'DisplayName', 'Z-Med');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
ylim([-5, 5]);

% Downforce
n = n + 1;
subplot(n_max, 1, n);
hold on;
plot(t, 0.2248 * F_aero, 'b', 'DisplayName', 'Total');
% plot(t(~brakes_released_pts), 0.2248 * F_aero(~brakes_released_pts), '.r', 'DisplayName', 'Brakes');
plot(t(~wheel_angle_pts), 0.2248 * F_aero(~wheel_angle_pts), '.g', 'DisplayName', 'Steer');
plot(t(~wy_susp_low_pts), 0.2248 * F_aero(~wy_susp_low_pts), '.r', 'DisplayName', 'Wy Susp');
% plot(t(~ax_low_pts), 0.2248 * F_aero(~ax_low_pts), '.g', 'DisplayName', 'Ax');
% plot(t(~ay_low_pts), 0.2248 * F_aero(~ay_low_pts), '.m', 'DisplayName', 'Ay');
% plot(t(~az_low_pts), 0.2248 * F_aero(~az_low_pts), '.k', 'DisplayName', 'Az');
% plot(t(~wy_dot_low_pts), 0.2248 * F_aero(~wy_dot_low_pts), '.r', 'DisplayName', 'Wy Dot');

plot(t, 0.2248 * F_aero_front, 'k--', 'DisplayName', 'Front');
plot(t, 0.2248 * F_aero_rear, 'm--', 'DisplayName', 'Rear');
% plot(t(valid_data_pts), 0.2248 * F_aero(valid_data_pts), '.r', 'DisplayName', 'Valid');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Downforce lb]');
ylim([-100, 300]);

% Aero coefficients
n = n + 1;
subplot(n_max, 1, n);
hold on;
plot(t, Cl, 'b', 'DisplayName', 'C_L');
plot(t(valid_data_pts), Cl(valid_data_pts), '.r', 'DisplayName', 'Valid');
grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Force Coefficient');
ylim([0, 2]);

% % Aero balance
% n = n + 1;
% subplot(n_max, 1, n);
% hold on;
% plot(t, 100 * aero_bal, '-b');
% plot(t(~low_DF_indices), 100 * aero_bal(~low_DF_indices), '.r');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Aero Balance [%]');
% ylim([-100, 100]);

% Total loads
n = n + 1;
subplot(n_max, 1, n);
hold on
plot(t, 0.2248 * (FL_aero.spring + FR_aero.spring), ':b', 'DisplayName', 'Spring-F');
% plot(t, 0.2248 * (FL_aero.damper + FR_aero.damper), ':m', 'DisplayName', 'Damper-F');
plot(t, 0.2248 * (FL_aero.anti + FR_aero.anti), ':m', 'DisplayName', 'Anti-F');
plot(t, 0.2248 * (FL_aero.static + FR_aero.static), ':r', 'DisplayName', 'Static-F');

plot(t, 0.2248 * (RL_aero.spring + RR_aero.spring), '-b', 'DisplayName', 'Spring-R');
% plot(t, 0.2248 * (RL_aero.damper + RR_aero.damper), '-m', 'DisplayName', 'Damper-R');
plot(t, 0.2248 * (RL_aero.anti + RR_aero.anti), '-m', 'DisplayName', 'Anti-R');
plot(t, 0.2248 * (RL_aero.static + RR_aero.static), '-r', 'DisplayName', 'Static-R');

% plot(t, 0.2248 * (FL_aero.spring + FR_aero.spring + RL_aero.spring + RR_aero.spring), ...
%     'g', 'DisplayName', 'Spring');
% plot(t, 0.2248 * (FL_aero.damper + FL_aero.damper + RL_aero.damper + RR_aero.damper), ...
%     'k', 'DisplayName', 'Damper');
% plot(t, 0.2248 * (FL_aero.anti + FL_aero.anti + RL_aero.anti + RR_aero.anti), ...
%     'c', 'DisplayName', 'Anti');
% plot(t, 0.2248 * (FL_aero.static + FR_aero.static + RL_aero.static + RR_aero.static), ...
%     'k', 'DisplayName', 'Static');

grid on;
legend(gca, 'show');
xlabel('Time [s]');
ylabel('Load Totals [lb]');
% ylim([5000, 10000]);
% ylim([-500, 1500]);


% % Shocks
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% plot(t, FL_aero.shock, 'b', 'DisplayName', 'FL');
% plot(t, FR_aero.shock, 'g', 'DisplayName', 'FR');
% plot(t, RL_aero.shock, 'r', 'DisplayName', 'RL');
% plot(t, RR_aero.shock, 'y', 'DisplayName', 'RR');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Shock [N]');

% % Susp WT
% n = n + 1;
% subplot(n_max, 1, n);
% hold on
% plot(t, FL_aero.susp + FR_aero.susp, '.b', 'DisplayName', 'Front');
% plot(t, RL_aero.susp + RR_aero.susp, '.m', 'DisplayName', 'Rear');
% grid on;
% legend(gca, 'show');
% xlabel('Time [s]');
% ylabel('Susp WT [N]');


%%

% % Plot downforce coefficeint vs acceleration
% figure(3);
% clf;
%
% subplot(1, 2, 1);
% grid on;
% hold on;
% plot(ax(valid_data_pts), Cl(valid_data_pts), 'x');
% xlabel('Longitudinal Acceleration [m/s^2]');
% ylabel('Downforce Coefficient');
% ylim([-0.5, 1.0]);
%
% subplot(1, 2, 2);
% grid on;
% hold on;
% plot(ay(valid_data_pts), Cl(valid_data_pts), 'x');
% xlabel('Lateral Acceleration [m/s^2]');
% ylabel('Downforce Coefficient');
% ylim([-0.5, 1.0]);


% Plot of downforce vs speed
% Cl_ref = 0.204;
% Cl_ref = 0.26;
% 
% v_sample = linspace(0, max(v_air));
% F_pred = 0.5 * rho_air * Cl_ref * exige.A_ref * v_sample.^2;
% 
% figure(3);
% clf;
% hold on;
% plot(3.6 * v_air(valid_data_pts), 0.2248 * F_aero(valid_data_pts), 'gx');
% plot(3.6 * v_sample, 0.2248 * F_pred, 'r-');
% xlabel('Speed [km/h]');
% ylabel('Downforce [lb]');
% grid  on;
% ylim([-50, 200]);

%
% % Plot of aero balance vs speed
% figure(5);
% clf;
% plot(3.6 * v_air(low_DF_indices), 100 * aero_bal(low_DF_indices), 'x');
% xlabel('Speed [km/h]');
% ylabel('Aero Balance [%]');
% grid  on;
% ylim([0, 100]);
