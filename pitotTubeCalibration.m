% Load all the data
ground_speed = {};
p_meas = {};
rho = {};

% Highway accelerating
handler = MotecHandlerExige('/Users/steven/car_logs/Exige/dev/pitot_calibration/highway_accelerating.mat');
ground_speed{1} = handler.getChannel('GROUND_SPEED');
p_meas{1} = 10 * max(0, handler.getChannel('AIR_DYN_PRESSURE'));
rho{1} = 1.180;

% Thunderhill lap
handler = MotecHandlerExige('/Users/steven/car_logs/Exige/dev/pitot_calibration/thunderhill_west_2023_06_10_session_3.mat');
ground_speed{2} = handler.getChannel('GROUND_SPEED');
p_meas{2} = max(0, handler.getChannel('AIR_DYN_PRESSURE'));
rho{2} = 1.154;

% close all
calibrator = PitotCalibrator;
calibrator = calibrator.addData(ground_speed{2}, p_meas{2}, rho{2});
calibrator = calibrator.binData(10.0, 45.0, 8, 1000);
calibrator = calibrator.fitPolynomial(2);
calibrator.plotVelocityData();
calibrator.plotPolynomialFit();







