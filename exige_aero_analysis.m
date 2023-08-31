clear;
% clc;

exige = createExige();
exige.gravity = 9.92;

analysis = AeroAnalysis(exige);

analysis = analysis.addFilteringParams('GROUND_SPEED', 0.2, -0.5);
analysis = analysis.addFilteringParams('AIR_DYN_PRESSURE', 0.5, -0.3);
analysis = analysis.addFilteringParams('ELEVATION', 1.0, -0.1);
analysis = analysis.addFilteringParams('VZ', 1.0, 0);
analysis = analysis.addFilteringParams('AX', 1.0, 0);
analysis = analysis.addFilteringParams('AY', 1.0, 0);
analysis = analysis.addFilteringParams('AZ', 1.0, 0);
analysis = analysis.addFilteringParams('WY', 3.0, 0);
analysis = analysis.addFilteringParams('PITCH_SUSP', 0.5, 0);
analysis = analysis.addFilteringParams('DAMPER_FL', 1.0, -0.05);
analysis = analysis.addFilteringParams('DAMPER_FR', 1.0, -0.05);
analysis = analysis.addFilteringParams('DAMPER_RL', 1.0, -0.05);
analysis = analysis.addFilteringParams('DAMPER_RR', 1.0, -0.05);
analysis = analysis.addFilteringParams('HEAVE_RATE', 0.5, 0);

analysis = analysis.addPlottingOverride('GROUND_SPEED', 'scale', 3.6, 'units', 'km/h');
analysis = analysis.addPlottingOverride('AIR_SPEED', 'scale', 3.6, 'units', 'km/h');
analysis = analysis.addPlottingOverride('ROAD_GRADE', 'scale', rad2deg(1), 'units', 'deg');
analysis = analysis.addPlottingOverride('AZ', 'bias', exige.gravity);
analysis = analysis.addPlottingOverride('WY', 'scale', rad2deg(1), 'units', 'deg');
analysis = analysis.addPlottingOverride('WY_SUSP', 'scale', rad2deg(1), 'units', 'deg');
analysis = analysis.addPlottingOverride('DAMPER_FL', 'scale', 1e3, 'units', 'mm');
analysis = analysis.addPlottingOverride('DAMPER_FR', 'scale', 1e3, 'units', 'mm');
analysis = analysis.addPlottingOverride('DAMPER_RL', 'scale', 1e3, 'units', 'mm');
analysis = analysis.addPlottingOverride('DAMPER_RR', 'scale', 1e3, 'units', 'mm');
analysis = analysis.addPlottingOverride('F_AERO_FRONT', 'scale', 0.2248, 'units', 'lb');
analysis = analysis.addPlottingOverride('F_AERO_REAR', 'scale', 0.2248, 'units', 'lb');
analysis = analysis.addPlottingOverride('F_AERO_TOTAL', 'scale', 0.2248, 'units', 'lb');

% % Define the regions of the track where we want to collect data
analysis = analysis.addLapZone(0, 1290); % Start to before T5
% analysis = analysis.addLapZone(590, 1290); % Before T3, to before T5
analysis = analysis.addLapZone(3000, inf); % After T10 to end

sessions = {};

% Session 1
sessions{1}.filename = 'exige_session_1.mat';
sessions{1}.rho_air = 1.173;
sessions{1}.Vf_i = 42.0;
sessions{1}.Vf_f = 31.2;

% Session 2
sessions{2}.filename = 'exige_session_2.mat';
sessions{2}.rho_air = 1.136;
sessions{2}.Vf_i = 31.2;
sessions{2}.Vf_f = 24.2;
% sessions{2}.t_trim = [0, 200]; % Entry
% sessions{2}.t_trim = [292, 414]; % Full fast lap
% sessions{2}.t_trim = [292, 310]; % Front straight
% sessions{2}.t_trim = [390, 423]; % T11 - T2
% sessions{2}.t_trim = [315, 351]; % T3-T5
% sessions{2}.t_trim = [364, 381]; % Corkscrew
% sessions{2}.t_trim = [950, inf]; % Exit

% Session 4 - A
sessions{4}.filename = 'exige_session_4.mat';
sessions{4}.rho_air = 1.126;
sessions{4}.Vf_i = 34.5;
sessions{4}.Vf_f = 18.3;
sessions{4}.lap_trim =  [2, 6];

% Session 4 - B
sessions{5}.filename = 'exige_session_4.mat';
sessions{5}.rho_air = 1.126;
sessions{5}.Vf_i = 34.5;
sessions{5}.Vf_f = 18.3;
sessions{5}.lap_trim =  [9, 15];

for i = 1 : length(sessions)
    if isempty(sessions{i})
        continue;
    end

    handler = MotecHandlerExige(sessions{i}.filename);
    handler = setFuelLevelInHandler(handler, sessions{i}.Vf_i, sessions{i}.Vf_f);

    handler = handler.removeInAndOutLaps();
    if isfield(sessions{i}, 'lap_trim') && ~isempty(sessions{i}.lap_trim)
        handler = handler.trimLaps(sessions{i}.lap_trim(1), sessions{i}.lap_trim(2));
    end
    if isfield(sessions{i}, 't_trim') && ~isempty(sessions{i}.t_trim)
        handler = handler.trimTimeRange(sessions{i}.t_trim(1), sessions{i}.t_trim(2));
    end

    analysis = analysis.addDataset(handler, sessions{i}.rho_air);
end

analysis = analysis.postProcess();

%%

% Main inputs
fig = figure(1);
clf;
n = 4;
subplot(n, 1, 1);
analysis.plotTimeData('GROUND_SPEED', '-b', fig);
analysis.plotTimeData('AIR_SPEED', '--r', fig);
analysis.plotProcTimeData('AIR_SPEED', '.g', fig);

subplot(n, 1, 2);
analysis.plotTimeData('THROTTLE', '-b', fig);
analysis.plotTimeData('BRAKE', '-r', fig);
analysis.plotTimeData('CLUTCH', '-y', fig);

subplot(n, 1, 3);
analysis.plotTimeData('AX', '-g', fig);
analysis.plotTimeData('AY', '-b', fig);
analysis.plotTimeData('AZ', '-r', fig);
ylim([-3, 3]);

subplot(n, 1, 4);
analysis.plotTimeData('DAMPER_FL', '-b', fig);
analysis.plotTimeData('DAMPER_FR', '-g', fig);
analysis.plotTimeData('DAMPER_RL', '-r', fig);
analysis.plotTimeData('DAMPER_RR', '-y', fig);
ylim([0, 80]);

% fig = figure(2);
% clf;
% n = 5;
%
% subplot(n, 1, 1);
% analysis.plotTimeData('GROUND_SPEED', '-b', fig, 511);
% subplot(n, 1, 2);
% analysis.plotTimeData('ELEVATION', '-b', fig, 512);
% subplot(n, 1, 3);
% analysis.plotTimeData('ROAD_GRADE', '-b', fig, 513);
% subplot(n, 1, 4);
% analysis.plotTimeData('VZ', '-b', fig, 514);
% subplot(n, 1, 5);
% analysis.plotTimeData('AZ', '-b', fig, 515);

%% Aero properties for all data
fig = figure(2);
clf;
n = 4;

subplot(n, 1, 1);
analysis.plotTimeData('GROUND_SPEED', '-b', fig);
analysis.plotTimeData('AIR_SPEED', '--r', fig);
analysis.plotProcTimeData('GROUND_SPEED', '.g', fig);

subplot(n, 1, 2);
analysis.plotTimeData('AX', '-g', fig);
analysis.plotTimeData('AY', '-b', fig);
analysis.plotTimeData('AZ', '-r', fig);
ylim([-5, 5]);

subplot(n, 1, 3);
analysis.plotTimeData('F_AERO_TOTAL', '-b', fig);
analysis.plotProcTimeData('F_AERO_TOTAL', '.m', fig);
analysis.plotTimeData('F_AERO_FRONT', '--g', fig);
analysis.plotTimeData('F_AERO_REAR', '--r', fig);
ylim([-100, 300]);


subplot(n, 1, 4);
analysis.plotTimeData('CL', '-b', fig);
analysis.plotProcTimeData('CL', '.g', fig);
ylim([0, 2]);

fig = figure(3);
clf;
n = 2;

subplot(n, 1, 1);
analysis.plotTimeData('F_AERO_TOTAL', '-b', fig);
analysis.plotTimeData('F_AERO_FRONT', '--g', fig);
analysis.plotTimeData('F_AERO_REAR', '--r', fig);
ylim([-100, 300]);

subplot(n, 1, 2);
analysis.plotTimeData('F_AERO_SPRING', '-b', fig);
analysis.plotTimeData('F_AERO_SPRING_F', '--b', fig);
analysis.plotTimeData('F_AERO_SPRING_R', ':b', fig);
analysis.plotTimeData('F_AERO_STATIC', '-r', fig);
analysis.plotTimeData('F_AERO_STATIC_F', '--r', fig);
analysis.plotTimeData('F_AERO_STATIC_R', ':r', fig);
analysis.plotTimeData('F_AERO_ANTI', '-g', fig);
analysis.plotTimeData('F_AERO_ANTI_F', '--g', fig);
analysis.plotTimeData('F_AERO_ANTI_R', ':g', fig);
% ylim([-100, 300]);


% %% Aero properties for only analysis data
% fig = figure(3);
% clf;
% n = 5;
%
% subplot(n, 1, 1);
% analysis.plotProcTimeData('GROUND_SPEED', '-b', fig);
% analysis.plotProcTimeData('AIR_SPEED', '--r', fig);
%
% subplot(n, 1, 2);
% analysis.plotProcTimeData('AX', '-g', fig);
% analysis.plotProcTimeData('AY', '-b', fig);
% analysis.plotProcTimeData('AZ', '-r', fig);
% ylim([-5, 5]);
%
% subplot(n, 1, 3);
% analysis.plotProcTimeData('F_AERO_TOTAL', '-b', fig);
% analysis.plotProcTimeData('F_AERO_FRONT', '--g', fig);
% analysis.plotProcTimeData('F_AERO_REAR', '--r', fig);
% ylim([-100, 300]);
%
% subplot(n, 1, 4);
% analysis.plotProcTimeData('CL', '-b', fig);
% ylim([0, 2]);
%
% subplot(n, 1, 5);
% analysis.plotProcTimeData('AERO_BALANCE', '-b', fig);
% ylim([-50, 150]);


%% Cl  plots
fig = figure(4);
clf;

V_x_lim = [60, 180];
F_y_lim = [-50, 200];

Cl_ref = 0.32;
A_ref = 1.70;
rho_air_mean = mean(cell2mat(analysis.proc_datasets.RHO_AIR));

V_ref = linspace(V_x_lim(1), V_x_lim(2));
F_aero_ref = 0.2248 * 0.5 * rho_air_mean * (V_ref / 3.6).^2 * A_ref * Cl_ref;

F_aero_ref_bias = -20.0;
F_aero_ref = F_aero_ref + F_aero_ref_bias;

subplot(2, 2, 1);
hold on;
analysis.plotProcData('AIR_SPEED', 'F_AERO_TOTAL', 'xb', fig);
plot(V_ref, F_aero_ref, 'r');
xlim(V_x_lim);
ylim(F_y_lim);

subplot(2, 2, 3);
analysis.plotProcData('AIR_SPEED', 'CL', 'xm', fig);
xlim(V_x_lim);
ylim([-0.2, 1.2]);

subplot(2, 2, 2);
analysis.plotProcData('AIR_SPEED', 'F_AERO_FRONT', 'xg', fig);
xlim(V_x_lim);
ylim(F_y_lim);

subplot(2, 2, 4);
analysis.plotProcData('AIR_SPEED', 'F_AERO_REAR', 'xr', fig);
xlim(V_x_lim);
ylim(F_y_lim);
