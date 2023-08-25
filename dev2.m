% TODO:
% - Fuel level calculation increasing?

clear;
clc;

exige = createExige();
exige.gravity = 9.92;

analysis = AeroAnalysis(exige);

analysis = analysis.addFilteringParams('GROUND_SPEED', 0.2, -0.5);
analysis = analysis.addFilteringParams('AIR_DYN_PRESSURE', 0.5, -0.3);
analysis = analysis.addFilteringParams('ELEVATION', 3.0, -0.5);
analysis = analysis.addFilteringParams('VZ', 2.5, 0);
analysis = analysis.addFilteringParams('AX', 1.0, 0);
analysis = analysis.addFilteringParams('AY', 1.0, 0);
analysis = analysis.addFilteringParams('AZ', 1.2, 0);
analysis = analysis.addFilteringParams('WY', 3.0, 0);
analysis = analysis.addFilteringParams('PITCH_SUSP', 0.5, 0);
analysis = analysis.addFilteringParams('DAMPER_FL', 1.5, -0.1);
analysis = analysis.addFilteringParams('DAMPER_FR', 1.5, -0.1);
analysis = analysis.addFilteringParams('DAMPER_RL', 1.5, 0.2);
analysis = analysis.addFilteringParams('DAMPER_RR', 1.5, 0.2);

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
analysis = analysis.addLapZone(590, 1290); % Before T3, to before T5

sessions = {};

% % Session 1
% sessions{1}.filename = 'exige_session_1.mat';
% sessions{1}.rho_air = 1.173;
% sessions{1}.Vf_i = 42.0;
% sessions{1}.Vf_f = 31.2;
% sessions{1}.t_trim = [201, 1077]; % All flying laps

% Session 2
sessions{2}.filename = 'exige_session_2.mat';
sessions{2}.rho_air = 1.136;
sessions{2}.Vf_i = 31.2;
sessions{2}.Vf_f = 24.2;
% sessions{2}.t_trim = [66.0, 976.0]; % Trim entry/exit
% sessions{2}.t_trim = [185, 842]; % All flying laps
% sessions{2}.t_trim = [0, 70]; % Track entry
sessions{2}.t_trim = [292, 414]; % Full fast lap
% sessions{2}.t_trim = [292, 519]; % Two fast laps
% sessions{2}.t_trim = [292, 310]; % Front straight
% sessions{2}.t_trim = [390, 403]; % T11
% sessions{2}.t_trim = [390, 423]; % T11 - T2
% sessions{2}.t_trim = [317, 353]; % T3-T5
% sessions{2}.t_trim = [325, 330]; % Curbs, exit of T3
% sessions{2}.t_trim = [364, 381]; % Corkscrew
% sessions{2}.t_trim = [80, 100]; % Clutch press
% sessions{2}.t_trim = [650, 700]; % Clutch press
% sessions{2}.t_trim = [800, 960]; % Clutch press

% % Session 4
% sessions{4}.filename = 'exige_session_4.mat';
% sessions{4}.rho_air = 1.126;
% sessions{4}.V_i = 34.5;
% sessions{4}.V_f = 18.3;
% % sessions{4}.t_trim = [106, 2070]; % Trim entry/exit
% % sessions{4}.t_trim = [333, 447]; % Lap 4
% % sessions{4}.t_trim = [322, 350]; % Finish straight

for i = 1 : length(sessions)
    if isempty(sessions{i})
        continue;
    end

    handler = MotecHandlerExige(sessions{i}.filename);
    handler = setFuelLevelInHandler(handler, sessions{i}.Vf_i, sessions{i}.Vf_f);
    if ~isempty(sessions{i}.t_trim)
        handler = handler.trimTimeRange(sessions{i}.t_trim(1), sessions{i}.t_trim(2));
    end
    
    analysis = analysis.addDataset(handler, sessions{i}.rho_air);
end

analysis = analysis.concatenateData();
analysis = analysis.extractAnalysisData();

%%

% Main inputs
fig = figure(3);
clf;
n = 4;
subplot(n, 1, 1);
analysis.plotData('GROUND_SPEED', '-b', fig);
analysis.plotData('AIR_SPEED', '--r', fig);
analysis.plotAnalysisData('AIR_SPEED', '.g', fig);

subplot(n, 1, 2);
analysis.plotData('THROTTLE', '-b', fig);
analysis.plotData('BRAKE', '-r', fig);
analysis.plotData('CLUTCH', '-y', fig);

subplot(n, 1, 3);
analysis.plotData('AX', '-g', fig);
analysis.plotData('AY', '-b', fig);
analysis.plotData('AZ', '-r', fig);
ylim([-1, 1]);

subplot(n, 1, 4);
analysis.plotData('DAMPER_FL', '-b', fig);
analysis.plotData('DAMPER_FR', '-g', fig);
analysis.plotData('DAMPER_RL', '-r', fig);
analysis.plotData('DAMPER_RR', '-y', fig);
ylim([0, 80]);

% fig = figure(2);
% clf;
% n = 5;
% 
% subplot(n, 1, 1);
% analysis.plotData('GROUND_SPEED', '-b', fig, 511);
% subplot(n, 1, 2);
% analysis.plotData('ELEVATION', '-b', fig, 512);
% subplot(n, 1, 3);
% analysis.plotData('ROAD_GRADE', '-b', fig, 513);
% subplot(n, 1, 4);
% analysis.plotData('VZ', '-b', fig, 514);
% subplot(n, 1, 5);
% analysis.plotData('AZ', '-b', fig, 515);

fig = figure(4);
clf;
n = 5;

subplot(n, 1, 1);
analysis.plotData('GROUND_SPEED', '-b', fig);
analysis.plotData('AIR_SPEED', '--r', fig);
analysis.plotAnalysisData('GROUND_SPEED', '.g', fig);

subplot(n, 1, 2);
analysis.plotData('AX', '-g', fig);
analysis.plotData('AY', '-b', fig);
analysis.plotData('AZ', '-r', fig);
ylim([-5, 5]);

subplot(n, 1, 3);
analysis.plotData('F_AERO_TOTAL', '-b', fig);
analysis.plotData('F_AERO_FRONT', '--g', fig);
analysis.plotData('F_AERO_REAR', '--r', fig);
ylim([-100, 300]);

subplot(n, 1, 4);
analysis.plotData('CL', '-b', fig);
ylim([0, 2]);

subplot(n, 1, 5);
analysis.plotData('AERO_BALANCE', '-b', fig);
ylim([-50, 150]);