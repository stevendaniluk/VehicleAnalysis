% createSimpleMotecLog
%
% Helper to generate a simple fake dataset in the same format as Matlab data outputted
% by MoTeC i2.
%
% INPUTS:
%   (filename): When present the generated log will be saved to 'filename'
% OUTPUTS:
%   log: Struct with fields for each channel
function log = createSimpleMotecLog(varargin)
    n_points = 11;
    t_start =  0.0;
    t_end = 5.0;

    t = linspace(t_start, t_end, n_points);

    throttle_pedal.Time = t;
    throttle_pedal.Value = linspace(0, 0.6, n_points);
    throttle_pedal.Units = '%';

    brake_pedal.Time = t;
    brake_pedal.Value = linspace(0, 0.9, n_points);
    brake_pedal.Units = '%';

    rpm_eng.Time = t;
    rpm_eng.Value = linspace(1000, 8000, n_points);
    rpm_eng.Units = '';

    if ~isempty(varargin) && isa(varargin{1}, 'char')
        save(varargin{1}, 'throttle_pedal', 'brake_pedal', 'rpm_eng')
    end

    % Put all channels into a single struct to return
    log.throttle_pedal = throttle_pedal;
    log.brake_pedal = brake_pedal;
    log.rpm_eng = rpm_eng;
end
