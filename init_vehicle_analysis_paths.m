function init_vehicle_analysis_paths()
    source_dir = fileparts(mfilename('fullpath'));
    contents = dir(source_dir);
    
    for i=1:length(contents)
        if contents(i).isdir && ~strcmp(contents(i).name, '.') && ~strcmp(contents(i).name, '..')
            addpath(fullfile(source_dir, contents(i).name));
        end
    end
end