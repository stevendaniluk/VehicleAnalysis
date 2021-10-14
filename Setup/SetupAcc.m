% SetupAcc
%
% Assetto Corsa Competizione specialization of to the Setup class.
%
% Adds support for loading the setup from a file.
classdef SetupAcc < Setup
    properties
        % Name of the car in the setup file
        car_name = '';
        % Front and rear toe angle values
        toe_f_vals = [];
        toe_r_vals = [];
        % Front and rear camber angle values
        camber_f_vals = [];
        camber_r_vals = [];
        % Caster angle values
        caster_vals = [];
        % Front and rear ARB values
        karb_f_vals = [];
        karb_r_vals = [];
        % Front and rear spring rate values
        kspring_f_vals = [];
        kspring_r_vals = [];
        % Front and rear ride height values
        RH_f_vals = [];
        RH_r_vals = [];
        % Front splitter values
        splitter_vals = [];
        % Rear wing values
        rear_wing_vals = [];
        % Map of track specific dry mass values
        dry_mass_track_map = containers.Map();
        % Map of fuel consumption for each track [L/s]
        fuel_consumption_track_map = containers.Map();
    end

    methods

        % loadFromJSON
        %
        % Updates a setup to match the quantities defined in a JSON setup file saved
        % from the game. This will set:
        %   -Fuel level
        %   -Wheel/spring rates
        %   -Ride height
        %   -Rod lengths
        %
        % INPUTS:
        %   filename: Path to JSON setup file from game
        function this = loadFromJSON(this, filename)
            [~, ~, ext] = fileparts(filename);
            if ~isequal(ext, '.json')
                filename = strcat(filename, '.json');
            end

            json_string = fileread(filename);
            json_setup = jsondecode(json_string);

            % Initialize the car type
            this = this.initializeCarType(json_setup.carName);

            % Set the fuel level (0 is equal to 2L)
            this.V_fuel = 2 + json_setup.basicSetup.strategy.fuel;

            % Set the toe angles
            this.toe_f = this.toe_f_vals(json_setup.basicSetup.alignment.toe(1) + 1);
            this.toe_r = this.toe_r_vals(json_setup.basicSetup.alignment.toe(3) + 1);

            % Set the camber angles
            this.camber_f = this.camber_f_vals(json_setup.basicSetup.alignment.camber(1) + 1);
            this.camber_r = this.camber_r_vals(json_setup.basicSetup.alignment.camber(3) + 1);

            % Set ARB rates
            this.karb_wheel_f = ...
                this.karb_f_vals(json_setup.advancedSetup.mechanicalBalance.aRBFront + 1);
            this.karb_wheel_r = ...
                this.karb_r_vals(json_setup.advancedSetup.mechanicalBalance.aRBRear + 1);

            % Set the spring rates
            this.kspring_wheel_f = ...
                this.kspring_f_vals(json_setup.advancedSetup.mechanicalBalance.wheelRate(1) + 1);
            this.kspring_wheel_r = ...
                this.kspring_r_vals(json_setup.advancedSetup.mechanicalBalance.wheelRate(3) + 1);
            this = this.setSpringRatesFromWheel();

            % Set the ride height
            this.RH_f_static = ...
                this.RH_f_vals(json_setup.advancedSetup.aeroBalance.rideHeight(1) + 1);
            this.RH_r_static = ...
                this.RH_r_vals(json_setup.advancedSetup.aeroBalance.rideHeight(3) + 1);

            % Set the wing/splitter
            this.wing_front = json_setup.advancedSetup.aeroBalance.splitter;
            this.wing_rear = json_setup.advancedSetup.aeroBalance.rearWing;
        end
    end

    methods (Access = protected)

        % initializeCarType
        %
        % Populates the possible values for setup properties so that the actual
        % values can be loaded from the JSON file.
        %
        % Also defines some physical properties for each car.
        %
        % Here we'll define all the setup value ranges for all possible cars.
        % This currently handles:
        %   -Toe angle
        %   -Camber angle
        %   -Caster angle
        %   -ARB value
        %   -Spring rates
        %   -Ride height
        %   -Front splitter
        %   -Rear wing
        %
        % INPUTS:
        %   car_name: Name of the car specified in the JSON file
        function this = initializeCarType(this, car_name)
            % Common settings for all cars
            this.steer_ratio = 11;

            if strcmp(car_name, 'amr_v8_vantage_gt3')
                % Vehicle constants
                this.L = 2.70;
                this.b_f = 1.67;
                this.b_r = 1.67;

                % Setup value ranges
                this.toe_f_vals = -deg2rad(-0.40:0.01:0.40);
                this.toe_r_vals = -deg2rad(-0.40:0.01:0.40);

                this.camber_f_vals = deg2rad(-4.0:0.1:-1.5);
                this.camber_r_vals = deg2rad(-3.5:0.1:-1.0);

                this.caster_vals = deg2rad(10.7:0.1:16.1);

                this.karb_f_vals = 0:1:8;
                this.karb_r_vals = 0:1:8;

                this.kspring_f_vals = 115000:10000:185000;
                this.kspring_r_vals = 105000:10000:195000;

                this.RH_f_vals = 55:1:80;
                this.RH_r_vals = 55:1:90;

                this.splitter_vals = 0;
                this.rear_wing_vals = 0:1:10;
            else
                error('Unrecognized car name: %s', car_name);
            end

            this.car_name = car_name;
        end
    end
end
