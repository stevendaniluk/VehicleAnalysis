function exige = createExige()
    exige = Vehicle;

    exige.L = 2.30;  % From factory specs
    exige.track_f = 1.457;  % From factory specs
    exige.track_r = 1.506;  % From factory specs

    exige.m_unsprung_f = 91.9;  % Measured
    exige.h_unsprung_f = 0.306;  % Derived from static tire radius

    exige.m_unsprung_r = 84.1;  % Measured
    exige.h_unsprung_r = 0.326;  % Derived from static tire radius

    exige.V_fuel = 0.0;
    exige.l_fuel = 1.50;  % Measured
    exige.h_fuel = 0.20;  % Guess

    exige.RWD = true;

    exige.steer_ratio = 15.8;  % From factory specs
    exige.ackermann = 0.3;  % From factory specs

    exige.brake_bias = 0.62; % Calculated from brake specs

    exige.FL_corner.toe = rad2deg(0.15);
    exige.FL_corner.camber = -2.7;
    exige.FL_corner.caster = 3.8;
    exige.FL_corner.MR_spring = 1.428;
    exige.FL_corner.kspring = 105e3;
    exige.FL_corner.kbump = 0;
    exige.FL_corner.bumpstop_thickness = 0;
    exige.FL_corner.shock_length = 0.35;  % TODO - Guessed
    exige.FL_corner.shock_travel = 0.0665;  % Measured
    exige.FL_corner.spring_perch_offset = 0;
    exige.FL_corner.spring_cup_height = 0.03;  % TODO - Guessed
    exige.FL_corner.spring_length = 0.1524;  % Measured
    exige.FL_corner.x0 = 0;
    exige.FL_corner.F_damper = @(v) 0*50000 * v;

    exige.FR_corner = exige.FL_corner;

    exige.RL_corner.toe = rad2deg(0.05);
    exige.RL_corner.camber = -2.7;
    exige.RL_corner.caster = 0;
    exige.RL_corner.MR_spring = 1.392;
    exige.RL_corner.kspring = 140e3;
    exige.RL_corner.kbump = 0;
    exige.RL_corner.bumpstop_thickness = 0;
    exige.RL_corner.shock_length = 0.45;  % TODO - Guessed
    exige.RL_corner.shock_travel = 0.095;  % Measured
    exige.RL_corner.spring_perch_offset = 0;
    exige.RL_corner.spring_cup_height = 0.03;  % TODO - Guessed
    exige.RL_corner.spring_length = 0.1778;  % Measured
    exige.RL_corner.x0 = 0;
    exige.RL_corner.F_damper = @(v) 0*50000 * v;

    exige.RR_corner = exige.RL_corner;

    exige.MR_arb_f = 1.0;
    exige.MR_arb_r = 1.0;

    exige.karb_f = 0; % TODO
    exige.karb_r = 0;

    exige.h_roll_f = @(RH) RH - 0.2; % TODO
    exige.h_roll_r = @(RH) RH - 0.2; % TODO

    exige.SVIC_ang_f = deg2rad(2.0); % TODO
    exige.SVIC_ang_r = deg2rad(0.0); % TODO

    exige.r_nominal_f = 0.306;
    exige.r_nominal_r = 0.326;
    exige.r_eff_f = @(P, w, Fz) 0.306;
    exige.r_eff_r = @(P, w, Fz) 0.326;

    % Initial guess for aerodynamic properties
    rho_air = 1.225;
    exige.A_ref = 1.70;
    Cl = 0.204;
    Cd = 0.442;
    aero_bal = 0.5;
    exige.DF_f = @(v, RH_f, pitch) aero_bal * 0.5 * rho_air *  Cl * exige.A_ref * v.^2;
    exige.DF_r = @(v, RH_f, pitch) (1 - aero_bal) * 0.5 * rho_air *  Cd * exige.A_ref * v.^2;
    exige.drag = @(v, RH_f, pitch) 0.5 * rho_air *  Cd * exige.A_ref * v.^2;

    % Compute spring perch offsets

    % Measurements from 2023/06/07
    m = 978;
    F_frac = 0.3946;
    L_frac = 0.5081;
    V_fuel = 31.8;
    RH_f = 0.115;
    RH_r = 0.13;
    x_fl = 0.02479;
    x_fr = 0.02621;
    x_rl = 0.03975;
    x_rr = 0.03779;
    h_cog = 0.4; % Guess

    m_FL = F_frac * L_frac * m;
    m_FR = F_frac * (1 - L_frac) * m;
    m_RL = (1 - F_frac) * L_frac * m;
    m_RR = (1 - F_frac) * (1 - L_frac) * m;

    W_FL = m_FL * exige.gravity;
    W_FR = m_FR * exige.gravity;
    W_RL = m_RL * exige.gravity;
    W_RR = m_RR * exige.gravity;

    exige = exige.sprungPropertiesFromTotal(m_FL, m_FR, m_RL, m_RR, V_fuel, h_cog, RH_f, RH_r);
    exige = exige.setReferenceRideHeight(RH_f, RH_r, x_fl, x_fr, x_rl, x_rr);
    exige = exige.setSpringPerchFromWheelLoads(x_fl, x_fr, x_rl, x_rr, W_FL, W_FR, W_RL, W_RR);
end
