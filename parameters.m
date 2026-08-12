%% parameters.m
% EMRAX 208 + simple FSAE traction-control parameter set (P304 / UTS Motorsports).
% Run this script (or add it to the model InitFcn) to load symbols into the workspace.
% Accompanying notes: parameters.md
%
% Units: SI unless noted. NaN = unknown / TBC. Choose winding & cooling below.

clearvars; close all; clc;

%% -------------------------------------------------------------------------
% Configuration switches (set before using derived limits)
% -------------------------------------------------------------------------
winding_type  = "HV";   % "HV" | "MV" | "LV"  — TBC for our car
cooling_type  = "liquid"; % "air" | "liquid" | "combined" — TBC
gear_ratio_i  = 3.5;    % primary reduction i; options for reflected inertia: 3, 3.5, 4, 4.5

%% =========================================================================
% 1. ACTUATOR — EMRAX 208 torque & power production
% =========================================================================

T_pk   = 150;           % [Nm] peak shaft torque (datasheet)
P_pk   = 86e3;          % [W]  peak shaft power (datasheet overview)
omega_max_rpm = 7000;   % [rpm] limiting speed
omega_max = omega_max_rpm * (2*pi/60); % [rad/s]

% Continuous torque by cooling [Nm]: air / liquid / combined
T_cont_air = 54; T_cont_liq = 84; T_cont_comb = 90;
switch cooling_type
    case "air",      T_cont = T_cont_air;
    case "liquid",   T_cont = T_cont_liq;
    case "combined", T_cont = T_cont_comb;
    otherwise, error("cooling_type must be air|liquid|combined");
end

% Torque constant K_T [Nm/A_RMS] by winding: HV / MV / LV
K_T_HV = 0.62; K_T_MV = 0.38; K_T_LV = 0.15;
% Peak / continuous current [A_RMS]: HV / MV / LV
I_pk_HV = 240;  I_pk_MV = 400;  I_pk_LV = 1000;
I_cont_HV = 140; I_cont_MV = 220; I_cont_LV = 560;
% Nominal bus voltage by winding [V]
V_mot_HV = 690; V_mot_MV = 420; V_mot_LV = 170;

switch winding_type
    case "HV"
        K_T = K_T_HV; I_pk = I_pk_HV; I_cont = I_cont_HV; V_mot = V_mot_HV;
    case "MV"
        K_T = K_T_MV; I_pk = I_pk_MV; I_cont = I_cont_MV; V_mot = V_mot_MV;
    case "LV"
        K_T = K_T_LV; I_pk = I_pk_LV; I_cont = I_cont_LV; V_mot = V_mot_LV;
    otherwise
        error("winding_type must be HV|MV|LV");
end

dKT_dT = NaN;           % [Nm/A/°C] torque-constant drift vs magnet/winding temp — TBD
tau_m  = 3.5e-3;        % [s] motor+inverter torque lag (~2–5 ms); mid-range estimate
reverse_ok = true;      % full negative torque (regen) available per datasheet

P_lim  = 80e3;          % [W] FSAE 2026 DC accumulator power limit (V_DC * I_DC)
eta_pk = 0.96;          % [-] peak motor efficiency (near efficiency island only)
% eta(omega, T) map — placeholder; load measured/lookup map later
eta_map = NaN;

% Continuous shaft power by cooling [W]
P_cont_air = 33e3; P_cont_liq = 52e3; P_cont_comb = 56e3;
switch cooling_type
    case "air",      P_cont = P_cont_air;
    case "liquid",   P_cont = P_cont_liq;
    case "combined", P_cont = P_cont_comb;
end

%% =========================================================================
% 2. ELECTRICAL — pack / inverter interfaces
% =========================================================================

V_OC  = NaN;            % [V] battery open-circuit voltage — measure
R_int = NaN;            % [Ohm] pack internal resistance — measure
I_inv_pk   = NaN;       % [A] inverter peak current limit — TBC (use min with I_pk)
I_inv_cont = NaN;       % [A] inverter continuous current — TBC
% Effective saturating currents once inverter is known:
% I_pk_eff   = min(I_pk,   I_inv_pk);
% I_cont_eff = min(I_cont, I_inv_cont);

%% =========================================================================
% 3. MECHANICAL, TEMPERATURE & MOTOR SENSING
% =========================================================================

J_r = 0.01569;          % [kg·m^2] rotor inertia (datasheet)
% Reflected rotor inertia at gear ratio i: J_r_refl = J_r * i^2
J_r_refl_table = containers.Map( ...
    {3, 3.5, 4, 4.5}, {0.141, 0.192, 0.251, 0.318}); % [kg·m^2] precomputed
if isKey(J_r_refl_table, gear_ratio_i)
    J_r_refl = J_r_refl_table(gear_ratio_i);
else
    J_r_refl = J_r * gear_ratio_i^2; % [kg·m^2]
end

m_m = 9.85;             % [kg] motor mass mid of 9.4–10.3 kg range
m_m_min = 9.4; m_m_max = 10.3;

T_lim_wind = 100;       % [°C] winding sensor derate start
T_lim_rotor = 100;      % [°C] rotor surface limit
sensor_type = "";       % "resolver" | "encoder" — fitted type TBC
omega_m = NaN;          % [rad/s] motor rotor speed (from resolver/encoder) — runtime
T_wind  = NaN;          % [°C] winding temperature — runtime / TBD sensor mapping

%% =========================================================================
% 4. DRIVELINE — gearbox / halfshafts (placeholders)
% =========================================================================

i_g = gear_ratio_i;     % [-] overall motor-to-driven-wheel ratio
eta_g = NaN;            % [-] gearbox efficiency — TBD
J_g   = NaN;            % [kg·m^2] gearbox inertia (motor-referred or axle — define) — TBD
T_shaft_max = NaN;      % [Nm] shaft/CV torque rating — TBD

%% =========================================================================
% 5. VEHICLE BODY (placeholders for longitudinal TC)
% =========================================================================

m_v   = NaN;            % [kg] vehicle mass (ready-to-run)
L     = NaN;            % [m] wheelbase
a     = NaN;            % [m] CG to front axle
b     = NaN;            % [m] CG to rear axle  (a + b = L)
h_cg  = NaN;            % [m] CG height
g     = 9.81;           % [m/s^2]
CdA   = NaN;            % [m^2] drag area
Crr   = NaN;            % [-] rolling resistance coefficient
rho_air = 1.225;        % [kg/m^3] air density (sea level approx.)
v_x   = NaN;            % [m/s] vehicle longitudinal speed — runtime / estimate

%% =========================================================================
% 6. WHEELS & TYRES — four corners (placeholders)
% Corner order throughout: FL, FR, RL, RR
% =========================================================================

R_e = NaN;              % [m] effective rolling radius (all four if same tyres)
R_e_FL = NaN; R_e_FR = NaN; R_e_RL = NaN; R_e_RR = NaN; % [m] per corner if needed
J_w  = NaN;             % [kg·m^2] one wheel+hub+rotor inertia about axle

% Wheel angular speeds [rad/s] — primary TC inputs
omega_w_FL = NaN; omega_w_FR = NaN; omega_w_RL = NaN; omega_w_RR = NaN;
omega_w = [omega_w_FL, omega_w_FR, omega_w_RL, omega_w_RR];

% Linear wheel speeds [m/s]: u = omega_w * R_e  (use when R_e known)
u_FL = NaN; u_FR = NaN; u_RL = NaN; u_RR = NaN;

% Longitudinal slip ratios [-]: kappa = (u - v_x) / max(|v_x|, v_eps)
v_eps = 0.5;            % [m/s] avoid divide-by-zero at standstill
kappa_FL = NaN; kappa_FR = NaN; kappa_RL = NaN; kappa_RR = NaN;
kappa = [kappa_FL, kappa_FR, kappa_RL, kappa_RR];

% Normal loads [N] — static or estimated with transfer
Fz_FL = NaN; Fz_FR = NaN; Fz_RL = NaN; Fz_RR = NaN;
Fz = [Fz_FL, Fz_FR, Fz_RL, Fz_RR];

% Longitudinal tyre forces [N]
Fx_FL = NaN; Fx_FR = NaN; Fx_RL = NaN; Fx_RR = NaN;

% Simple tyre / friction placeholders
mu_x_max = NaN;         % [-] peak long. friction (track/tyre TBC)
kappa_opt = NaN;        % [-] slip at peak mu — TC setpoint target
B_tire = NaN; C_tire = NaN; D_tire = NaN; E_tire = NaN; % Pacejka magic-formula coeffs — TBD

%% =========================================================================
% 7. TRACTION CONTROL — signals & gains (placeholders)
% =========================================================================

T_cmd     = NaN;        % [Nm] torque request from driver / VCU (pre-TC)
T_tc      = NaN;        % [Nm] torque after traction controller
T_regen_max = NaN;      % [Nm] regen torque ceiling — TBD
kappa_target = NaN;     % [-] target slip for driven axle(s)
K_p_tc = NaN;           % [-] TC proportional gain — tune
K_i_tc = NaN;           % [-] TC integral gain — tune
T_s    = NaN;           % [s] controller sample time — match ECU / model fixed step
driven_axle = "RWD";    % "RWD" | "FWD" | "AWD" — TBC for our car

%% =========================================================================
% 8. Useful derived helpers (fill once config + pack known)
% =========================================================================

% Peak torque from current (ideal): T = K_T * I  (RMS convention per datasheet)
T_from_I_pk   = K_T * I_pk;     % [Nm] check vs T_pk; take min in limiter
T_from_I_cont = K_T * I_cont;   % [Nm]

% Shaft power ↔ angular speed: P = T * omega
% DC-side power for FSAE limiter (needs efficiency): P_DC = (T * omega) / eta

fprintf('parameters.m loaded: winding=%s, cooling=%s, i=%.2f, K_T=%.2f, T_cont=%g Nm\n', ...
    winding_type, cooling_type, i_g, K_T, T_cont);
