function dxdt = driveline_dynamics(t, x, Tcmd_fun, Tload_fun, dp)
%DRIVELINE_DYNAMICS  Inverter torque command -> wheel speed, as an ODE.
%   dxdt = driveline_dynamics(t, x, Tcmd_fun, Tload_fun, dp)
%
%   States:  x(1) = T_m      [Nm]     actual motor torque
%            x(2) = omega_w  [rad/s]  axle/wheel speed (both rear wheels lumped as one)
%   Inputs:  Tcmd_fun(t)          -> commanded torque [Nm]
%            Tload_fun(t,omega_w) -> resisting torque at the axle [Nm]
%            dp                   -> driveline struct from driveline_params.m

    T_m     = x(1);   % current motor torque
    omega_w = x(2);   % current axle/wheel speed

    %% 1. Inverter/motor: command -> actual torque
    LIMIT_BY_TORQUE_RATING = false;   % motor/inverter peak-torque rating -- off
    LIMIT_BY_HARD_LIMIT    = true;    % 80 Nm hard limit (team-informed) -- on
    LIMIT_BY_SPEED_CURVE   = true;    % peak-torque-vs-rpm curve (datasheet chart) -- on
    LIMIT_BY_MOTOR_SPEED   = true;    % cut torque to 0 past the rpm limit -- on

    omega_m_est = omega_w * dp.chain.ratio;                    % motor speed = axle speed * gear ratio [rad/s]
    omega_m_rpm_est = abs(omega_m_est) * 60/(2*pi);             % same, in rpm

    T_cmd = Tcmd_fun(t);   % what's being asked for right now
    T_cmd_sat = T_cmd;     % start unclamped, then apply whichever limits are on

    if LIMIT_BY_TORQUE_RATING
        T_max = min(dp.motor.T_pk, dp.motor.K_T * dp.motor.I_pk);   % motor/inverter rated ceiling
        T_cmd_sat = max(-T_max, min(T_max, T_cmd_sat));              % clamp to +/-T_max
    end

    if LIMIT_BY_HARD_LIMIT
        % Clamp equation: never let commanded torque exceed +/- the hard limit
        T_cmd_sat = max(-dp.motor.T_hard_limit, min(dp.motor.T_hard_limit, T_cmd_sat));
    end

    if LIMIT_BY_SPEED_CURVE
        % Clamp equation: never exceed the peak torque available at the
        % CURRENT motor speed (see motor_peak_torque below + the 3-point
        % breakpoint table in parameters.m).
        T_curve_max = motor_peak_torque(omega_m_rpm_est, dp);
        T_cmd_sat = max(-T_curve_max, min(T_curve_max, T_cmd_sat));
    end

    if LIMIT_BY_MOTOR_SPEED
        omega_max_rad = dp.motor.omega_max_rpm * 2*pi/60;   % rpm limit -> rad/s
        if abs(omega_m_est) >= omega_max_rad
            T_cmd_sat = 0;   % motor is at its speed limit, no more torque available
        end
    end

    dT_m = (T_cmd_sat - T_m) / dp.motor.tau_m;   % first-order lag: torque chases the clamped command

    %% 2. Chain + differential -> axle torque
    eta_chain = default_if_nan(dp.chain.eta, 0.97, 'chain.eta');   % chain efficiency (placeholder if unmeasured)
    eta_diff  = default_if_nan(dp.diff.eta,  0.95, 'diff.eta');    % diff efficiency (placeholder if unmeasured)

    T_axle = T_m * dp.chain.ratio * eta_chain * eta_diff;   % motor torque scaled up by the gear ratio, minus losses

    %% 3. Axle rotational equation of motion
    J_w_each = default_if_nan(dp.wheel.J_w, 0.35, 'wheel.J_w (per wheel)');   % per-wheel inertia (placeholder if unmeasured)
    J_eff = 2*J_w_each + dp.motor.J_r_refl;   % both wheels + motor rotor inertia reflected through the gear ratio

    T_load = Tload_fun(t, omega_w);          % resisting torque (0 for free-spin)
    domega_w = (T_axle - T_load) / J_eff;     % Newton's 2nd law for rotation: torque / inertia = angular accel

    dxdt = [dT_m; domega_w];   % hand both derivatives back to the ODE solver
end

function T_max = motor_peak_torque(rpm, dp)
    % Peak torque available at a given motor speed [rpm] -- piecewise-linear
    % lookup through dp.motor.torque_curve_rpm/_Nm (set in parameters.m),
    % held flat beyond the table's first/last rpm.
    rpm_bp = dp.motor.torque_curve_rpm;
    Nm_bp  = dp.motor.torque_curve_Nm;
    rpm_clamped = max(rpm_bp(1), min(rpm_bp(end), abs(rpm)));
    T_max = interp1(rpm_bp, Nm_bp, rpm_clamped, 'linear');
end

function val = default_if_nan(val, default, name)
    % Substitutes a placeholder value when the real one is still NaN in
    % parameters.m, warning once per name per session (not once per call --
    % ode45 calls this hundreds of times per simulation).
    persistent warned
    if isempty(warned)
        warned = {};
    end
    if isnan(val)
        if ~any(strcmp(warned, name))
            warning('driveline_dynamics:defaulted', ...
                '%s is not yet measured (NaN) -- using placeholder %.3g for this run.', ...
                name, default);
            warned{end+1} = name;
        end
        val = default;
    end
end