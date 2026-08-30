%% run_driveline_demo.m
% Runs driveline_dynamics.m against a 100 Nm step torque command with no
% load (free-spin), and plots motor torque, motor speed, and axle speed.

run('driveline_params.m');   % loads parameters.m and builds the "driveline" struct

T_cmd_fun  = @(t) 150 * (t >= 0.05);   % [Nm] step to 150 Nm at t = 50 ms -- above every limit,
                                         % so T_m just traces whichever limit is lowest at each instant
T_load_fun = @(t, w) 0;                 % free-spin: no resisting torque

x0 = [0; 0];        % start at rest: zero motor torque, zero axle speed
tspan = [0 3];       % simulate 3 seconds

[t, x] = ode45(@(t, x) driveline_dynamics(t, x, T_cmd_fun, T_load_fun, driveline), tspan, x0);

T_m     = x(:,1);                                   % motor torque over time [Nm]
omega_w = x(:,2);                                   % axle/wheel speed over time [rad/s]
omega_m = omega_w * driveline.chain.ratio;          % motor speed = axle speed * gear ratio [rad/s]
omega_m_rpm = omega_m * 60/(2*pi);                  % convert to rpm for plotting

% Same lookup as the local motor_peak_torque() function inside
% driveline_dynamics.m, duplicated here since it's not visible outside
% that file -- the speed-curve ceiling, evaluated at each instant's
% actual motor speed.
rpm_bp = driveline.motor.torque_curve_rpm;
Nm_bp  = driveline.motor.torque_curve_Nm;
rpm_clamped = max(rpm_bp(1), min(rpm_bp(end), omega_m_rpm));
T_curve = interp1(rpm_bp, Nm_bp, rpm_clamped);

T_hard  = driveline.motor.T_hard_limit * ones(size(t));   % flat 80 Nm reference

figure;

subplot(3,1,1);
plot(t, T_m, t, T_curve, '--', t, T_hard, ':');
xlabel('Time [s]'); ylabel('Motor torque [Nm]');
title('Torque response to a 150 Nm command -- traces min(hard limit, speed curve)');
legend('T_m (simulated)', 'peak-torque curve at current rpm', '80 Nm hard limit', 'Location', 'southeast');
grid on;

subplot(3,1,2);
plot(t, omega_m_rpm, t, driveline.motor.omega_max_rpm * ones(size(t)), '--r');
xlabel('Time [s]'); ylabel('Motor speed [rpm]');
title('Motor speed vs its rpm limit');
legend('\omega_m (simulated)', sprintf('\\omega_{max} (%.0f rpm)', driveline.motor.omega_max_rpm), ...
    'Location', 'southeast');
grid on;

subplot(3,1,3);
plot(t, omega_w * 60/(2*pi));
xlabel('Time [s]'); ylabel('Axle speed [rpm]');
title('Axle speed (lumped, both rear wheels)');
grid on;