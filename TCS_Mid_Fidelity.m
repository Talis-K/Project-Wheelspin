%% TCS Mid-Fidelity Real-Time Simulation
% Single-wheel traction-control model
%
% UI controls:
%   - Accelerator slider: 0-100 %
%   - TCS ON/OFF switch
%   - Vehicle speed gauge
%   - Live plots:
%       Vehicle speed
%       Wheel slip
%       Motor torque
%       Tyre force
%
% The simulation runs continuously while the UI is open.
% Switching TCS OFF pauses the simulation.

clear;
clc;
close all;

%% ========================================================================
%  UI
% =========================================================================

app = TCS_UI;


%% ========================================================================
%  VEHICLE SPECIFICATION
% =========================================================================

P.mass = 1600;                 % Vehicle mass [kg]

P.wheelRadius = 0.31;         % Effective tyre radius [m]

P.finalDriveRatio = 3.5;      % Final drive ratio

P.drivetrainEfficiency = 0.95;

P.gravity = 9.81;              % [m/s^2]

P.airDensity = 1.225;          % [kg/m^3]

P.frontalArea = 2.2;           % [m^2]

P.dragCoefficient = 0.30;

P.rollingResistance = 0.015;


%% ========================================================================
%  LOAD DISTRIBUTION
% =========================================================================

% Rear-wheel drive.
% One driven rear wheel represents half of the rear axle.

P.rearWeightDistribution = 0.55;

P.rearAxleLoad = ...
    P.mass * P.gravity * P.rearWeightDistribution;

P.drivenWheelLoad = P.rearAxleLoad / 2;


%% ========================================================================
%  WHEEL / DRIVETRAIN
% =========================================================================

P.wheelInertia = 0.90;         % Wheel inertia [kg m^2]

P.motorTimeConstant = 0.020;   % Motor response time [s]


%% ========================================================================
%  MOTOR DATA
% =========================================================================

P.motorPeakTorque = 220;       % Peak torque [Nm]

P.motorContinuousTorque = 112; % Continuous torque [Nm]

P.motorPeakPower = 104e3;      % Peak power [W]

P.motorContinuousPower = 64e3; % Continuous power [W]

P.motorPowerRPM = 4500;

P.motorMaximumRPM = 5170;

P.motorBaseSpeedRPM = 6500;


%% ========================================================================
%  TYRE / ROAD PARAMETERS
% =========================================================================

P.muDry = 1.0;

P.muLow = 0.35;

P.targetSlip = 0.12;

P.activationSlip = 0.13;

P.lowSpeedThreshold = 1.5;

P.minimumSlipSpeed = 0.5;


%% ========================================================================
%  CONTROLLER PARAMETERS
% =========================================================================

P.slipFilterTime = 0.020;

P.Kp = 500;

P.Ki = 1500;

P.torqueRateIncrease = 5000;   % Nm/s

P.torqueRateDecrease = 15000;  % Nm/s


%% ========================================================================
%  SIMULATION CONFIGURATION
% =========================================================================

% Physics timestep
P.dt = 0.0005;                 % 0.5 ms

% Controller update rate
P.controllerSampleTime = 0.002;    % 2 ms

% Display update rate
P.displaySampleTime = 0.033;       % approximately 30 Hz

% Number of physics steps between controller updates
P.controllerStride = ...
    round(P.controllerSampleTime / P.dt);

% Number of physics steps between display updates
P.displayStride = ...
    round(P.displaySampleTime / P.dt);

% Rolling graph history
P.historyTime = 15.0;

P.historySteps = ...
    round(P.historyTime / P.dt);

assert(P.controllerStride >= 1);
assert(P.displayStride >= 1);


%% ========================================================================
%  DRIVER / TORQUE SETTINGS
% =========================================================================

% Feed-forward torque used by the driver demand.
%
% The accelerator slider determines the requested torque.

P.driverTorqueGain = P.motorPeakTorque;

% Small launch torque cap to avoid an unrealistic initial torque spike.

P.launchTorqueCap = 160;


%% ========================================================================
%  LIVE PLOT SETUP
% =========================================================================

plotFigure = figure( ...
    'Name', 'TCS Real-Time Simulation', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [80 80 1100 700]);

tiledlayout(plotFigure, 2, 2);


%% Vehicle speed ----------------------------------------------------------

axSpeed = nexttile;

speedLine = plot( ...
    axSpeed, ...
    nan, ...
    nan, ...
    'LineWidth', 1.5);

grid(axSpeed, 'on');

xlabel(axSpeed, 'Time [s]');
ylabel(axSpeed, 'Vehicle Speed [km/h]');

title(axSpeed, 'Vehicle Speed');

xlim(axSpeed, [0 P.historyTime]);

ylim(axSpeed, [0 100]);


%% Slip -------------------------------------------------------------------

axSlip = nexttile;

slipLine = plot( ...
    axSlip, ...
    nan, ...
    nan, ...
    'LineWidth', 1.5);

hold(axSlip, 'on');

targetSlipLine = plot( ...
    axSlip, ...
    nan, ...
    nan, ...
    '--', ...
    'LineWidth', 1.0);

grid(axSlip, 'on');

xlabel(axSlip, 'Time [s]');
ylabel(axSlip, 'Slip');

title(axSlip, 'Wheel Slip');

legend( ...
    axSlip, ...
    {'Measured Slip', 'Target Slip'}, ...
    'Location', 'best');

xlim(axSlip, [0 P.historyTime]);


%% Motor torque -----------------------------------------------------------

axTorque = nexttile;

torqueLine = plot( ...
    axTorque, ...
    nan, ...
    nan, ...
    'LineWidth', 1.5);

hold(axTorque, 'on');

requestedTorqueLine = plot( ...
    axTorque, ...
    nan, ...
    nan, ...
    '--', ...
    'LineWidth', 1.0);

grid(axTorque, 'on');

xlabel(axTorque, 'Time [s]');
ylabel(axTorque, 'Torque [Nm]');

title(axTorque, 'Motor Torque');

legend( ...
    axTorque, ...
    {'Actual Motor Torque', 'Driver Request'}, ...
    'Location', 'best');

xlim(axTorque, [0 P.historyTime]);


%% Tyre force -------------------------------------------------------------

axForce = nexttile;

forceLine = plot( ...
    axForce, ...
    nan, ...
    nan, ...
    'LineWidth', 1.5);

grid(axForce, 'on');

xlabel(axForce, 'Time [s]');
ylabel(axForce, 'Force [N]');

title(axForce, 'Driven Tyre Force');

xlim(axForce, [0 P.historyTime]);


%% ========================================================================
%  INITIAL STATE
% =========================================================================

state.vehicleSpeed = 0;

state.wheelSpeed = 0;

state.motorTorque = 0;

state.torqueCommand = 0;

state.desiredTorque = 0;

state.filteredSlipMemory = 0;

state.integratorMemory = 0;

state.commandMemory = 0;

state.desiredMemory = 0;

state.activeMemory = false;

state.currentTime = 0;

state.stepCount = 0;


%% ========================================================================
%  FILTER INITIALISATION
% =========================================================================

state.filterAlpha = ...
    P.dt / (P.slipFilterTime + P.dt);


%% ========================================================================
%  ROLLING HISTORY
% =========================================================================

historyTime = nan(P.historySteps, 1);

historySpeed = nan(P.historySteps, 1);

historySlip = nan(P.historySteps, 1);

historyTorque = nan(P.historySteps, 1);

historyRequestedTorque = nan(P.historySteps, 1);

historyTyreForce = nan(P.historySteps, 1);


%% ========================================================================
%  REAL-TIME LOOP
% =========================================================================

wallClock = tic;

while isvalid(app.UIFigure)

    %% --------------------------------------------------------------------
    %  Check simulation switch
    % ---------------------------------------------------------------------

    if strcmp(app.Switch.Value, 'Off')

        drawnow;

        pause(0.02);

        % Reset wall-clock reference so that the simulation does not
        % attempt to catch up after the user turns it back on.

        wallClock = tic;

        continue;

    end


    %% --------------------------------------------------------------------
    %  Read accelerator
    % ---------------------------------------------------------------------

    pedal = app.AccelerationsliderSlider.Value / 100;


    %% --------------------------------------------------------------------
    %  Road friction
    % ---------------------------------------------------------------------
    %
    % Keep the existing low-friction road event:
    %
    %   0 - 2.2 s       Dry road
    %   2.2 - 3.0 s     Low friction
    %   > 3.0 s         Dry road
    %

    if state.currentTime >= 2.2 && ...
            state.currentTime < 3.0

        roadMu = P.muLow;

    else

        roadMu = P.muDry;

    end


    %% --------------------------------------------------------------------
    %  One physics step
    % ---------------------------------------------------------------------

    [state, output] = ...
        simulateVehicleStep( ...
        state, ...
        pedal, ...
        roadMu, ...
        P);


    %% --------------------------------------------------------------------
    %  Update simulation time
    % ---------------------------------------------------------------------

    state.currentTime = ...
        state.currentTime + P.dt;

    state.stepCount = ...
        state.stepCount + 1;


    %% --------------------------------------------------------------------
    %  Update rolling history
    % ---------------------------------------------------------------------

    if state.stepCount <= P.historySteps

        index = state.stepCount;

    else

        % Shift history when buffer is full.

        historyTime(1:end-1) = ...
            historyTime(2:end);

        historySpeed(1:end-1) = ...
            historySpeed(2:end);

        historySlip(1:end-1) = ...
            historySlip(2:end);

        historyTorque(1:end-1) = ...
            historyTorque(2:end);

        historyRequestedTorque(1:end-1) = ...
            historyRequestedTorque(2:end);

        historyTyreForce(1:end-1) = ...
            historyTyreForce(2:end);

        index = P.historySteps;

    end


    historyTime(index) = state.currentTime;

    historySpeed(index) = ...
        state.vehicleSpeed * 3.6;

    historySlip(index) = output.slip;

    historyTorque(index) = ...
        state.motorTorque;

    historyRequestedTorque(index) = ...
        output.requestedTorque;

    historyTyreForce(index) = ...
        output.tyreForce;


    %% --------------------------------------------------------------------
    %  Display update
    % ---------------------------------------------------------------------

    if mod(state.stepCount, P.displayStride) == 0

        valid = ~isnan(historyTime);

        tPlot = historyTime(valid);

        speedPlot = historySpeed(valid);

        slipPlot = historySlip(valid);

        torquePlot = historyTorque(valid);

        requestedTorquePlot = ...
            historyRequestedTorque(valid);

        forcePlot = ...
            historyTyreForce(valid);


        %% Vehicle speed

        set( ...
            speedLine, ...
            'XData', tPlot, ...
            'YData', speedPlot);


        %% Slip

        set( ...
            slipLine, ...
            'XData', tPlot, ...
            'YData', slipPlot);

        set( ...
            targetSlipLine, ...
            'XData', tPlot, ...
            'YData', ...
            P.targetSlip * ones(size(tPlot)));


        %% Torque

        set( ...
            torqueLine, ...
            'XData', tPlot, ...
            'YData', torquePlot);

        set( ...
            requestedTorqueLine, ...
            'XData', tPlot, ...
            'YData', requestedTorquePlot);


        %% Tyre force

        set( ...
            forceLine, ...
            'XData', tPlot, ...
            'YData', forcePlot);


        %% Rolling 15-second window

        if state.currentTime <= P.historyTime

            xStart = 0;

        else

            xStart = ...
                state.currentTime - P.historyTime;

        end

        xEnd = ...
            max(P.historyTime, state.currentTime);


        xlim(axSpeed, [xStart xEnd]);

        xlim(axSlip, [xStart xEnd]);

        xlim(axTorque, [xStart xEnd]);

        xlim(axForce, [xStart xEnd]);


        %% Gauge

        speedKmh = ...
            state.vehicleSpeed * 3.6;

        app.Gauge.Value = ...
            min(max(speedKmh, 0), 100);


        %% Update UI

        drawnow limitrate;

    end


    %% --------------------------------------------------------------------
    %  Real-time pacing
    % ---------------------------------------------------------------------

    elapsedWallTime = toc(wallClock);

    waitTime = ...
        state.currentTime - elapsedWallTime;

    if waitTime > 0

        pause(waitTime);

    end

end


%% ========================================================================
%  SINGLE PHYSICS STEP
% =========================================================================

function [state, output] = ...
    simulateVehicleStep(state, pedal, roadMu, P)

% -------------------------------------------------------------------------
% Motor speed
% -------------------------------------------------------------------------

motorRPM = ...
    state.wheelSpeed * ...
    P.finalDriveRatio * ...
    60 / (2*pi);


% -------------------------------------------------------------------------
% Motor torque limit
% -------------------------------------------------------------------------

motorTorqueLimit = ...
    calculateMotorTorqueLimit( ...
    motorRPM, ...
    P);


% -------------------------------------------------------------------------
% Driver requested torque
% -------------------------------------------------------------------------

requestedTorque = ...
    pedal * P.driverTorqueGain;

requestedTorque = ...
    min(requestedTorque, motorTorqueLimit);


% -------------------------------------------------------------------------
% Calculate slip
% -------------------------------------------------------------------------

slip = ...
    calculateSlip( ...
    state.wheelSpeed, ...
    state.vehicleSpeed, ...
    P);


% -------------------------------------------------------------------------
% Controller update
% -------------------------------------------------------------------------

controllerUpdate = ...
    mod(state.stepCount, P.controllerStride) == 0;


if controllerUpdate

    % Low-pass filter the slip signal.

    state.filteredSlipMemory = ...
        state.filteredSlipMemory + ...
        state.filterAlpha * ...
        (slip - state.filteredSlipMemory);


    filteredSlip = ...
        state.filteredSlipMemory;


    % Default desired torque is the driver's request.

    desiredTorque = requestedTorque;


    % ---------------------------------------------------------------------
    % Very low accelerator input
    % ---------------------------------------------------------------------

    if pedal < 0.02

        desiredTorque = 0;

        state.integratorMemory = 0;

        state.activeMemory = false;


    % ---------------------------------------------------------------------
    % Low-speed launch limitation
    % ---------------------------------------------------------------------

    elseif state.vehicleSpeed < P.lowSpeedThreshold

        desiredTorque = ...
            min(requestedTorque, P.launchTorqueCap);

        state.integratorMemory = 0;

        state.activeMemory = false;


    % ---------------------------------------------------------------------
    % TCS control
    % ---------------------------------------------------------------------

    elseif filteredSlip > P.activationSlip

        state.activeMemory = true;

        slipError = ...
            filteredSlip - P.targetSlip;

        state.integratorMemory = ...
            state.integratorMemory + ...
            slipError * ...
            P.controllerSampleTime;

        % Anti-windup

        state.integratorMemory = ...
            max( ...
            min(state.integratorMemory, 0.25), ...
            -0.25);


        % PI torque correction

        torqueCorrection = ...
            P.Kp * slipError + ...
            P.Ki * state.integratorMemory;


        desiredTorque = ...
            requestedTorque - torqueCorrection;


    else

        % Below activation threshold.

        state.activeMemory = false;

        % Gradually release the integrator.

        state.integratorMemory = ...
            0.95 * state.integratorMemory;

        desiredTorque = requestedTorque;

    end


    % Limit desired torque.

    desiredTorque = ...
        max(0, min(desiredTorque, motorTorqueLimit));


    state.desiredTorque = desiredTorque;

end


% -------------------------------------------------------------------------
% Use previous controller command between controller updates
% -------------------------------------------------------------------------

desiredTorque = state.desiredTorque;


% -------------------------------------------------------------------------
% Torque slew-rate limitation
% -------------------------------------------------------------------------

state.torqueCommand = ...
    applyRateLimit( ...
    state.torqueCommand, ...
    desiredTorque, ...
    P);


% -------------------------------------------------------------------------
% Tyre / longitudinal forces
% -------------------------------------------------------------------------

[tyreForce, aerodynamicDrag, rollingForce] = ...
    longitudinalForces( ...
    state.vehicleSpeed, ...
    state.wheelSpeed, ...
    state.torqueCommand, ...
    roadMu, ...
    P);


% -------------------------------------------------------------------------
% Motor dynamics
% -------------------------------------------------------------------------

motorTorqueDerivative = ...
    (state.torqueCommand - state.motorTorque) / ...
    P.motorTimeConstant;

state.motorTorque = ...
    state.motorTorque + ...
    motorTorqueDerivative * P.dt;


% -------------------------------------------------------------------------
% Motor torque cannot be negative
% -------------------------------------------------------------------------

state.motorTorque = ...
    max(0, state.motorTorque);


% -------------------------------------------------------------------------
% Wheel dynamics
% -------------------------------------------------------------------------

wheelTorque = ...
    state.motorTorque * ...
    P.finalDriveRatio * ...
    P.drivetrainEfficiency;


wheelAngularAcceleration = ...
    (wheelTorque - tyreForce * P.wheelRadius) / ...
    P.wheelInertia;


% -------------------------------------------------------------------------
% Vehicle dynamics
% -------------------------------------------------------------------------

totalResistingForce = ...
    aerodynamicDrag + rollingForce;


vehicleAcceleration = ...
    (tyreForce - totalResistingForce) / ...
    P.mass;


% -------------------------------------------------------------------------
% Integrate wheel speed
% -------------------------------------------------------------------------

state.wheelSpeed = ...
    state.wheelSpeed + ...
    wheelAngularAcceleration * P.dt;


% -------------------------------------------------------------------------
% Integrate vehicle speed
% -------------------------------------------------------------------------

state.vehicleSpeed = ...
    state.vehicleSpeed + ...
    vehicleAcceleration * P.dt;


% -------------------------------------------------------------------------
% Prevent negative speeds
% -------------------------------------------------------------------------

state.wheelSpeed = ...
    max(0, state.wheelSpeed);

state.vehicleSpeed = ...
    max(0, state.vehicleSpeed);


% -------------------------------------------------------------------------
% Recalculate output slip
% -------------------------------------------------------------------------

output.slip = ...
    calculateSlip( ...
    state.wheelSpeed, ...
    state.vehicleSpeed, ...
    P);


output.filteredSlip = ...
    state.filteredSlipMemory;


output.tyreForce = tyreForce;

output.normalLoad = ...
    P.drivenWheelLoad;

output.acceleration = ...
    vehicleAcceleration;

output.requestedTorque = ...
    requestedTorque;

output.motorTorqueLimit = ...
    motorTorqueLimit;

output.controllerActive = ...
    state.activeMemory;

end


%% ========================================================================
%  SLIP CALCULATION
% =========================================================================

function slip = ...
    calculateSlip(wheelSpeed, vehicleSpeed, P)

wheelRoadSpeed = ...
    wheelSpeed * P.wheelRadius;


denominator = ...
    max(abs(vehicleSpeed), P.minimumSlipSpeed);


slip = ...
    (wheelRoadSpeed - vehicleSpeed) / denominator;


% Avoid extreme numerical values at standstill.

slip = ...
    max(-1, min(slip, 2));

end


%% ========================================================================
%  MOTOR TORQUE LIMIT
% =========================================================================

function torqueLimit = ...
    calculateMotorTorqueLimit(motorRPM, P)

motorRPM = ...
    max(0, motorRPM);


% Peak torque region

if motorRPM <= P.motorPowerRPM

    torqueLimit = ...
        P.motorPeakTorque;

else

    motorSpeedRad = ...
        motorRPM * 2*pi/60;

    torqueLimit = ...
        P.motorPeakPower / ...
        max(motorSpeedRad, 1);

end


% Motor maximum speed limit

if motorRPM >= P.motorMaximumRPM

    torqueLimit = 0;

end


torqueLimit = ...
    max(0, min(torqueLimit, P.motorPeakTorque));

end


%% ========================================================================
%  LONGITUDINAL FORCES
% =========================================================================

function [tyreForce, aerodynamicDrag, rollingForce] = ...
    longitudinalForces( ...
    vehicleSpeed, ...
    wheelSpeed, ...
    wheelTorque, ...
    roadMu, ...
    P)

% -------------------------------------------------------------------------
% Calculate slip
% -------------------------------------------------------------------------

slip = ...
    calculateSlip( ...
    wheelSpeed, ...
    vehicleSpeed, ...
    P);


% -------------------------------------------------------------------------
% Simple tyre friction model
% -------------------------------------------------------------------------

% The tyre force increases approximately linearly with slip until the
% available friction limit is reached.

tyreForceIdeal = ...
    P.drivenWheelLoad * ...
    roadMu * ...
    tanh(8 * max(slip, 0));


% -------------------------------------------------------------------------
% Available friction force
% -------------------------------------------------------------------------

maximumTyreForce = ...
    roadMu * P.drivenWheelLoad;


tyreForce = ...
    min(tyreForceIdeal, maximumTyreForce);


% Prevent negative driving force.

tyreForce = ...
    max(0, tyreForce);


% -------------------------------------------------------------------------
% Aerodynamic drag
% -------------------------------------------------------------------------

aerodynamicDrag = ...
    0.5 * ...
    P.airDensity * ...
    P.frontalArea * ...
    P.dragCoefficient * ...
    vehicleSpeed^2;


% -------------------------------------------------------------------------
% Rolling resistance
% -------------------------------------------------------------------------

rollingForce = ...
    P.rollingResistance * ...
    P.mass * ...
    P.gravity;


% At zero speed, avoid applying rolling resistance against a stationary
% vehicle.

if vehicleSpeed <= 0 && tyreForce <= rollingForce

    rollingForce = 0;

end

end


%% ========================================================================
%  TORQUE RATE LIMITER
% =========================================================================

function newCommand = ...
    applyRateLimit(previousCommand, desiredCommand, P)

difference = ...
    desiredCommand - previousCommand;


if difference >= 0

    maximumChange = ...
        P.torqueRateIncrease * P.dt;

else

    maximumChange = ...
        P.torqueRateDecrease * P.dt;

end


if abs(difference) <= maximumChange

    newCommand = desiredCommand;

else

    newCommand = ...
        previousCommand + ...
        sign(difference) * maximumChange;

end


newCommand = ...
    max(0, newCommand);

end