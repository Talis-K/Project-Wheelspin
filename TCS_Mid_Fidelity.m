%% SINGLE-WHEEL TRACTION-CONTROL SIMULATION
% Simulation-only model based on VehicleSpecSheet2025.pdf.
%
% The model represents one rear driven wheel. The other rear wheel is
% assumed to behave identically and receive 50% of the differential torque.
%
% Model equations:
%   Slip ratio:       kappa = (r*omega - Vx)/max(|Vx|,Vepsilon)
%   Wheel dynamics:   J*d(omega)/dt = Twheel - Fx*r
%   Vehicle dynamics: m*d(Vx)/dt = sum(Fx) - resistance
%   Wheel torque:     Twheel = Tmotor*gearRatio*efficiency*torqueSplit
%
% Parameters not included in the vehicle specification sheet are clearly
% marked as modelling assumptions.

clear;
clc;
close all;

%% Vehicle specification data

P.g = 9.81;

% Mass data
P.massWithoutDriver       = 242;       % kg
P.driverMass              = 68;        % kg
P.vehicleMass             = P.massWithoutDriver + P.driverMass;
P.frontWeightFraction     = 0.50;
P.rearWeightFraction      = 1 - P.frontWeightFraction;
P.drivenWheelCount        = 2;

% Each rear-wheel model must accelerate half of the complete vehicle.
P.equivalentMass          = P.vehicleMass/P.drivenWheelCount;

% Static vertical load on one rear wheel
P.staticNormalLoad = ...
    P.vehicleMass*P.g*P.rearWeightFraction/P.drivenWheelCount;

% Vehicle geometry
P.wheelbase               = 1550e-3;   % m
P.cgHeight                = 289e-3;    % m
P.frontTrack              = 1200e-3;   % m
P.rearTrack               = 1150e-3;   % m

% Tyre data: Goodyear D2704 20.0x7.0-13
% The spec sheet lists a measured diameter of 20.4 inches.
P.tyreDiameter            = 20.4*0.0254;
P.tyreRadius              = P.tyreDiameter/2;

% Motor data: EMRAX 228 High Voltage
P.motorPeakTorque         = 220;       % Nm
P.motorContinuousTorque   = 112;       % Nm
P.motorPeakPower          = 104e3;     % W
P.motorContinuousPower    = 64e3;      % W
P.motorPowerRPM           = 4500;      % rpm
P.motorMaximumRPM         = 5170;      % rpm
P.motorBaseSpeedSheet     = 6500;      % rpm, sheet value

% Electrical power limits
P.inverterPeakPower       = 85e3;      % W
P.accumulatorPeakPower    = 80e3;      % W
P.systemPeakPower = min([P.motorPeakPower, ...
                         P.inverterPeakPower, ...
                         P.accumulatorPeakPower]);

% Drivetrain
P.gearRatio               = 6.0;
P.differentialTorqueSplit = 1/P.drivenWheelCount;

%% Modelling assumptions

P.drivetrainEfficiency    = 0.95;
P.wheelInertia            = 0.90;      % kg*m^2
P.motorTimeConstant       = 0.020;     % s
P.rollingResistance       = 0.015;
P.rollSpeedSmoothing      = 0.20;      % m/s

% No aerodynamic drag is included because the specification sheet lists
% the aerodynamic configuration as N/A.

% Pacejka-style longitudinal tyre-model assumptions
P.muDry                   = 1.20;
P.muLow                   = 0.70;
P.magicB                  = 10.0;
P.magicC                  = 1.90;
P.magicE                  = 0.97;

%% Traction-control assumptions

P.targetSlip              = 0.12;
P.activationSlip          = 0.13;
P.lowSpeedThreshold       = 1.50;      % m/s
P.slipDenominatorMinimum  = 0.50;      % m/s
P.slipFilterTimeConstant  = 0.020;     % s

% PI torque controller
P.Kp                      = 1000;      % Nm/slip
P.Ki                      = 1200;      % Nm/(slip*s)
P.Kaw                     = 1/P.Kp;
P.integratorMinimum       = -0.20;
P.integratorMaximum       = 0.20;

% Torque slew limits
P.torqueCutRate           = 8000;      % Nm/s
P.torqueReturnRate        = 300;       % Nm/s

%% Simulation configuration

P.dt                      = 0.0005;    % plant time step
P.controllerSampleTime    = 0.002;     % controller sample time
P.stopTime                = 4.0;
P.controllerStride = round(P.controllerSampleTime/P.dt);

assert(abs(P.controllerStride*P.dt - ...
    P.controllerSampleTime) < 1e-12, ...
    'Controller sample time must be an integer multiple of dt.');

%% Calculate nominal controller feed-forward torque

[FxTarget, ~, accelerationTarget] = ...
    longitudinalForces(P.targetSlip, 10, P.muDry, P);

targetWheelAcceleration = ...
    (1 + P.targetSlip)*accelerationTarget/P.tyreRadius;

targetWheelTorque = ...
    FxTarget*P.tyreRadius + ...
    P.wheelInertia*targetWheelAcceleration;

P.feedForwardTorque = targetWheelTorque/( ...
    P.gearRatio*P.drivetrainEfficiency* ...
    P.differentialTorqueSplit);

% Conservative torque cap while slip ratio is unreliable near standstill
P.launchTorqueCap = min(0.90*P.feedForwardTorque, ...
                        P.motorPeakTorque);

%% Driver and road inputs

time = (0:P.dt:P.stopTime).';

% Full-throttle launch with a pedal ramp
pedal = (time - 0.10)/(0.80 - 0.10);
pedal = min(max(pedal,0),1);

% Temporary low-friction surface
roadMu = P.muDry*ones(size(time));
roadMu(time >= 2.20 & time < 3.00) = P.muLow;


%% UI figure

app = TCS_UI;


%% Wait for simulation switch

while strcmp(app.Switch.Value, 'Off')
    drawnow;
    pause(0.05);
end

%% Run simulations

TCS_OFF = simulateVehicle(time, pedal, roadMu, P, false);
TCS_ON  = simulateVehicle(time, pedal, roadMu, P, true);

%% Performance calculations

validOff = TCS_OFF.vehicleSpeed > 0.5;
validOn  = TCS_ON.vehicleSpeed  > 0.5;

peakSlipOff = max(TCS_OFF.slip(validOff));
peakSlipOn  = max(TCS_ON.slip(validOn));

slipPowerOff = abs(TCS_OFF.tyreForce .* ...
    (P.tyreRadius*TCS_OFF.wheelSpeed - TCS_OFF.vehicleSpeed));

slipPowerOn = abs(TCS_ON.tyreForce .* ...
    (P.tyreRadius*TCS_ON.wheelSpeed - TCS_ON.vehicleSpeed));

slipEnergyOff = trapz(time,slipPowerOff);
slipEnergyOn  = trapz(time,slipPowerOn);

activeIndex = find(TCS_ON.controllerActive,1,'first');

if isempty(activeIndex)
    activationTime = NaN;
else
    activationTime = time(activeIndex);
end

speedAtMaximumPower = ...
    (P.motorPowerRPM*2*pi/60)/P.gearRatio * ...
    P.tyreRadius*3.6;

maximumGearedSpeed = ...
    (P.motorMaximumRPM*2*pi/60)/P.gearRatio * ...
    P.tyreRadius*3.6;

fprintf('\nSINGLE-WHEEL TRACTION-CONTROL RESULTS\n');
fprintf('Vehicle mass with driver:       %.1f kg\n',P.vehicleMass);
fprintf('Equivalent mass per model:      %.1f kg\n',P.equivalentMass);
fprintf('Static rear-wheel normal load:  %.1f N\n',P.staticNormalLoad);
fprintf('Effective tyre radius:          %.4f m\n',P.tyreRadius);
fprintf('Nominal TCS feed-forward torque: %.1f Nm\n',P.feedForwardTorque);
fprintf('Low-speed launch torque cap:    %.1f Nm\n',P.launchTorqueCap);
fprintf('Vehicle speed at 4500 rpm:      %.1f km/h\n',speedAtMaximumPower);
fprintf('Maximum geared vehicle speed:   %.1f km/h\n\n',maximumGearedSpeed);

fprintf('Peak slip, TCS OFF:             %.3f\n',peakSlipOff);
fprintf('Peak slip, TCS ON:              %.3f\n',peakSlipOn);
fprintf('Final speed, TCS OFF:           %.2f km/h\n', ...
    TCS_OFF.vehicleSpeed(end)*3.6);
fprintf('Final speed, TCS ON:            %.2f km/h\n', ...
    TCS_ON.vehicleSpeed(end)*3.6);
fprintf('Slip energy, TCS OFF:           %.1f J\n',slipEnergyOff);
fprintf('Slip energy, TCS ON:            %.1f J\n',slipEnergyOn);
fprintf('TCS activation time:            %.3f s\n',activationTime);

%% Results plots

figure('Color','w','Position',[100 100 1200 720]);

subplot(2,2,1);
plot(time,TCS_OFF.vehicleSpeed*3.6,'--','LineWidth',1.5);
hold on;
plot(time,TCS_ON.vehicleSpeed*3.6,'LineWidth',1.8);
grid on;
xlabel('Time (s)');
ylabel('Vehicle speed (km/h)');
title('Vehicle speed');
legend('TCS OFF','TCS ON','Location','northwest');

subplot(2,2,2);
plot(time,TCS_OFF.slip,'--','LineWidth',1.3);
hold on;
plot(time,TCS_ON.slip,'LineWidth',1.8);
plot(time,P.targetSlip*ones(size(time)),':','LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Longitudinal slip ratio');
title('Wheel slip — display limited to \kappa = 1');
legend('TCS OFF','TCS ON','Target','Location','northeast');
ylim([-0.05 1]);

subplot(2,2,3);
plot(time,TCS_ON.requestedTorque,':','LineWidth',1.3);
hold on;
plot(time,TCS_OFF.motorTorque,'--','LineWidth',1.3);
plot(time,TCS_ON.motorTorque,'LineWidth',1.8);
grid on;
xlabel('Time (s)');
ylabel('Motor torque (Nm)');
title('Motor torque intervention');
legend('Driver request','TCS OFF','TCS ON','Location','best');

subplot(2,2,4);
plot(time,TCS_OFF.tyreForce/1000,'--','LineWidth',1.3);
hold on;
plot(time,TCS_ON.tyreForce/1000,'LineWidth',1.8);
grid on;
xlabel('Time (s)');
ylabel('Longitudinal tyre force (kN)');
title('Force generated by one rear tyre');
legend('TCS OFF','TCS ON','Location','best');

if exist('sgtitle','file') == 2
    sgtitle('E59 Single-Wheel Traction-Control Simulation');
end

%% Local functions

function R = simulateVehicle(time,pedal,roadMu,P,tcsEnabled)

    numberOfSteps = length(time);

    vehicleSpeed       = zeros(numberOfSteps,1);
    wheelSpeed         = zeros(numberOfSteps,1);
    motorTorque        = zeros(numberOfSteps,1);
    torqueCommand      = zeros(numberOfSteps,1);
    desiredTorque      = zeros(numberOfSteps,1);
    requestedTorque    = zeros(numberOfSteps,1);
    motorTorqueLimit   = zeros(numberOfSteps,1);
    slip               = zeros(numberOfSteps,1);
    filteredSlip       = zeros(numberOfSteps,1);
    tyreForce          = zeros(numberOfSteps,1);
    normalLoad         = zeros(numberOfSteps,1);
    acceleration       = zeros(numberOfSteps,1);
    controllerActive   = false(numberOfSteps,1);

    filteredSlipMemory = 0;
    integratorMemory   = 0;
    commandMemory      = 0;
    desiredMemory      = 0;
    activeMemory       = false;

    filterAlpha = P.controllerSampleTime/ ...
                  P.slipFilterTimeConstant;

    for k = 1:numberOfSteps

        omegaMotor = P.gearRatio*wheelSpeed(k);

        motorTorqueLimit(k) = ...
            calculateMotorTorqueLimit(omegaMotor,P);

        requestedTorque(k) = min( ...
            pedal(k)*P.motorPeakTorque, ...
            motorTorqueLimit(k));

        slip(k) = calculateSlip( ...
            wheelSpeed(k),vehicleSpeed(k),P);

        controllerHit = ...
            mod(k-1,P.controllerStride) == 0;

        if controllerHit

            limitedSlip = min(max(slip(k),-3),3);

            filteredSlipMemory = filteredSlipMemory + ...
                filterAlpha*(limitedSlip-filteredSlipMemory);

            if ~tcsEnabled

                desiredMemory    = requestedTorque(k);
                integratorMemory = 0;
                activeMemory     = false;

            elseif pedal(k) < 0.02

                desiredMemory    = 0;
                integratorMemory = 0;
                activeMemory     = false;

            elseif vehicleSpeed(k) < P.lowSpeedThreshold

                % Slip ratio is unreliable near zero vehicle speed.
                desiredMemory = min(requestedTorque(k), ...
                                    P.launchTorqueCap);

                integratorMemory = 0;
                activeMemory     = false;

            else

                if ~activeMemory && ...
                        filteredSlipMemory > P.activationSlip

                    activeMemory     = true;
                    integratorMemory = 0;
                end

                if activeMemory

                    slipError = ...
                        P.targetSlip-filteredSlipMemory;

                    unsaturatedTorque = ...
                        P.feedForwardTorque + ...
                        P.Kp*slipError + ...
                        P.Ki*integratorMemory;

                    saturatedTorque = min(max( ...
                        unsaturatedTorque,0), ...
                        requestedTorque(k));

                    integratorDerivative = slipError + ...
                        P.Kaw*(saturatedTorque- ...
                               unsaturatedTorque);

                    integratorMemory = integratorMemory + ...
                        P.controllerSampleTime* ...
                        integratorDerivative;

                    integratorMemory = min(max( ...
                        integratorMemory, ...
                        P.integratorMinimum), ...
                        P.integratorMaximum);

                    desiredMemory = saturatedTorque;

                else
                    desiredMemory = requestedTorque(k);
                end
            end
        end

        filteredSlip(k)     = filteredSlipMemory;
        desiredTorque(k)    = desiredMemory;
        controllerActive(k) = activeMemory;

        if tcsEnabled
            commandMemory = applyRateLimit( ...
                desiredMemory,commandMemory,P.dt, ...
                P.torqueCutRate,P.torqueReturnRate);
        else
            commandMemory = desiredMemory;
        end

        torqueCommand(k) = commandMemory;

        [tyreForce(k),normalLoad(k),acceleration(k)] = ...
            longitudinalForces( ...
                slip(k),vehicleSpeed(k),roadMu(k),P);

        if k < numberOfSteps

            appliedMotorCommand = min( ...
                torqueCommand(k),motorTorqueLimit(k));

            motorTorqueDerivative = ...
                (appliedMotorCommand-motorTorque(k))/ ...
                P.motorTimeConstant;

            wheelTorque = motorTorque(k)* ...
                P.gearRatio* ...
                P.drivetrainEfficiency* ...
                P.differentialTorqueSplit;

            wheelAcceleration = ...
                (wheelTorque- ...
                 tyreForce(k)*P.tyreRadius)/ ...
                 P.wheelInertia;

            motorTorque(k+1) = motorTorque(k) + ...
                P.dt*motorTorqueDerivative;

            wheelSpeed(k+1) = wheelSpeed(k) + ...
                P.dt*wheelAcceleration;

            vehicleSpeed(k+1) = vehicleSpeed(k) + ...
                P.dt*acceleration(k);

            motorTorque(k+1)  = min(max( ...
                motorTorque(k+1),0),P.motorPeakTorque);

            wheelSpeed(k+1)   = max(wheelSpeed(k+1),0);
            vehicleSpeed(k+1) = max(vehicleSpeed(k+1),0);
        end
    end

    R.vehicleSpeed     = vehicleSpeed;
    R.wheelSpeed       = wheelSpeed;
    R.motorTorque      = motorTorque;
    R.torqueCommand    = torqueCommand;
    R.desiredTorque    = desiredTorque;
    R.requestedTorque  = requestedTorque;
    R.motorTorqueLimit = motorTorqueLimit;
    R.slip             = slip;
    R.filteredSlip     = filteredSlip;
    R.tyreForce        = tyreForce;
    R.normalLoad       = normalLoad;
    R.acceleration     = acceleration;
    R.controllerActive = controllerActive;
end

function kappa = calculateSlip(wheelSpeed,vehicleSpeed,P)

    denominator = max(abs(vehicleSpeed), ...
                      P.slipDenominatorMinimum);

    kappa = (P.tyreRadius*wheelSpeed-vehicleSpeed)/ ...
            denominator;
end

function torqueLimit = calculateMotorTorqueLimit(omegaMotor,P)

    motorRPM = abs(omegaMotor)*60/(2*pi);

    if motorRPM >= P.motorMaximumRPM
        torqueLimit = 0;
    else
        powerLimitedTorque = ...
            P.systemPeakPower/max(abs(omegaMotor),1);

        torqueLimit = min(P.motorPeakTorque, ...
                          powerLimitedTorque);
    end
end

function [Fx,Fz,ax] = longitudinalForces( ...
    kappa,vehicleSpeed,roadMu,P)

    % Pacejka-style pure-longitudinal tyre shape
    limitedSlip = min(max(kappa,-20),20);

    magicArgument = ...
        P.magicB*limitedSlip - ...
        P.magicE*(P.magicB*limitedSlip - ...
                  atan(P.magicB*limitedSlip));

    tyreShape = sin(P.magicC*atan(magicArgument));

    % q is the signed longitudinal force coefficient
    q = roadMu*tyreShape;

    % Rolling resistance allocated to one equivalent wheel channel
    rollingForcePerWheel = ...
        P.rollingResistance*P.vehicleMass*P.g/ ...
        P.drivenWheelCount * ...
        tanh(vehicleSpeed/P.rollSpeedSmoothing);

    % Quasi-static longitudinal load-transfer solution:
    %
    % Fz,rear = m*g*rearFraction + m*ax*h/L
    % Fx      = q*Fz
    %
    % The expression below solves the coupled force/load-transfer equations.
    denominator = 1-q*P.cgHeight/P.wheelbase;
    denominator = max(denominator,0.25);

    ax = (q*P.g*P.rearWeightFraction - ...
          P.rollingResistance*P.g* ...
          tanh(vehicleSpeed/P.rollSpeedSmoothing))/ ...
          denominator;

    Fz = (P.vehicleMass*P.g*P.rearWeightFraction + ...
          P.vehicleMass*ax*P.cgHeight/P.wheelbase)/ ...
          P.drivenWheelCount;

    Fz = max(Fz,0);
    Fx = q*Fz;

    % Numerical consistency check for the equivalent single-wheel channel
    axCheck = (Fx-rollingForcePerWheel)/P.equivalentMass;

    if isfinite(axCheck)
        ax = axCheck;
    end
end

function output = applyRateLimit( ...
    desired,previous,dt,cutRate,returnRate)

    torqueChange = desired-previous;

    if torqueChange < 0
        torqueChange = max(torqueChange,-cutRate*dt);
    else
        torqueChange = min(torqueChange,returnRate*dt);
    end

    output = max(previous+torqueChange,0);
end
