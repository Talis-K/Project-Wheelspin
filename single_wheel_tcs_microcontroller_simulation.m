

clear; clc; close all;

%% 1. USER-ADJUSTABLE DEMONSTRATION INPUT
% Change this value between 0 and 220 N.m, then rerun the script.
driverTorqueSetting_Nm = 220;

%% 2. VEHICLE-SPECIFICATION VALUES
% Values marked SPECIFICATION are taken from the supplied 2025 vehicle data.
% Values marked ASSUMPTION must be replaced when measured team data exist.
P.g = 9.81;
P.vehicleMass = 242 + 68;                % kg, SPECIFICATION: car + driver
P.channelMass = P.vehicleMass/2;          % kg, one rear drive channel
P.rearStaticFraction = 0.50;             % ASSUMPTION
P.wheelbase = 1.550;                      % m, SPECIFICATION
P.cgHeight = 0.289;                       % m, SPECIFICATION
P.tyreRadius = (20.4*0.0254)/2;           % m, SPECIFICATION: D2704 diameter
P.gearRatio = 6.0;                        % motor/wheel speed, SPECIFICATION
P.drivetrainEfficiency = 0.95;            % ASSUMPTION
P.differentialSplit = 0.50;               % one of two driven rear wheels
P.motorPeakTorque = 220;                  % N.m, SPECIFICATION
P.motorPeakPower = 80e3;                  % W, SPECIFICATION/accumulator limit
P.motorMaximumRPM = 5170;                 % rpm, SPECIFICATION
P.motorTimeConstant = 0.020;              % s, ASSUMPTION
P.wheelEquivalentInertia = 0.90;          % kg.m^2, ASSUMPTION
P.rollingResistance = 0.015;              % ASSUMPTION
P.airDensity = 1.225;                     % kg/m^3
P.dragArea = 0.80;                        % Cd*A (m^2), ASSUMPTION

%% 3. TYRE AND ROAD MODEL
% Pacejka-style curve is used because a simple force saturation cannot show
% the loss of usable force at excessive slip.
P.magicB = 10.0;
P.magicC = 1.90;
P.magicE = 0.97;
P.muDry = 1.20;
P.muLow = 0.48;
P.muRecovery = 0.90;
P.muEstimateDry = 1.05;                   % controller's conservative estimate
P.muEstimateLow = 0.58;                   % intentionally imperfect estimate
P.muEstimateRecovery = 0.80;
P.lowGripStart = 1.50;                    % s
P.lowGripEnd = 4.20;                      % s

%% 4. MICROCONTROLLER CALIBRATION
P.controllerSampleTime = 0.002;           % s, 500 Hz controller
P.plantStep = 0.0005;                     % s, numerical integration
P.stopTime = 6.0;                         % s
P.slipDenominatorGuard = 0.50;            % m/s
P.slipFilterTimeConstant = 0.015;         % s
P.slipTarget = 0.12;
P.slipActivation = 0.14;
P.slipRelease = 0.09;
P.activationPersistence = 0.020;          % s
P.releasePersistence = 0.080;             % s
P.recoveryTime = 0.120;                   % s
P.controlEnableSpeed = 1.50;              % m/s
P.launchSlipVelocity = 0.35;              % m/s, low-speed detection variable
P.launchVelocityGain = 80;                 % N.m per (m/s) excess launch slip
P.launchTorqueCap = 85;                   % N.m motor-side
P.admissibleTorqueSafetyFactor = 0.95;
P.Kp = 650;                               % N.m per unit slip error
P.Ki = 900;                               % N.m/(unit slip*s)
P.Kaw = 1/P.Kp;                           % back-calculation anti-windup
P.integratorMinimum = -0.35;
P.integratorMaximum = 0.10;
P.torqueCutRate = 9000;                   % N.m/s
P.torqueRestoreRate = 500;                % N.m/s
P.sensorResolutionRPM = 0.25;             % simulated speed quantisation
P.sensorDropoutEnabled = false;
P.sensorDropoutStart = 3.00;
P.sensorDropoutEnd = 3.10;
P.faultTorque = 0;

assert(driverTorqueSetting_Nm >= 0 && ...
    driverTorqueSetting_Nm <= P.motorPeakTorque, ...
    'driverTorqueSetting_Nm must be between 0 and %.0f N.m.',P.motorPeakTorque);
assert(abs(P.controllerSampleTime/P.plantStep - ...
    round(P.controllerSampleTime/P.plantStep)) < 1e-12, ...
    'Controller sample time must be an integer multiple of plant step.');

%% 5. RUN IDENTICAL BASELINE AND CONTROLLED CASES
off = runCase(P,driverTorqueSetting_Nm,false);
on = runCase(P,driverTorqueSetting_Nm,true);

metricsOff = calculateMetrics(off,P);
metricsOn = calculateMetrics(on,P);
checks = runAcceptanceChecks(off,on,metricsOff,metricsOn,P);

%% 6. COMMAND-WINDOW SUMMARY
fprintf('\n============================================================\n');
fprintf(' SINGLE-WHEEL MICROCONTROLLER TCS - VALIDATION SUMMARY\n');
fprintf('============================================================\n');
fprintf('Driver torque setting       : %.1f N.m\n',driverTorqueSetting_Nm);
fprintf('Controller sample time      : %.1f ms\n',1e3*P.controllerSampleTime);
fprintf('Peak slip, TCS OFF / ON     : %.3f / %.3f\n', ...
    metricsOff.peakSlip,metricsOn.peakSlip);
fprintf('RMS excess slip, OFF / ON   : %.3f / %.3f\n', ...
    metricsOff.rmsExcessSlip,metricsOn.rmsExcessSlip);
fprintf('Slip energy, OFF / ON       : %.1f / %.1f J\n', ...
    metricsOff.slipEnergy,metricsOn.slipEnergy);
fprintf('Final speed, OFF / ON       : %.2f / %.2f km/h\n', ...
    3.6*metricsOff.finalSpeed,3.6*metricsOn.finalSpeed);
fprintf('TCS ACTIVE duration         : %.3f s\n',metricsOn.activeTime);
fprintf('TCS interventions           : %d\n',metricsOn.interventions);
fprintf('------------------------------------------------------------\n');
for k = 1:numel(checks.name)
    if checks.pass(k), resultText = 'PASS'; else, resultText = 'FAIL'; end
    fprintf('%-46s %s\n',checks.name{k},resultText);
end
if all(checks.pass)
    fprintf('OVERALL RESULT: PASS\n');
else
    fprintf('OVERALL RESULT: REVIEW FAILED CHECKS\n');
end
fprintf('============================================================\n\n');

%% 7. PLOTS
fig = figure('Color','w','Name','Single-wheel microcontroller TCS', ...
    'Position',[50 40 1350 900]);

subplot(4,2,1);
plot(off.time,3.6*off.vehicleSpeed,'--','LineWidth',1.2); hold on;
plot(on.time,3.6*on.vehicleSpeed,'LineWidth',1.6);
plot(on.time,3.6*on.wheelTreadSpeed,':','LineWidth',1.2);
grid on; ylabel('Speed (km/h)'); title('Vehicle and driven-wheel speed');
legend('Vehicle: OFF','Vehicle: ON','Tread: ON','Location','best');

subplot(4,2,2);
plot(off.time,off.filteredSlip,'--','LineWidth',1.2); hold on;
plot(on.time,on.filteredSlip,'LineWidth',1.6);
plot(on.time,P.slipTarget*ones(size(on.time)),'k:','LineWidth',1.0);
plot(on.time,P.slipActivation*ones(size(on.time)),'r:','LineWidth',1.0);
grid on; ylabel('Slip ratio'); title('Measured longitudinal slip');
legend('TCS OFF','TCS ON','Target','Activation','Location','best');
ylim([-0.05 1.00]);

subplot(4,2,3);
plot(on.time,on.driverTorque,'k--','LineWidth',1.1); hold on;
plot(on.time,on.motorTorqueLimit,'Color',[0.45 0.45 0.45],'LineWidth',1.2);
plot(on.time,on.admissibleTorque,'Color',[0.10 0.55 0.20],'LineWidth',1.4);
plot(on.time,on.torqueCommand,'LineWidth',1.6);
grid on; ylabel('Motor torque (N.m)'); title('Microcontroller torque arbitration');
legend('Driver request','Motor envelope','Admissible torque','Final command', ...
    'Location','best');

subplot(4,2,4);
plot(on.time,on.motorRPM,'LineWidth',1.4); hold on;
plot(on.time,on.referenceWheelRPM,'LineWidth',1.4);
plot(on.time,P.motorMaximumRPM*ones(size(on.time)),'k:');
grid on; ylabel('Sensor speed (rpm)'); title('Resolver and reference-wheel sensors');
legend('Motor resolver','Reference wheel','Maximum motor rpm','Location','best');

subplot(4,2,5);
plot(off.time,off.tyreForce,'--','LineWidth',1.2); hold on;
plot(on.time,on.tyreForce,'LineWidth',1.5);
plot(on.time,on.frictionLimit,'r:','LineWidth',1.1);
grid on; ylabel('Force (N)'); title('Usable longitudinal tyre force');
legend('TCS OFF','TCS ON','\muF_z limit','Location','best');

subplot(4,2,6);
plot(on.time,on.trueRoadMu,'LineWidth',1.5); hold on;
plot(on.time,on.estimatedRoadMu,'--','LineWidth',1.3);
grid on; ylabel('Friction coefficient'); title('Actual and estimated grip');
legend('Actual road \mu','Controller estimate','Location','best');

subplot(4,2,7);
stairs(on.time,on.controllerState,'LineWidth',1.4); hold on;
stairs(on.time,double(on.controllerActive),'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('State'); title('Controller supervisor');
yticks(0:5); yticklabels({'OFF','LAUNCH','ARMED','ACTIVE','RECOVERY','FAULT'});
legend('State','ACTIVE flag','Location','best');

subplot(4,2,8);
plot(off.time,off.slipPower,'--','LineWidth',1.2); hold on;
plot(on.time,on.slipPower,'LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Slip power (W)'); title('Energy wasted in slip');
legend('TCS OFF','TCS ON','Location','best');

if exist('sgtitle','file') ~= 0
    sgtitle('TCS: Precautionary Torque Limit + Reactive Slip Control');
end
if exist('exportgraphics','file') ~= 0
    exportgraphics(fig,'tcs_microcontroller_validation.png','Resolution',200);
else
    print(fig,'tcs_microcontroller_validation.png','-dpng','-r200');
end

save('tcs_microcontroller_results.mat','P','driverTorqueSetting_Nm', ...
    'off','on','metricsOff','metricsOn','checks');

%% LOCAL FUNCTIONS
function R = runCase(P,torqueSetting,tcsEnabled)
%RUNCASE Execute one deterministic TCS OFF or TCS ON simulation.

N = round(P.stopTime/P.plantStep)+1;
R.time = (0:N-1)'*P.plantStep;
controllerRatio = round(P.controllerSampleTime/P.plantStep);

signalNames = {'vehicleSpeed','wheelAngularSpeed','wheelTreadSpeed', ...
    'motorRPM','referenceWheelRPM','trueSlip','measuredSlip','filteredSlip', ...
    'slipVelocity','driverTorque','motorTorqueLimit','admissibleTorque', ...
    'reactiveTorqueCap','torqueCommand','actualMotorTorque','wheelTorque', ...
    'tyreForce','normalLoad','frictionLimit','longitudinalAcceleration', ...
    'trueRoadMu','estimatedRoadMu','controllerState','controllerActive', ...
    'activationTimer','releaseTimer','slipPower','distance'};
for n = 1:numel(signalNames)
    R.(signalNames{n}) = zeros(N,1);
end

memory.filteredSlip = 0;
memory.integrator = 0;
memory.previousCommand = 0;
memory.state = 0;
memory.activationTimer = 0;
memory.releaseTimer = 0;
memory.recoveryTimer = 0;
memory.previousVehicleSpeed = 0;
memory.filteredAcceleration = 0;

held.command = 0;
held.filteredSlip = 0;
held.measuredSlip = 0;
held.slipVelocity = 0;
held.driverTorque = 0;
held.motorLimit = P.motorPeakTorque;
held.admissibleTorque = P.launchTorqueCap;
held.reactiveCap = P.launchTorqueCap;
held.state = 0;
held.active = false;
held.activationTimer = 0;
held.releaseTimer = 0;

for k = 2:N
    timeNow = R.time(k);
    vx = R.vehicleSpeed(k-1);
    omegaWheel = R.wheelAngularSpeed(k-1);

    motorRPMTrue = omegaWheel*P.gearRatio*60/(2*pi);
    referenceRPMTrue = (vx/P.tyreRadius)*60/(2*pi);

    if mod(k-2,controllerRatio) == 0
        driverRequest = driverProfile(timeNow,torqueSetting);
        [held,memory] = microcontrollerTick(P,timeNow,driverRequest, ...
            motorRPMTrue,referenceRPMTrue,tcsEnabled,memory);
    end

    actualMotorTorque = R.actualMotorTorque(k-1) + P.plantStep* ...
        (held.command-R.actualMotorTorque(k-1))/P.motorTimeConstant;
    wheelTorque = actualMotorTorque*P.gearRatio* ...
        P.drivetrainEfficiency*P.differentialSplit;

    treadSpeed = P.tyreRadius*omegaWheel;
    trueSlipVelocity = treadSpeed-vx;
    trueSlip = trueSlipVelocity/max(abs(vx),P.slipDenominatorGuard);
    muTrue = roadFriction(timeNow,P,false);
    [tyreForce,Fz,ax,frictionLimit] = tyreAndVehicle(P,trueSlip,vx,muTrue);

    wheelAcceleration = (wheelTorque-P.tyreRadius*tyreForce)/ ...
        P.wheelEquivalentInertia;
    nextOmegaWheel = max(0,omegaWheel+P.plantStep*wheelAcceleration);
    nextVehicleSpeed = max(0,vx+P.plantStep*ax);

    R.wheelAngularSpeed(k) = nextOmegaWheel;
    R.vehicleSpeed(k) = nextVehicleSpeed;
    R.wheelTreadSpeed(k) = P.tyreRadius*nextOmegaWheel;
    R.motorRPM(k) = nextOmegaWheel*P.gearRatio*60/(2*pi);
    R.referenceWheelRPM(k) = (nextVehicleSpeed/P.tyreRadius)*60/(2*pi);
    R.trueSlip(k) = trueSlip;
    R.measuredSlip(k) = held.measuredSlip;
    R.filteredSlip(k) = held.filteredSlip;
    R.slipVelocity(k) = held.slipVelocity;
    R.driverTorque(k) = held.driverTorque;
    R.motorTorqueLimit(k) = held.motorLimit;
    R.admissibleTorque(k) = held.admissibleTorque;
    R.reactiveTorqueCap(k) = held.reactiveCap;
    R.torqueCommand(k) = held.command;
    R.actualMotorTorque(k) = actualMotorTorque;
    R.wheelTorque(k) = wheelTorque;
    R.tyreForce(k) = tyreForce;
    R.normalLoad(k) = Fz;
    R.frictionLimit(k) = frictionLimit;
    R.longitudinalAcceleration(k) = ax;
    R.trueRoadMu(k) = muTrue;
    R.estimatedRoadMu(k) = roadFriction(timeNow,P,true);
    R.controllerState(k) = held.state;
    R.controllerActive(k) = held.active;
    R.activationTimer(k) = held.activationTimer;
    R.releaseTimer(k) = held.releaseTimer;
    R.slipPower(k) = abs(tyreForce*trueSlipVelocity);
    R.distance(k) = R.distance(k-1) + P.plantStep*0.5*(vx+nextVehicleSpeed);
end
end

function [out,memory] = microcontrollerTick(P,t,driverRequest, ...
    motorRPMTrue,referenceRPMTrue,tcsEnabled,memory)
%MICROCONTROLLERTICK Code structure suitable for later embedded conversion.

Ts = P.controllerSampleTime;
dropout = P.sensorDropoutEnabled && ...
    t >= P.sensorDropoutStart && t <= P.sensorDropoutEnd;

if dropout
    motorRPMMeasured = NaN;
    referenceRPMMeasured = NaN;
else
    q = P.sensorResolutionRPM;
    motorRPMMeasured = q*round(motorRPMTrue/q);
    referenceRPMMeasured = q*round(referenceRPMTrue/q);
end

sensorValid = isfinite(motorRPMMeasured) && ...
    isfinite(referenceRPMMeasured) && motorRPMMeasured >= 0 && ...
    referenceRPMMeasured >= 0;

if sensorValid
    omegaWheelMeasured = motorRPMMeasured*(2*pi/60)/P.gearRatio;
    treadSpeedMeasured = P.tyreRadius*omegaWheelMeasured;
    vehicleSpeedMeasured = referenceRPMMeasured*(2*pi/60)*P.tyreRadius;
    slipVelocity = treadSpeedMeasured-vehicleSpeedMeasured;
    rawSlip = slipVelocity/max(abs(vehicleSpeedMeasured), ...
        P.slipDenominatorGuard);
    rawSlip = min(max(rawSlip,-3),3);
else
    vehicleSpeedMeasured = memory.previousVehicleSpeed;
    slipVelocity = 0;
    rawSlip = 0;
end

alpha = Ts/(P.slipFilterTimeConstant+Ts);
memory.filteredSlip = memory.filteredSlip + ...
    alpha*(rawSlip-memory.filteredSlip);

motorOmega = max(abs(motorRPMMeasured)*2*pi/60,1);
if ~sensorValid || motorRPMMeasured >= P.motorMaximumRPM
    motorLimit = 0;
else
    motorLimit = min(P.motorPeakTorque,P.motorPeakPower/motorOmega);
end

rawAcceleration = (vehicleSpeedMeasured-memory.previousVehicleSpeed)/Ts;
rawAcceleration = min(max(rawAcceleration,-15),15);
memory.filteredAcceleration = memory.filteredAcceleration + ...
    0.15*(rawAcceleration-memory.filteredAcceleration);
memory.previousVehicleSpeed = vehicleSpeedMeasured;
rearLoad = P.vehicleMass*P.g*P.rearStaticFraction + ...
    P.vehicleMass*memory.filteredAcceleration*P.cgHeight/P.wheelbase;
normalLoadEstimate = max(rearLoad/2,0);
muEstimate = roadFriction(t,P,true);

% T_adm = SF*mu_est*Fz*r/(gear*efficiency*differentialSplit)
admissibleTorque = P.admissibleTorqueSafetyFactor*muEstimate* ...
    normalLoadEstimate*P.tyreRadius/(P.gearRatio* ...
    P.drivetrainEfficiency*P.differentialSplit);
admissibleTorque = min(max(admissibleTorque,0),motorLimit);

requested = min(max(driverRequest,0),motorLimit);
lowSpeed = vehicleSpeedMeasured < P.controlEnableSpeed;
if lowSpeed
    launchFeedbackCap = P.launchTorqueCap-P.launchVelocityGain* ...
        max(slipVelocity-P.launchSlipVelocity,0);
    proactiveCap = min([requested,admissibleTorque,max(launchFeedbackCap,0)]);
else
    proactiveCap = min(requested,admissibleTorque);
end

% Supervisor states: 0 OFF, 1 LAUNCH, 2 ARMED, 3 ACTIVE,
%                    4 RECOVERY, 5 FAULT.
if ~sensorValid
    memory.state = 5;
elseif ~tcsEnabled || requested < 0.01
    memory.state = 0;
    memory.activationTimer = 0;
    memory.releaseTimer = 0;
    memory.recoveryTimer = 0;
elseif lowSpeed
    memory.state = 1;
    memory.activationTimer = 0;
    memory.releaseTimer = 0;
    memory.recoveryTimer = 0;
else
    if memory.state == 0 || memory.state == 1 || memory.state == 5
        memory.state = 2;
    end

    if memory.filteredSlip > P.slipActivation
        memory.activationTimer = memory.activationTimer+Ts;
    else
        memory.activationTimer = 0;
    end
    if memory.filteredSlip < P.slipRelease
        memory.releaseTimer = memory.releaseTimer+Ts;
    else
        memory.releaseTimer = 0;
    end

    if memory.state == 2 && ...
            memory.activationTimer >= P.activationPersistence
        memory.state = 3;
        memory.releaseTimer = 0;
    elseif memory.state == 3 && ...
            memory.releaseTimer >= P.releasePersistence
        memory.state = 4;
        memory.recoveryTimer = 0;
    elseif memory.state == 4
        if memory.filteredSlip > P.slipActivation
            memory.state = 3;
            memory.releaseTimer = 0;
        else
            memory.recoveryTimer = memory.recoveryTimer+Ts;
            if memory.recoveryTimer >= P.recoveryTime
                memory.state = 2;
                memory.recoveryTimer = 0;
            end
        end
    end
end

active = memory.state == 3;
if active
    slipError = P.slipTarget-memory.filteredSlip;
    unsaturatedCap = proactiveCap + P.Kp*slipError + ...
        P.Ki*memory.integrator;
    reactiveCap = min(max(unsaturatedCap,0),proactiveCap);
    integratorDerivative = slipError + ...
        P.Kaw*(reactiveCap-unsaturatedCap);
    memory.integrator = min(max(memory.integrator + ...
        Ts*integratorDerivative,P.integratorMinimum),P.integratorMaximum);
else
    reactiveCap = proactiveCap;
    if memory.state ~= 4
        memory.integrator = 0;
    end
end

if memory.state == 5
    desiredTorque = P.faultTorque;
elseif tcsEnabled
    desiredTorque = reactiveCap;
else
    desiredTorque = requested;
end

if memory.state == 5
    command = P.faultTorque;              % immediate fail-safe intervention
else
    change = desiredTorque-memory.previousCommand;
    change = min(max(change,-P.torqueCutRate*Ts), ...
        P.torqueRestoreRate*Ts);
    command = min(max(memory.previousCommand+change,0),motorLimit);
end
memory.previousCommand = command;

out.command = command;
out.filteredSlip = memory.filteredSlip;
out.measuredSlip = rawSlip;
out.slipVelocity = slipVelocity;
out.driverTorque = driverRequest;
out.motorLimit = motorLimit;
out.admissibleTorque = admissibleTorque;
out.reactiveCap = reactiveCap;
out.state = memory.state;
out.active = active;
out.activationTimer = memory.activationTimer;
out.releaseTimer = memory.releaseTimer;
end

function torque = driverProfile(t,maximumTorque)
if t < 0.20
    torque = 0;
elseif t < 0.60
    torque = maximumTorque*(t-0.20)/0.40;
else
    torque = maximumTorque;
end
end

function mu = roadFriction(t,P,useEstimate)
if t < P.lowGripStart
    if useEstimate, mu = P.muEstimateDry; else, mu = P.muDry; end
elseif t < P.lowGripEnd
    if useEstimate, mu = P.muEstimateLow; else, mu = P.muLow; end
else
    if useEstimate, mu = P.muEstimateRecovery; else, mu = P.muRecovery; end
end
end

function [Fx,Fz,ax,frictionLimit] = tyreAndVehicle(P,kappa,vx,mu)
k = min(max(kappa,-5),5);
argument = P.magicB*k-P.magicE*(P.magicB*k-atan(P.magicB*k));
normalisedForce = sin(P.magicC*atan(argument));
q = mu*normalisedForce;

rollingForce = P.rollingResistance*P.vehicleMass*P.g*tanh(vx/0.20);
dragForce = 0.5*P.airDensity*P.dragArea*vx*abs(vx);
resistancePerMass = (rollingForce+dragForce)/P.vehicleMass;
denominator = max(1-q*P.cgHeight/P.wheelbase,0.25);
ax = (q*P.g*P.rearStaticFraction-resistancePerMass)/denominator;
Fz = max((P.vehicleMass*P.g*P.rearStaticFraction + ...
    P.vehicleMass*ax*P.cgHeight/P.wheelbase)/2,0);
frictionLimit = mu*Fz;
Fx = q*Fz;
end

function M = calculateMetrics(R,P)
mask = R.time >= 0.60 & R.vehicleSpeed >= P.controlEnableSpeed;
if ~any(mask), mask = R.time >= 0.60; end
excess = max(R.filteredSlip(mask)-P.slipTarget,0);
M.peakSlip = max(R.filteredSlip(mask));
M.rmsExcessSlip = sqrt(mean(excess.^2));
M.slipEnergy = trapz(R.time,R.slipPower);
M.finalSpeed = R.vehicleSpeed(end);
M.maximumSpeed = max(R.vehicleSpeed);
M.distance = R.distance(end);
M.activeTime = sum(R.controllerActive)*P.plantStep;
M.interventions = sum(diff([false; R.controllerActive]) > 0);
M.maximumCommand = max(R.torqueCommand);
M.maximumMotorRPM = max(R.motorRPM);
M.maximumTyreForce = max(R.tyreForce);
end

function checks = runAcceptanceChecks(~,on,mOff,mOn,P)
mainSignals = [on.vehicleSpeed; on.wheelAngularSpeed; on.filteredSlip; ...
    on.torqueCommand; on.actualMotorTorque; on.tyreForce];
checks.name = { ...
    'All principal signals remain finite'; ...
    'Command never becomes negative'; ...
    'Command respects 220 N.m motor peak'; ...
    'Command respects instantaneous motor envelope'; ...
    'TCS command does not exceed driver request'; ...
    'TCS controller reaches ACTIVE state'; ...
    'TCS ON reduces peak slip'; ...
    'TCS ON reduces RMS excess slip'; ...
    'TCS ON reduces total slip energy'; ...
    'Positive torque produces forward motion'};
checks.pass = [ ...
    all(isfinite(mainSignals)); ...
    all(on.torqueCommand >= -1e-9); ...
    all(on.torqueCommand <= P.motorPeakTorque+1e-6); ...
    all(on.torqueCommand <= on.motorTorqueLimit+1e-6); ...
    all(on.torqueCommand <= on.driverTorque+1e-6); ...
    mOn.activeTime > 0; ...
    mOn.peakSlip < mOff.peakSlip; ...
    mOn.rmsExcessSlip < mOff.rmsExcessSlip; ...
    mOn.slipEnergy < mOff.slipEnergy; ...
    on.vehicleSpeed(end) > 0.1];
end
