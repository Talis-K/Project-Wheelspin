function P = FSEAVehicle()

P.Vehicle.Mass = 310;       % kg, vehicle + 68 kg driver
P.Vehicle.Wheelbase = 1.550; % m
P.Vehicle.FrontTrack = 1.200; % m
P.Vehicle.RearTrack = 1.150;  % m
P.Vehicle.CGHeight = 0.289;   % m

% Static axle masses from spec, without driver
P.Vehicle.FrontMassNoDriver = 96.8;  % kg
P.Vehicle.RearMassNoDriver = 145.2;  % kg

% Powertrain
P.Motor.PeakTorque = 220;       % Nm
P.Motor.ContinuousTorque = 112; % Nm
P.Drivetrain.FinalDriveRatio = 6;

% Suspension wheel rates
P.Suspension.FrontWheelRate = 12.93e3; % N/m
P.Suspension.RearWheelRate  = 17.04e3; % N/m


P.Tire.RadiusInitial = 0.259; %  approximation
P.Tire.MuInitial = 1.5;       %  placeholder, NOT measured

end