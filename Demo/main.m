
function runSimulation()
% Launch the E59 vehicle simulation interface.

modelName = 'FSEA_Vehicle';

% Load the Simulink model without starting it.
load_system(modelName);

% Open the App Designer interface.
app = Control;

end
