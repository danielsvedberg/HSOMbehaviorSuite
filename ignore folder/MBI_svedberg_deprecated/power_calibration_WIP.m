%%  Start Two Color Experiment
exp = TwoColorExperimentSvedberg('laserCOM', 'COM5', 'noGUI', false);
%% Calibrate Green Power Levels
% note 08/21/25 100ma from lime driver yields 71.2uW 
exp.calibrateChrimson( ...
    targetPowers=[0.1, 0.2, 0.4, 1, 2, 4].*1e-4, ...     % we're using these power levels because 2 definitely evoked spikes but the next previous value was 0.1 so 1 seems reasonable to detect low activation and 5 should be saturation ask lingfeng tho
    wavelengths=[555,0], ... 
    stepDelays=[0.5, 0.5], ...
    maxIters=64, ...
    maxStationaryIters=4, ...
    powerMeterThreshold=10e-6, ...
    aoutMin=50, ...
    stepSizeConst=500);

% Validate resutls
exp.validateChrimson('validationDelay', 1);  % Wait 1 seconds before reading power

%Save Red Calibration
exp.saveCalibration(['GreenCalibration_' datestr(now, 'yyyymmdd')]);

%% Close and Restart Two Color Experiment 
exp = TwoColorExperimentSvedberg('laserCOM', 'COM5', 'noGUI', false);

%% Calibrate Blue Power Levels
% note 08/21/25 100ma from blue driver yields 0.0.661mW
exp.calibrateChR2( ...
    targetPowers= [0.11, 0.2, 0.4, 1, 2, 4].*1e-4, ...
    wavelengths=[0,465], ... 
    stepDelays=[0.5,0.5], ...
    maxIters=64, ...
    maxStationaryIters=4, ...
    powerMeterThreshold=10e-6, ...
    aoutMin=50, ...
    stepSizeConst=500);

% Validate resutls
exp.validateChR2('validationDelay', 1);  % Wait 1 seconds before reading power

%Save Calibration
exp.saveCalibration(['blueCalibration_' datestr(now, 'yyyymmdd')]);


%% Merge the Calibrations
%exp.mergeCalibration('redCalibration_20250602.mat', 'blueCalibration_05-18-2025.mat');
blueCalStr = ['blueCalibration_' datestr(now, 'yyyymmdd')];
greenCalStr = ['GreenCalibration_' datestr(now, 'yyyymmdd')];
exp.mergeCalibration(greenCalStr, blueCalStr);

% Save the margeed Calibration
filename = ['mergedCalibration_' datestr(now, 'yyyymmdd')];
exp.saveCalibration(filename);

%% Open New Two Color Experiment (behavior + excitaibility)
exp = TwoColorExperimentSvedberg('laserCOM', 'COM5', 'noGUI', false);

%% Load Merged cal
%filename = "C:\Users\assad\Documents\codevault\HSOMbehaviorSuite\mergedCalibration_20250824.mat";
exp.loadCalibration(filename)

%% IGNORE THESE FOR NOW
% %% run opto ladder Multipe
% 
% %runOptoLadderMultiple(exp, chr2Aout, numTrials, checkInterval, trialDelaySec, useCalibrated)
% % runOptoLadderMultiple(exp, 4100, 1, 0.1, 15);  % Runs 1 trials at
% % CHR2_AOUT = 4100, 15 seconds delay
% 
% runOptoLadderMultiple(exp, 4100, 20, 0.1, 10, false);
% 
% %%
% runOptoLadder(exp,'ChR2', 4100, false);
% 
% %% ------Extra functions ----%
% % Run ONE Red Stim Train
% exp.runStimTrain(1,1); %runStimTrain(targetPowers(1), channel(1 red 2 blue), varargin)
% 
% %% Run ONE Blue Stim Train 
% exp.runStimTrain(1,2);
% 
% %% Run Chr2StimSession -- one train at each light level
% exp.runCHR2CalibrationSession('repeats',20,'randomize',true,'iti',10);
% 
% %% Run ChrimsonStimSession -- one train at each light level
% exp.runChrimsonCalibrationSession('repeats',1,'randomize',true);
% 
% %% Function loadCalibration
% exp.loadCalibration('C:\Users\assad\Desktop\Neual_Analysis\AutoOpto MBI\calibrations\06052025\blueCalibration_20250605.mat')
% 
% %% printLastOptoLadderLog();
% exp.printLastOptoLadderLog();
