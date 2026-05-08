%%THIS IS THE SCRIPT TO CALIBRATE AND RUN TAGGING OPTOGENETICS

%% SECTION 1: Calibrate Green Power Levels
% Start Two Color Experiment for green calibration
exp = TwoColorExperimentSvedberg('laserCOM', 'COM5', 'noGUI', false);

% run calibration of green power levels
% note 08/21/25 100ma from lime driver yields 71.2uW 
exp.calibrateChrimson( ...
    targetPowers=[1, 3, 5, 7, 9, 10].*1e-4, ...     % we're using these power levels because 2 definitely evoked spikes but the next previous value was 0.1 so 1 seems reasonable to detect low activation and 5 should be saturation ask lingfeng tho
    wavelengths=[600, 0], ... %555 for lime light 600 for orange
    stepDelays=[0.5, 0.5], ...
    maxIters=64, ...
    maxStationaryIters=4, ...
    powerMeterThreshold=10e-6, ...
    aoutMin=50, ...
    stepSizeConst=500);

% Validate resutls
exp.validateChrimson('validationDelay', 1);  % Wait 1 seconds before reading power

%Save Green Calibration
exp.saveCalibration(['OrangeCalibration_' datestr(now, 'yyyymmdd')]);

%% SECTION 2: Calibrate Blue Power Levels
% before running this section, close and restart Two Color Experiment for
% blue calibration
exp = TwoColorExperimentSvedberg('laserCOM', 'COM5', 'noGUI', false);

% run calibration of Blue Power Levels
% note 08/21/25 100ma from blue driver yields 0.0.661mW
exp.calibrateChR2( ...
    targetPowers=[1, 3, 5, 7, 9, 10].*1e-4, ...
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


%% SECTION 3: Merge the Calibrations
blueCalStr = ['blueCalibration_' datestr(now, 'yyyymmdd') '.mat'];
greenCalStr = ['OrangeCalibration_' datestr(now, 'yyyymmdd') '.mat'];
exp.mergeCalibration(greenCalStr, blueCalStr);

% Save the merged Calibration
filename = ['mergedCalibration_' datestr(now, 'yyyymmdd')];
exp.saveCalibration(filename);

%% SECTION 4: Open New Two Color Experiment (behavior + excitaibility), load merged calibrations, and run the experiment
exp = TwoColorExperimentSvedberg('laserCOM', 'COM5', 'noGUI', false);
% save this file for the experiment when prompted by the dialog box

% Load Merged cal
[file, location] = uigetfile('calibration_files\mergedcalibration_*.mat', 'Select a calibration file');
exp.loadCalibration(file)
% console should report that various parameters (12-28) are changed and
% updated. You can now run the experiment using the open window
% use the "dan tag" button to start the dan tag protocol

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
