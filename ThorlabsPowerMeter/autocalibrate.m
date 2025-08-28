hLaser = MouseBehaviorInterface('COM5'); % This is the laser control arduino
hMotor = MouseBehaviorInterface('COM7'); % This is the stepping motor arduino (for moving mirror)
hLaser.Arduino.DebugMode = false;
hMotor.Arduino.DebugMode = false;

%%
try
    meter.disconnect;                                      % Disconnect and release
end
hLaser.Arduino.SendMessage(sprintf('A %i %i', 1, 0));
hLaser.Arduino.SendMessage(sprintf('A %i %i', 2, 0));

clear p result
p.mirrorPositions = [-300, 0];
p.targetPowers = [0.5, 2, 4, 10] .* 1e-3;
p.wavelengths = [473, 593];
p.stepDelays = [0.5, 8];
p.aoutMin = 500;
p.aoutMax = 4095;
p.tolerancePercentage = 2.5e-2;
p.maxIters = 100;
p.maxStationaryIters = 5;
% p.stepSizeMultiplier = 50000; % Step size = -multiplier * (read wattage - target wattage)
p.stepSizeMultiplier = 50000; % Step size = -multiplier * (read wattage - target wattage)
p.stepSizeMultiplierHeur = 0.25; % 
p.readThreshold = 60e-6; % Watts, values below this are considered zero.
% p.subthresholdStepSize = 500; % 
p.subthresholdStepSize = 250; % 

result.aoutValues = zeros(length(p.targetPowers), length(p.mirrorPositions), 2);
result.targetReached = false(length(p.targetPowers), length(p.mirrorPositions), 2);
result.powers = zeros(length(p.targetPowers), length(p.mirrorPositions), 2);

meterList=ThorlabsPowerMeter;                              % Initiate the meter_list
DeviceDescription=meterList.listdevices;               	% List available device(s)
meter=meterList.connect(DeviceDescription);           % Connect single/the first devices

meter.setDispBrightness(0.3);                          % Set display brightness
meter.setAttenuation(0);                               % Set Attenuation
meter.setAverageTime(0.01);                            % Set average time for the measurement
meter.setTimeout(1000);                                % Set timeout value 

for iMirrorPos = 1:length(p.mirrorPositions)
    hMotor.Arduino.SetParam(find(strcmpi(hMotor.Arduino.ParamNames, 'TARGET_1')), p.mirrorPositions(iMirrorPos));
    while ~strcmpi('AT_TARGET', hMotor.Arduino.StateNames{hMotor.Arduino.State})
        pause(0.5);
    end
    for iLaser = 1:2
        meter.setWaveLength(p.wavelengths(iLaser));            % Set sensor wavelength
        meter.sensorInfo;                                      % Retrive the sensor info
        meter.setPowerAutoRange(1);                            % Set Autorange
        pause(5)                                                    % Pause the program a bit to allow the power meter to autoadjust
            
        for iPower = 1:length(p.targetPowers)
            hLaser.Arduino.SendMessage(sprintf('A %i %i', 1, 0));
            hLaser.Arduino.SendMessage(sprintf('A %i %i', 2, 0));
            
            aoutValue = 0;
            aoutValueOld = 0;
            pwr = NaN;
            pwrOld = NaN;
            targetPwr = p.targetPowers(iPower);
            i = 0;
            nStationaryIters = 0;
            
            while true
                if i > p.maxIters || nStationaryIters > p.maxStationaryIters
                    warning('Failed to reach target after %i iterations. Target = %g, Value = %g', i, targetPwr, targetPwr + dist)
                    break
                end

                pause(p.stepDelays(iLaser));
                meter.updateReading(0);                          % Update the power reading(with interal period of 0.5s)
                assert(meter.meterPowerUnit == 'W', 'Power meter unit is "%s" instead of "W"', meter.meterPowerUnit);

                if meter.meterPowerReading < p.readThreshold
                    pwrOld = NaN;
                    pwr = NaN;
                else
                    pwrOld = pwr;
                    pwr = meter.meterPowerReading;
                end

                i = i + 1;
                dist = meter.meterPowerReading - targetPwr;
                aoutValueOld = aoutValue;
                dPwr = pwr - pwrOld;
                if meter.meterPowerReading < p.readThreshold
                    thisStepSize = p.subthresholdStepSize;
                    stepType = 'bigStep';
                elseif isnan(dPwr)
                    thisStepSize = -sign(dist)*max(1, ceil(p.stepSizeMultiplier * abs(dist)));
                    stepType = 'distStep';
                else
                    thisStepSize = -sign(dist)*max(1, ceil(abs(p.stepSizeMultiplierHeur*dist./(dPwr./dAout))));
                    stepType = 'gradStep';
                end
                if abs(dist) > p.tolerancePercentage * targetPwr
                    aoutValue = aoutValue + thisStepSize;
                    aoutValue = min(aoutValue, p.aoutMax);
                    aoutValue = max(aoutValue, p.aoutMin);
                    dAout = aoutValue - aoutValueOld;
                    if dAout == 0
                        nStationaryIters = nStationaryIters + 1;
                    else
                        nStationaryIters = 0;
                    end
                else
                    fprintf('Target reached in %i iterations.\n', i)
                    result.targetReached(iPower, iMirrorPos, iLaser) = true;
                    break
                end
                hLaser.Arduino.SendMessage(sprintf('A %i %i', iLaser, aoutValue));
                fprintf('%gnm, i=%i, tgt=%.1fmW, df=%.3fmW, %s=%i, AOUT(%i)=%i\n', p.wavelengths(iLaser), i, targetPwr*1e3, dist*1e3, stepType, thisStepSize, iLaser, aoutValue)
            
        %         fprintf('%.10f%c\r',test_meter.meterPowerReading,test_meter.meterPowerUnit);
            end
        
            pause(p.stepDelays(iLaser));
            meter.updateReading(0);
            result.aoutValues(iPower, iMirrorPos, iLaser) = aoutValue;
            result.powers(iPower, iMirrorPos, iLaser) = meter.meterPowerReading;
        end
    end
end

% Turn lasers off
hLaser.Arduino.SendMessage(sprintf('A %i %i', 1, 0));
hLaser.Arduino.SendMessage(sprintf('A %i %i', 2, 0));

meter.disconnect;                                      % Disconnect and release

clearvars -except hLaser hMotor p result

% Validate

try
    meter.disconnect;                                      % Disconnect and release
end

meterList=ThorlabsPowerMeter;                              % Initiate the meter_list
DeviceDescription=meterList.listdevices;               	% List available device(s)
meter=meterList.connect(DeviceDescription);           % Connect single/the first devices

meter.setDispBrightness(0.3);                          % Set display brightness
meter.setAttenuation(0);                               % Set Attenuation
meter.setAverageTime(0.01);                            % Set average time for the measurement
meter.setTimeout(1000);                                % Set timeout value 

p.validationDelay = [4, 15];
result.powersValidation = zeros(size(result.powers));

for iMirrorPos = 1:length(p.mirrorPositions)
    hMotor.Arduino.SetParam(find(strcmpi(hMotor.Arduino.ParamNames, 'TARGET_1')), p.mirrorPositions(iMirrorPos));
    while ~strcmpi('AT_TARGET', hMotor.Arduino.StateNames{hMotor.Arduino.State})
        pause(0.5);
    end
    for iLaser = 1:2
        hLaser.Arduino.SendMessage(sprintf('A %i %i', 1, 0));
        hLaser.Arduino.SendMessage(sprintf('A %i %i', 2, 0));

        meter.setWaveLength(p.wavelengths(iLaser));            % Set sensor wavelength
        meter.sensorInfo;                                      % Retrive the sensor info
        meter.setPowerAutoRange(1);                            % Set Autorange
        pause(5);

        for iPower = 1:length(p.targetPowers)
            % Playback
            hLaser.Arduino.SendMessage(sprintf('A %i %i', iLaser, result.aoutValues(iPower, iMirrorPos, iLaser)));
            pause(p.validationDelay(iLaser))                               % Pause the program a bit to allow the power meter to autoadjust

            meter.updateReading(0);
            result.powersValidation(iPower, iMirrorPos, iLaser) = meter.meterPowerReading;

            fprintf('%inm, mirror=%i, Tgt=%.1fmW, Cal=%.3fmW, Val=%.3fmW\n', p.wavelengths(iLaser), p.mirrorPositions(iMirrorPos), p.targetPowers(iPower)*1e3, result.powers(iPower, iMirrorPos, iLaser)*1e3, meter.meterPowerReading*1e3);
        end
    end
end

meter.disconnect;
hLaser.Arduino.SendMessage(sprintf('A %i %i', 1, 0));
hLaser.Arduino.SendMessage(sprintf('A %i %i', 2, 0));
clearvars -except hLaser hMotor p result
