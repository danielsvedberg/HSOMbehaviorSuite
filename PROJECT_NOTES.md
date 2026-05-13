# HSOMbehaviorSuite — Project Notes

## System Overview

HSOMbehaviorSuite is a system for controlling a mouse behavior task and optogenetic LED drivers.

## Components

### MATLAB Scripts (User Interface)

| Script | Alias | Role |
|---|---|---|
| `MouseBehaviorInterface.m` | MBI | Operates the behavior task; interfaces with the behavior arduino over serial port COM3 |
| `TwoColorExperimentSvedberg.m` | TCE | Interfaces with the optogenetics arduino over serial port COM5 |

### Arduino Firmware

| File | Alias | Port | Controller |
|---|---|---|---|
| `Video_Lick_Detection_svedberg_final/Video_Lick_Detection_svedberg_final.ino` | VLD | COM3 | Behavior task — driven by MBI |
| `OptogeneticController_Svedberg/OptogeneticController_Svedberg.ino` | OCI | COM5 | Optogenetic LED driver — driven by TCE |

## Serial Port Map

| Port | Arduino Firmware | MATLAB Interface |
|---|---|---|
| COM3 | VLD | MBI |
| COM5 | OCI | TCE |

## OCI Features

### 1. Manual UI Stim (working)
Commands `'C'` (Chrimson) and `'D'` (ChR2) from TCE trigger a single pulse train on the respective LED driver. Works from IDLE_STATE and WAIT_REQUEST_STATE. Uses `CHRIMSON_NUM_PULSES` / `CHRIMSON_PULSE_WIDTH` / `CHRIMSON_IPI` (and CHR2 equivalents).

### 2. DanTag (working)
Command `'M'`. Runs autonomously: alternating Chrimson and ChR2 pulses at increasing AOUT levels across `CHRIMSON_CHR2_NUM_PULSES` pulses, stepping through 6-level ladders (`CHRIMSON_AOUT_1_VALUE` … `CHRIMSON_AOUT_6_VALUE` and CHR2 equivalents). Reports progress to MATLAB, self-terminates, returns to WAIT_REQUEST_STATE.

### 3. Stim-on-Command / WAIT_REQUEST_STATE (nonfunctional — needs full rework)
Command `'G'` starts the experiment and enters WAIT_REQUEST_STATE. OCI should await a `stim_request` pin signal from COM3/VLD, look up the next entry in a pre-generated trial schedule (stimulated? which channel? what AOUT?), execute or skip, and end on `stim_cancel` or when the pin drops. Currently broken throughout.

### 4. OptoLadder (command `'L'`, status unclear)
Structured sequence: N Chrimson trains → inter-train delay → N ChR2 trains → delay → N more Chrimson trains. Parameters: `CHRIMSON_TRAINS_BEFORE_CHR2`, `CHR2_NUM_TRAINS`, `CHRIMSON_TRAINS_AFTER_CHR2`, `STIM_TRAIN_DELAY`. Uses only the first 5 AOUT ladder values (6th defined but not used). Distinct from DanTag.

### 5. Direct Analog Output Command (`'A'`)
`handleAnalogOutput()` runs every loop. Allows TCE to set a specific DAC level on either channel directly (`A channel value`), bypassing pulse logic entirely.

### 6. PIN Alignment Signals (passive)
`checkTrialStart()` monitors `PIN_TRIAL_INIT` and `checkReferenceEvent()` monitors `PIN_CUE_ON`. These send timestamped event markers to MATLAB for trial/cue alignment only — no stim control.

### 7. Hardware Cancel Pin
`PIN_CANCEL_STIM` — when pulled HIGH, aborts ongoing Chrimson or ChR2 stim mid-train. Gated by `CANCEL_ENABLED` param.

### 8. Stim Receipt Pin
`PIN_STIM_RECEIPT` — digital output back to COM3/VLD confirming stim was received and initiated.

---

## OCI Known Bugs (pre-overhaul)

- `Chrimson_stim_state()` has a syntax error (missing parentheses on `if`) and an incomplete second `if` — does not compile
- `boss_stim_in_progress` referenced in `checkStimRequest()` (line 931) but declared as `_BOSS_stim_in_progress`
- `_CHRIMSON_trigger_on`, `_CHR2_trigger_on`, `_chrimson_stimulation_requested`, `_chR2_stimulation_requested` assigned in `mySetup()` but never declared globally — compile error
- `runChrimsonPulse()` and `runChR2Pulse()` defined but never called
- BOSS-triggered stim path in `checkStimRequest()` does not call `initChrimsonPulses()`/`initChR2Pulses()`, so `singleTrainMode` is never set → BOSS-triggered trains never return to WAIT_REQUEST_STATE

---

*Notes added as information is provided.*
