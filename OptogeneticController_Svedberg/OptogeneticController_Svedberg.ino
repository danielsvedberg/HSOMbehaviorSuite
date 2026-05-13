/*********************************************************************
  Optogenetics Controller — Svedberg Lab
  Adapted from ahamilos / Nigel Hunter / Ofer Mazor

  Last Modified: May 2025   Version: "STIM_ON_CMD_MAY_2025"

  Serial protocol (host → arduino):
    G#            Start experiment (generates schedule, enters WAIT_REQUEST)
    Q#            Quit → IDLE
    R#            Reset (restarts mySetup loop)
    C#            Manual Chrimson pulse train (from IDLE or WAIT_REQUEST)
    D#            Manual ChR2 pulse train
    M#            DanTag
    P id val#     Update parameter id to val
    O#            Parameter update complete
    A ch val#     Direct analog output (ch 1=chrimson, 2=chr2)

  Serial protocol (arduino → host):
    ~             Online
    @ n name can  State definition
    + n name      Event marker definition
    # n name val  Parameter definition
    * n name      Result code definition
    $ n           State entry
    & n t         Event marker with timestamp
    ` n           Result code
    R             DanTag/stim complete receipt
*********************************************************************/

// ===================================================================
// PIN DEFINITIONS
// ===================================================================
#define PIN_STIM_REQ          25   // Digital IN:  stim request from VLD arduino
#define PIN_TRIAL_INIT         5   // Digital IN:  trial start alignment
#define PIN_CUE_ON            26   // Digital IN:  cue alignment
#define PIN_CANCEL_STIM       20   // Digital IN:  abort ongoing stim when HIGH

#define PIN_STIM_RECEIPT       7   // Digital OUT: receipt back to VLD
#define PIN_CHRIMSON_STIM     23   // Digital OUT: NIDAQ record of chrimson AOUT
#define PIN_CHR2_STIM         22   // Digital OUT: NIDAQ record of chr2 AOUT

#define PIN_CHRIMSON_PWR     A21   // Analog OUT:  chrimson LED driver power
#define PIN_CHR2_PWR         A22   // Analog OUT:  chr2 LED driver power

#define ANALOG_WRITE_RESOLUTION 12 // 12-bit DAC: 0–4095

// ===================================================================
// VERSION
// ===================================================================
static String versionCode = "STIM_ON_CMD_MAY_2025";

// ===================================================================
// STATE MACHINE
// ===================================================================
enum State {
  _INIT,
  IDLE_STATE,
  INIT_EXP,
  WAIT_REQUEST_STATE,
  CHRIMSON_STIM_STATE,
  CHR2_STIM_STATE,
  DAN_TAG_STATE,
  UPDATE_PARAMS_STATE,
  _NUM_STATES
};

static const char *_stateNames[] = {
  "_INIT", "IDLE", "INIT_EXP",
  "WAIT_REQUEST", "CHRIMSON_STIM", "CHR2_STIM",
  "DAN_TAG_STIM", "UPDATE_PARAMS"
};

static const int _stateCanUpdateParams[] = {1,1,1,1,1,1,1,1};

// ===================================================================
// EVENT MARKERS
// ===================================================================
enum EventMarkers {
  EVENT_EXP_INIT,
  EVENT_TRIAL_INIT,
  EVENT_CUE_ON,
  BOSS_STIM_REQUEST,
  BOSS_STIM_BEGIN,
  BOSS_STIM_END,
  EVENT_UI_CHRIMSON_REQUEST,
  EVENT_UI_CHRIMSON_STIM_BEGIN,
  EVENT_UI_CHRIMSON_STIM_END,
  EVENT_CHRIMSON_ERROR,
  EVENT_UI_CHR2_REQUEST,
  EVENT_UI_CHR2_STIM_BEGIN,
  EVENT_UI_CHR2_STIM_END,
  EVENT_CHR2_ERROR,
  _NUM_OF_EVENT_MARKERS
};

static const char *_eventMarkerNames[] = {
  "EXP_INIT", "TRIAL_INIT", "CUE_ON",
  "BOSS_STIM_REQUEST", "BOSS_STIM_BEGIN", "BOSS_STIM_END",
  "UI_CHRIMSON_REQUEST", "UI_CHRIMSON_STIM_BEGIN", "UI_CHRIMSON_STIM_END", "CHRIMSON_ERROR",
  "UI_CHR2_REQUEST", "UI_CHR2_STIM_BEGIN", "UI_CHR2_STIM_END", "CHR2_ERROR"
};

// ===================================================================
// RESULT CODES
// ===================================================================
enum ResultCode {
  CODE_CHRIMSON,
  CODE_CHR2,
  CODE_CHRIMSON_CANCEL,
  CODE_CHR2_CANCEL,
  CODE_UI_CHRIMSON,
  CODE_UI_CHR2,
  CODE_DANTAG,
  _NUM_RESULT_CODES
};

static const char *_resultCodeNames[] = {
  "CHRIMSON", "CHR2",
  "CHRIMSON_CANCELLED", "CHR2_CANCELLED",
  "UI_CHRIMSON", "UI_CHR2",
  "DANTAG"
};

// ===================================================================
// PARAMETERS
// ===================================================================
enum ParamID {
  _DEBUG,
  OPTO_ENABLED,
  CANCEL_ENABLED,

  CHRIMSON_NUM_PULSES,
  CHRIMSON_PULSE_WIDTH,
  CHRIMSON_IPI,

  CHR2_NUM_PULSES,
  CHR2_PULSE_WIDTH,
  CHR2_IPI,

  CHRIMSON_CHR2_NUM_PULSES,
  CHRIMSON_CHR2_PULSE_WIDTH,
  CHRIMSON_CHR2_IPI,

  CHRIMSON_AOUT_1_VALUE,
  CHRIMSON_AOUT_2_VALUE,
  CHRIMSON_AOUT_3_VALUE,
  CHRIMSON_AOUT_4_VALUE,
  CHRIMSON_AOUT_5_VALUE,
  CHRIMSON_AOUT_6_VALUE,

  CHR2_AOUT_1_VALUE,
  CHR2_AOUT_2_VALUE,
  CHR2_AOUT_3_VALUE,
  CHR2_AOUT_4_VALUE,
  CHR2_AOUT_5_VALUE,
  CHR2_AOUT_6_VALUE,

  N_TRIALS_PER_BLOCK,   // total trials per block
  N_STIM_PER_BLOCK,     // stimulated trials per block (rest are no-stim)
  N_BLOCKS,             // total number of blocks

  _NUM_PARAMS
};

static const char *_paramNames[] = {
  "_DEBUG", "OPTO_ENABLED", "CANCEL_ENABLED",
  "CHRIMSON_NUM_PULSES", "CHRIMSON_PULSE_WIDTH", "CHRIMSON_IPI",
  "CHR2_NUM_PULSES", "CHR2_PULSE_WIDTH", "CHR2_IPI",
  "CHRIMSON_CHR2_NUM_PULSES", "CHRIMSON_CHR2_PULSE_WIDTH", "CHRIMSON_CHR2_IPI",
  "CHRIMSON_AOUT_1_VALUE", "CHRIMSON_AOUT_2_VALUE", "CHRIMSON_AOUT_3_VALUE",
  "CHRIMSON_AOUT_4_VALUE", "CHRIMSON_AOUT_5_VALUE", "CHRIMSON_AOUT_6_VALUE",
  "CHR2_AOUT_1_VALUE", "CHR2_AOUT_2_VALUE", "CHR2_AOUT_3_VALUE",
  "CHR2_AOUT_4_VALUE", "CHR2_AOUT_5_VALUE", "CHR2_AOUT_6_VALUE",
  "N_TRIALS_PER_BLOCK", "N_STIM_PER_BLOCK", "N_BLOCKS"
};

float _params[_NUM_PARAMS] = {
  1,     // _DEBUG
  1,     // OPTO_ENABLED
  1,     // CANCEL_ENABLED

  5,     // CHRIMSON_NUM_PULSES
  10,    // CHRIMSON_PULSE_WIDTH (ms)
  100,   // CHRIMSON_IPI (ms)

  5,     // CHR2_NUM_PULSES
  5,     // CHR2_PULSE_WIDTH (ms)
  35,    // CHR2_IPI (ms)

  200,   // CHRIMSON_CHR2_NUM_PULSES
  100,   // CHRIMSON_CHR2_PULSE_WIDTH (ms)
  2900,  // CHRIMSON_CHR2_IPI (ms)

  100,   // CHRIMSON_AOUT_1_VALUE
  200,   // CHRIMSON_AOUT_2_VALUE
  400,   // CHRIMSON_AOUT_3_VALUE
  800,   // CHRIMSON_AOUT_4_VALUE
  1600,  // CHRIMSON_AOUT_5_VALUE
  2000,  // CHRIMSON_AOUT_6_VALUE

  100,   // CHR2_AOUT_1_VALUE
  200,   // CHR2_AOUT_2_VALUE
  400,   // CHR2_AOUT_3_VALUE
  800,   // CHR2_AOUT_4_VALUE
  1600,  // CHR2_AOUT_5_VALUE
  2000,  // CHR2_AOUT_6_VALUE

  6,     // N_TRIALS_PER_BLOCK
  2,     // N_STIM_PER_BLOCK (1 chrimson + 1 chr2)
  30     // N_BLOCKS
};

// ===================================================================
// GLOBAL VARIABLES
// ===================================================================

// --- State machine ---
static State _state     = _INIT;
static State _prevState = _INIT;
static char  _command   = ' ';
static int   _arguments[2] = {0};
static int   _argcopy[2]   = {0};
static int   _resultCode   = -1;

// --- Timing ---
static signed long _exp_timer = 0;

// --- Stim source tracking (for result code disambiguation) ---
static bool _UI_CHRIMSON_stim_in_progress = false;
static bool _UI_CHR2_stim_in_progress     = false;
static bool _BOSS_CHRIMSON_stim_in_progress = false;
static bool _BOSS_CHR2_stim_in_progress     = false;

// --- UI pulse train state ---
static bool singleTrainMode = false;
unsigned long pulseTimer   = 0;
bool  inPulse              = false;
int   pulseIndex           = 0;
int   currentTrain         = 0;
int   currentAout          = 0;
unsigned long ipiTimer     = 0;
unsigned long chR2ipiTimer = 0;

// --- DanTag state ---
int   chrimsonidx = 0;
int   chr2idx     = 0;
bool  dingdong    = false;
float chrimsonLadder[6];
float chr2Ladder[6];

// --- Trial schedule ---
struct TrialScheduleEntry {
  int channel;  // 0=no stim, 1=chrimson, 2=chr2
  int aout;     // DAC value 0–4095
};

#define MAX_SCHEDULE_TRIALS 600
#define MAX_TRIALS_PER_BLOCK 30

static TrialScheduleEntry _trialSchedule[MAX_SCHEDULE_TRIALS];
static int  _scheduleLength = 0;
static int  _scheduleIndex  = 0;
static bool _stimActive     = false;
static bool _prevStimReq    = false;

// --- Trial alignment ---
static bool _ready_for_next_trial = true;
static bool _ready_for_cue        = true;

// --- Param update handshake ---
static bool _hold                  = false;
static bool _isParamsUpdateStarted = false;
static bool _isParamsUpdateDone    = true;

// ===================================================================
// FORWARD DECLARATIONS
// ===================================================================
void sendMessage(String message);
long signedMillis();
bool checkQuit();
bool checkParamUpdate();
bool checkUpdateComplete();
void checkTrialStart();
void checkReferenceEvent();
void setChrimsonPower(int value);
void setChR2Power(int value);
void setStimReceipt(bool turnOn);
void initChrimsonPulses();
void initChR2Pulses();
void runChrimsonTrain();
void runChR2Train();
void runChR2ChrimsonPulses();
void checkStimChrimson();
void checkStimChR2();
void generateSchedule();
void transmitSchedule();
void hostInit();

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  pinMode(PIN_CHRIMSON_STIM, OUTPUT);
  pinMode(PIN_STIM_RECEIPT,  OUTPUT);
  pinMode(PIN_CHR2_STIM,     OUTPUT);
  pinMode(PIN_STIM_REQ,      INPUT);
  pinMode(PIN_TRIAL_INIT,    INPUT);
  pinMode(PIN_CUE_ON,        INPUT);
  pinMode(PIN_CANCEL_STIM,   INPUT);
  analogWriteResolution(ANALOG_WRITE_RESOLUTION);
  pinMode(PIN_CHRIMSON_PWR,  OUTPUT);
  pinMode(PIN_CHR2_PWR,      OUTPUT);
  Serial.begin(115200);
}

void mySetup() {
  setChrimsonPower(0);
  setChR2Power(0);
  setStimReceipt(false);

  _state        = _INIT;
  _prevState    = _INIT;
  _command      = ' ';
  _arguments[0] = 0;
  _arguments[1] = 0;
  _resultCode   = -1;
  _exp_timer    = 0;

  _UI_CHRIMSON_stim_in_progress   = false;
  _UI_CHR2_stim_in_progress       = false;
  _BOSS_CHRIMSON_stim_in_progress = false;
  _BOSS_CHR2_stim_in_progress     = false;
  singleTrainMode = false;

  _scheduleLength = 0;
  _scheduleIndex  = 0;
  _stimActive     = false;
  _prevStimReq    = false;

  _ready_for_next_trial = true;
  _ready_for_cue        = true;
  _hold                  = false;
  _isParamsUpdateStarted = false;
  _isParamsUpdateDone    = true;

  hostInit();
}

// ===================================================================
// MAIN LOOP
// ===================================================================
void loop() {
  mySetup();

  while (true) {
    // Read one character from host per iteration
    static String usbMessage = "";
    _command      = ' ';
    _arguments[0] = 0;
    _arguments[1] = 0;

    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == '#') {
        _command = getCommand(usbMessage);
        getArguments(usbMessage, _arguments);
        usbMessage = "";
        if (_command == 'R') break;  // reset
      } else {
        usbMessage += inByte;
      }
    }

    handleAnalogOutput();  // direct DAC write, runs every loop

    switch (_state) {
      case _INIT:               idle_state();           break;
      case IDLE_STATE:          idle_state();           break;
      case INIT_EXP:            init_exp();             break;
      case WAIT_REQUEST_STATE:  wait_request_state();  break;
      case CHRIMSON_STIM_STATE: Chrimson_stim_state(); break;
      case CHR2_STIM_STATE:     Chr2_stim_state();     break;
      case DAN_TAG_STATE:       dan_tag_state();        break;
      case UPDATE_PARAMS_STATE: update_params_state(); break;
      default: break;
    }
  }
}

// ===================================================================
// STATES
// ===================================================================

void idle_state() {
  if (_state != _prevState) {
    sendMessage("-");
    sendMessage("Optogenetics_Controller");
    sendMessage("Version: " + versionCode);
    _prevState = _state;
    sendMessage("$" + String(_state));
    setChrimsonPower(0);
    setChR2Power(0);
    setStimReceipt(false);
    _resultCode = -1;
    if (_params[_DEBUG]) sendMessage("Idle.");
  }

  if (_command == 'G') { _state = INIT_EXP;            return; }
  if (_command == 'C') { _state = CHRIMSON_STIM_STATE; return; }
  if (_command == 'D') { _state = CHR2_STIM_STATE;     return; }
  if (_command == 'M') { _state = DAN_TAG_STATE;        return; }
  if (checkParamUpdate()) return;
  _state = IDLE_STATE;
}

void init_exp() {
  if (_state != _prevState) {
    _exp_timer = signedMillis();
    sendMessage("&" + String(EVENT_EXP_INIT) + " 0");
    _prevState = _state;
    sendMessage("$" + String(_state));
    if (_params[_DEBUG]) sendMessage("Experiment started. Generating schedule...");
    generateSchedule();
    transmitSchedule();
    _state = WAIT_REQUEST_STATE;
  }
}

void wait_request_state() {
  if (_state != _prevState) {
    _prevState   = _state;
    sendMessage("$" + String(_state));
    _stimActive  = false;
    _prevStimReq = false;
    if (_scheduleLength == 0)
      sendMessage("WARNING: Empty trial schedule. Check AOUT params.");
    else if (_params[_DEBUG])
      sendMessage("Awaiting stim request. Schedule: " + String(_scheduleLength) + " trials.");
  }

  if (checkQuit())        return;
  if (checkParamUpdate()) return;
  checkTrialStart();
  checkReferenceEvent();

  // UI manual stim — only accepted between trials
  if (!_stimActive) {
    if (_command == 'C') {
      sendMessage("&" + String(EVENT_UI_CHRIMSON_REQUEST) + " " + String(signedMillis() - _exp_timer));
      _UI_CHRIMSON_stim_in_progress = true;
      initChrimsonPulses();
      _command = ' ';
      setStimReceipt(true);
      _state = CHRIMSON_STIM_STATE;
      return;
    }
    if (_command == 'D') {
      sendMessage("&" + String(EVENT_UI_CHR2_REQUEST) + " " + String(signedMillis() - _exp_timer));
      _UI_CHR2_stim_in_progress = true;
      initChR2Pulses();
      _command = ' ';
      setStimReceipt(true);
      _state = CHR2_STIM_STATE;
      return;
    }
  }

  if (_params[OPTO_ENABLED] != 1 || _scheduleLength == 0) {
    _state = WAIT_REQUEST_STATE;
    return;
  }

  bool stimReqNow = digitalRead(PIN_STIM_REQ);

  // Rising edge → execute current schedule entry
  if (!_stimActive && stimReqNow && !_prevStimReq) {
    sendMessage("&" + String(BOSS_STIM_REQUEST) + " " + String(signedMillis() - _exp_timer));
    TrialScheduleEntry entry = _trialSchedule[_scheduleIndex];

    if (entry.channel == 1 && entry.aout > 0) {
      setChrimsonPower(entry.aout);
      setStimReceipt(true);
      sendMessage("&" + String(BOSS_STIM_BEGIN) + " " + String(signedMillis() - _exp_timer));
      if (_params[_DEBUG])
        sendMessage("Trial " + String(_scheduleIndex) + ": Chrimson AOUT=" + String(entry.aout));
    } else if (entry.channel == 2 && entry.aout > 0) {
      setChR2Power(entry.aout);
      setStimReceipt(true);
      sendMessage("&" + String(BOSS_STIM_BEGIN) + " " + String(signedMillis() - _exp_timer));
      if (_params[_DEBUG])
        sendMessage("Trial " + String(_scheduleIndex) + ": ChR2 AOUT=" + String(entry.aout));
    } else {
      if (_params[_DEBUG])
        sendMessage("Trial " + String(_scheduleIndex) + ": No stim.");
    }
    _stimActive = true;
  }

  // Active trial — poll for end condition
  if (_stimActive) {
    bool cancelFired  = (_params[CANCEL_ENABLED] == 1 && digitalRead(PIN_CANCEL_STIM) == HIGH);
    bool requestEnded = !stimReqNow;

    if (cancelFired || requestEnded) {
      setChrimsonPower(0);
      setChR2Power(0);
      setStimReceipt(false);
      sendMessage("&" + String(BOSS_STIM_END) + " " + String(signedMillis() - _exp_timer));

      TrialScheduleEntry entry = _trialSchedule[_scheduleIndex];
      if      (entry.channel == 1) _resultCode = cancelFired ? CODE_CHRIMSON_CANCEL : CODE_CHRIMSON;
      else if (entry.channel == 2) _resultCode = cancelFired ? CODE_CHR2_CANCEL     : CODE_CHR2;

      if (_resultCode >= 0) {
        sendMessage("`" + String(_resultCode));
        _resultCode = -1;
      }

      _scheduleIndex = (_scheduleIndex + 1) % _scheduleLength;  // wrap
      _stimActive    = false;
      if (_params[_DEBUG])
        sendMessage("Trial done. Next index: " + String(_scheduleIndex));
    }
  }

  _prevStimReq = stimReqNow;
  _state = WAIT_REQUEST_STATE;
}

// UI manual stim — runs a pulse train at CHRIMSON_AOUT_1_VALUE
void Chrimson_stim_state() {
  if (_state != _prevState) {
    _prevState = _state;
    sendMessage("$" + String(_state));
    if (_params[_DEBUG]) sendMessage("Stimulating Chrimson.");
    initChrimsonPulses();
  }
  if (checkQuit()) return;
  checkStimChrimson();
}

// UI manual stim — runs a pulse train at CHR2_AOUT_1_VALUE
void Chr2_stim_state() {
  if (_state != _prevState) {
    _prevState = _state;
    sendMessage("$" + String(_state));
    if (_params[_DEBUG]) sendMessage("Stimulating ChR2.");
    initChR2Pulses();
  }
  if (checkQuit()) return;
  checkStimChR2();
}

void update_params_state() {
  if (!_hold) {
    sendMessage("$" + String(_state));
    _hold = true;
    _params[_argcopy[0]] = _argcopy[1];
    if (_params[_DEBUG])
      sendMessage("Param " + String(_argcopy[0]) + " = " + String(_argcopy[1]));
  }
  if (checkQuit())          return;
  if (checkUpdateComplete()) return;
  _state = UPDATE_PARAMS_STATE;
}

void dan_tag_state() {
  if (_state != _prevState) {
    _prevState  = _state;
    sendMessage("$" + String(_state));
    pulseIndex  = 0;
    chrimsonidx = 0;
    chr2idx     = 0;
    currentAout = 0;
    inPulse     = false;
    dingdong    = false;

    for (int i = 0; i < 6; i++) chrimsonLadder[i] = _params[CHRIMSON_AOUT_1_VALUE + i];
    for (int i = 0; i < 6; i++) chr2Ladder[i]     = _params[CHR2_AOUT_1_VALUE + i];

    if (_params[_DEBUG]) sendMessage("DanTag started.");
  }

  // Done when chr2 phase has exhausted all 6 AOUT levels
  bool danTagDone = (dingdong && chr2idx >= 6);
  if (!danTagDone) {
    runChR2ChrimsonPulses();
  } else {
    sendMessage("R");
    _resultCode = CODE_DANTAG;
    sendMessage("`" + String(_resultCode));
    if (_params[_DEBUG]) sendMessage("DanTag complete.");
    _state = IDLE_STATE;
  }
}

// ===================================================================
// SCHEDULE GENERATION
// ===================================================================
void generateSchedule() {
  _scheduleLength = 0;
  _scheduleIndex  = 0;

  // Build per-channel AOUT pools (non-zero values only).
  // Each channel is fully independent — AOUTs are never mixed across channels.
  int chrimsonPool[6], nChr = 0;
  for (int i = 0; i < 6; i++) {
    int v = (int)_params[CHRIMSON_AOUT_1_VALUE + i];
    if (v > 0) chrimsonPool[nChr++] = v;
  }
  int chr2Pool[6], nCh2 = 0;
  for (int i = 0; i < 6; i++) {
    int v = (int)_params[CHR2_AOUT_1_VALUE + i];
    if (v > 0) chr2Pool[nCh2++] = v;
  }

  if (nChr == 0 && nCh2 == 0) {
    sendMessage("WARNING: All AOUT values are 0. No schedule generated.");
    return;
  }

  int nBlocks         = max((int)_params[N_BLOCKS], 1);
  int nTrialsPerBlock = constrain((int)_params[N_TRIALS_PER_BLOCK], 1, MAX_TRIALS_PER_BLOCK);
  int nStimPerBlock   = constrain((int)_params[N_STIM_PER_BLOCK],  0, nTrialsPerBlock);

  // Shuffle an int array in place (Fisher-Yates)
  auto shuffle = [](int* a, int n) {
    for (int i = n - 1; i > 0; i--) {
      int j = random(0, i + 1);
      int t = a[i]; a[i] = a[j]; a[j] = t;
    }
  };

  shuffle(chrimsonPool, nChr);
  shuffle(chr2Pool, nCh2);

  int chrIdx = 0, ch2Idx = 0;
  TrialScheduleEntry blockBuf[MAX_TRIALS_PER_BLOCK];

  for (int block = 0; block < nBlocks; block++) {
    if (_scheduleLength + nTrialsPerBlock > MAX_SCHEDULE_TRIALS) {
      sendMessage("WARNING: Schedule full at " + String(_scheduleLength) + " trials.");
      break;
    }

    // Per-block channel split, alternating which gets the extra on odd N_STIM_PER_BLOCK
    int half = nStimPerBlock / 2;
    int nChrThis = half, nCh2This = half;
    if (nStimPerBlock % 2 != 0) {
      if (block % 2 == 0) nChrThis++; else nCh2This++;
    }
    // Fold empty-channel allocation into the other channel
    if (nChr == 0) { nCh2This += nChrThis; nChrThis = 0; }
    if (nCh2 == 0) { nChrThis += nCh2This; nCh2This = 0; }

    int len = 0;

    // ---- Chrimson draws ----
    // Rule: never use an AOUT again until all others have been scheduled.
    // At pool wrap, items already drawn this block go to the BACK of the next cycle
    // so they can't repeat within the block.
    {
      int drawn[6], nDrawn = 0;
      for (int i = 0; i < nChrThis; i++) {
        if (chrIdx >= nChr) {
          // Partition pool: not-drawn-this-block → front (fresh), drawn → back (deferred)
          int fresh[6], nF = 0, deferred[6], nD = 0;
          for (int k = 0; k < nChr; k++) {
            bool used = false;
            for (int m = 0; m < nDrawn; m++) if (chrimsonPool[k] == drawn[m]) { used = true; break; }
            if (used) deferred[nD++] = chrimsonPool[k];
            else      fresh[nF++]    = chrimsonPool[k];
          }
          shuffle(fresh, nF);
          shuffle(deferred, nD);
          for (int k = 0; k < nF; k++) chrimsonPool[k]      = fresh[k];
          for (int k = 0; k < nD; k++) chrimsonPool[nF + k] = deferred[k];
          chrIdx = 0;
        }
        int aout = chrimsonPool[chrIdx++];
        drawn[nDrawn++] = aout;
        blockBuf[len++] = {1, aout};
      }
    }

    // ---- ChR2 draws (same logic, independent pool) ----
    {
      int drawn[6], nDrawn = 0;
      for (int i = 0; i < nCh2This; i++) {
        if (ch2Idx >= nCh2) {
          int fresh[6], nF = 0, deferred[6], nD = 0;
          for (int k = 0; k < nCh2; k++) {
            bool used = false;
            for (int m = 0; m < nDrawn; m++) if (chr2Pool[k] == drawn[m]) { used = true; break; }
            if (used) deferred[nD++] = chr2Pool[k];
            else      fresh[nF++]    = chr2Pool[k];
          }
          shuffle(fresh, nF);
          shuffle(deferred, nD);
          for (int k = 0; k < nF; k++) chr2Pool[k]      = fresh[k];
          for (int k = 0; k < nD; k++) chr2Pool[nF + k] = deferred[k];
          ch2Idx = 0;
        }
        int aout = chr2Pool[ch2Idx++];
        drawn[nDrawn++] = aout;
        blockBuf[len++] = {2, aout};
      }
    }

    // Fill remaining slots with no-stim
    for (int i = nStimPerBlock; i < nTrialsPerBlock; i++) blockBuf[len++] = {0, 0};

    // Shuffle the completed block
    for (int i = len - 1; i > 0; i--) {
      int j = random(0, i + 1);
      TrialScheduleEntry t = blockBuf[i]; blockBuf[i] = blockBuf[j]; blockBuf[j] = t;
    }

    for (int i = 0; i < len; i++) _trialSchedule[_scheduleLength++] = blockBuf[i];
  }

  if (_params[_DEBUG]) {
    sendMessage("Schedule: " + String(_scheduleLength) + " trials | " +
                String(nBlocks) + " blocks x " + String(nTrialsPerBlock) +
                " trials/block (" + String(nStimPerBlock) + " stim)");
  }
}

void transmitSchedule() {
  sendMessage("! BEGIN " + String(_scheduleLength));
  for (int i = 0; i < _scheduleLength; i++) {
    sendMessage("! " + String(i) + " " + String(_trialSchedule[i].channel) + " " + String(_trialSchedule[i].aout));
  }
  sendMessage("! END");
}

// ===================================================================
// STIMULATION HELPERS
// ===================================================================

void checkStimChrimson() {
  if (_params[CANCEL_ENABLED] == 1 && digitalRead(PIN_CANCEL_STIM) == HIGH) {
    setChrimsonPower(0);
    inPulse = false; pulseIndex = 0; currentTrain = 0;
    setStimReceipt(false);
    _resultCode = CODE_CHRIMSON_CANCEL;
    sendMessage("`" + String(_resultCode));
    if (_params[_DEBUG]) sendMessage("Chrimson stim cancelled.");
    _state = WAIT_REQUEST_STATE;
    return;
  }
  runChrimsonTrain();
  if (currentTrain >= 1) {
    currentTrain = 0; pulseIndex = 0;
    setStimReceipt(false);
    _state = WAIT_REQUEST_STATE;
  }
}

void checkStimChR2() {
  if (_params[CANCEL_ENABLED] == 1 && digitalRead(PIN_CANCEL_STIM) == HIGH) {
    setChR2Power(0);
    inPulse = false; pulseIndex = 0; currentTrain = 0;
    setStimReceipt(false);
    _resultCode = CODE_CHR2_CANCEL;
    sendMessage("`" + String(_resultCode));
    if (_params[_DEBUG]) sendMessage("ChR2 stim cancelled.");
    _state = WAIT_REQUEST_STATE;
    return;
  }
  runChR2Train();
  if (currentTrain >= 1) {
    currentTrain = 0; pulseIndex = 0;
    setStimReceipt(false);
    _state = WAIT_REQUEST_STATE;
  }
}

// ===================================================================
// PULSE TRAIN FUNCTIONS
// ===================================================================

// Single-train pulse at CHRIMSON_AOUT_1_VALUE — used by UI manual stim
void runChrimsonTrain() {
  if (!inPulse && pulseIndex < _params[CHRIMSON_NUM_PULSES] &&
      (millis() - ipiTimer >= (unsigned long)_params[CHRIMSON_IPI])) {
    setChrimsonPower((int)_params[CHRIMSON_AOUT_1_VALUE]);
    pulseTimer = millis();
    inPulse    = true;
    if (_params[_DEBUG]) sendMessage("Chrimson pulse " + String(pulseIndex + 1) + " ON");
  }
  if (inPulse && (millis() - pulseTimer >= (unsigned long)_params[CHRIMSON_PULSE_WIDTH])) {
    setChrimsonPower(0);
    inPulse  = false;
    ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("Chrimson pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;
  }
  if (pulseIndex >= _params[CHRIMSON_NUM_PULSES]) {
    pulseIndex = 0;
    currentTrain++;
    if (_params[_DEBUG]) sendMessage("Chrimson train " + String(currentTrain) + " done.");
    if (singleTrainMode) {
      _resultCode = _UI_CHRIMSON_stim_in_progress ? CODE_UI_CHRIMSON : CODE_CHRIMSON;
      sendMessage("`" + String(_resultCode));
      _UI_CHRIMSON_stim_in_progress   = false;
      _BOSS_CHRIMSON_stim_in_progress = false;
      _resultCode     = -1;
      singleTrainMode = false;
    }
  }
}

// Single-train pulse at CHR2_AOUT_1_VALUE — used by UI manual stim
void runChR2Train() {
  if (!inPulse && pulseIndex < _params[CHR2_NUM_PULSES] &&
      (millis() - chR2ipiTimer >= (unsigned long)_params[CHR2_IPI])) {
    setChR2Power((int)_params[CHR2_AOUT_1_VALUE]);
    pulseTimer   = millis();
    inPulse      = true;
    if (_params[_DEBUG]) sendMessage("ChR2 pulse " + String(pulseIndex + 1) + " ON");
  }
  if (inPulse && (millis() - pulseTimer >= (unsigned long)_params[CHR2_PULSE_WIDTH])) {
    setChR2Power(0);
    inPulse      = false;
    chR2ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("ChR2 pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;
  }
  if (pulseIndex >= _params[CHR2_NUM_PULSES]) {
    pulseIndex = 0;
    currentTrain++;
    if (_params[_DEBUG]) sendMessage("ChR2 train " + String(currentTrain) + " done.");
    if (singleTrainMode) {
      _resultCode = _UI_CHR2_stim_in_progress ? CODE_UI_CHR2 : CODE_CHR2;
      sendMessage("`" + String(_resultCode));
      _UI_CHR2_stim_in_progress   = false;
      _BOSS_CHR2_stim_in_progress = false;
      _resultCode     = -1;
      singleTrainMode = false;
    }
  }
}

// DanTag: all CHRIMSON_CHR2_NUM_PULSES pulses at each Chrimson AOUT level (escalating),
// then all pulses at each ChR2 AOUT level. dingdong=false → chrimson phase, true → chr2 phase.
void runChR2ChrimsonPulses() {
  if (checkQuit()) { setChrimsonPower(0); setChR2Power(0); return; }

  // Switch to chr2 phase once all 6 chrimson AOUT levels are done
  if (!dingdong && chrimsonidx >= 6) {
    dingdong   = true;
    pulseIndex = 0;
  }

  if (!inPulse && pulseIndex < (int)_params[CHRIMSON_CHR2_NUM_PULSES] &&
      (millis() - ipiTimer >= (unsigned long)_params[CHRIMSON_CHR2_IPI])) {
    if (!dingdong) setChrimsonPower(chrimsonLadder[chrimsonidx]);
    else           setChR2Power(chr2Ladder[chr2idx]);
    pulseTimer = millis();
    inPulse    = true;
    if (_params[_DEBUG]) sendMessage("DanTag pulse " + String(pulseIndex + 1) + " ON");
  }

  if (inPulse && (millis() - pulseTimer >= (unsigned long)_params[CHRIMSON_CHR2_PULSE_WIDTH])) {
    setChrimsonPower(0);
    setChR2Power(0);
    inPulse  = false;
    ipiTimer = millis();
    pulseIndex++;
    if (_params[_DEBUG]) sendMessage("DanTag pulse " + String(pulseIndex) + " OFF");

    // After completing all pulses at this AOUT level, advance to the next
    if (pulseIndex >= (int)_params[CHRIMSON_CHR2_NUM_PULSES]) {
      pulseIndex = 0;
      if (!dingdong) {
        if (_params[_DEBUG])
          sendMessage("DanTag: Chrimson AOUT " + String(chrimsonidx + 1) + " complete.");
        chrimsonidx++;
      } else {
        if (_params[_DEBUG])
          sendMessage("DanTag: ChR2 AOUT " + String(chr2idx + 1) + " complete.");
        chr2idx++;
      }
    }
  }
}

void initChrimsonPulses() {
  currentTrain    = 0; pulseIndex = 0; inPulse = false; currentAout = 0;
  singleTrainMode = true;
  if (_params[_DEBUG]) sendMessage("Init Chrimson pulse train.");
}

void initChR2Pulses() {
  currentTrain    = 0; pulseIndex = 0; inPulse = false;
  singleTrainMode = true;
  if (_params[_DEBUG]) sendMessage("Init ChR2 pulse train.");
}

// ===================================================================
// HARDWARE CONTROL
// ===================================================================

void handleAnalogOutput() {
  if (_command == 'A') {
    if      (_arguments[0] == 1) setChrimsonPower(_arguments[1]);
    else if (_arguments[0] == 2) setChR2Power(_arguments[1]);
  }
}

void setChrimsonPower(int value) {
  analogWrite(PIN_CHRIMSON_PWR, value);
  digitalWrite(PIN_CHRIMSON_STIM, value > 0 ? HIGH : LOW);
}

void setChR2Power(int value) {
  analogWrite(PIN_CHR2_PWR, value);
  digitalWrite(PIN_CHR2_STIM, value > 0 ? HIGH : LOW);
}

void setStimReceipt(bool turnOn) {
  digitalWrite(PIN_STIM_RECEIPT, turnOn ? HIGH : LOW);
}

// ===================================================================
// SERIAL COMMUNICATION
// ===================================================================

void sendMessage(String message) { Serial.println(message); }

char getCommand(String message) { message.trim(); return message[0]; }

void getArguments(String message, int *args) {
  args[0] = 0; args[1] = 0;
  message.trim();
  String p = message;
  p.remove(0, 1);
  p.trim();

  String s = "";
  while (p.length() > 0 && isDigit(p[0])) { s += p[0]; p.remove(0, 1); }
  args[0] = s.toInt();

  p.trim(); s = "";
  while (p.length() > 0 && isDigit(p[0])) { s += p[0]; p.remove(0, 1); }
  args[1] = s.toInt();
}

void hostInit() {
  for (int i = 0; i < _NUM_STATES; i++)
    sendMessage("@ " + String(i) + " " + _stateNames[i] + " " + String(_stateCanUpdateParams[i]));
  for (int i = 0; i < _NUM_OF_EVENT_MARKERS; i++)
    sendMessage("+ " + String(i) + " " + _eventMarkerNames[i]);
  for (int i = 0; i < _NUM_PARAMS; i++)
    sendMessage("# " + String(i) + " " + _paramNames[i] + " " + String(_params[i]));
  for (int i = 0; i < _NUM_RESULT_CODES; i++)
    sendMessage("* " + String(i) + " " + _resultCodeNames[i]);
  sendMessage("~");
}

long signedMillis() { return (long)millis(); }

// ===================================================================
// TRANSITION CHECKS
// ===================================================================

bool checkQuit() {
  if (_command == 'Q') { _state = IDLE_STATE; return true; }
  return false;
}

void checkTrialStart() {
  if (_ready_for_next_trial && !digitalRead(PIN_TRIAL_INIT)) {
    _ready_for_next_trial = false;
    _ready_for_cue        = true;
    sendMessage("&" + String(EVENT_TRIAL_INIT) + " " + String(signedMillis() - _exp_timer));
  } else if (!_ready_for_next_trial && digitalRead(PIN_TRIAL_INIT)) {
    _ready_for_next_trial = true;
  }
}

void checkReferenceEvent() {
  if (_ready_for_cue && digitalRead(PIN_CUE_ON)) {
    sendMessage("&" + String(EVENT_CUE_ON) + " " + String(signedMillis() - _exp_timer));
    _ready_for_cue = false;
  }
}

bool checkParamUpdate() {
  if (_command == 'P') {
    _argcopy[0] = _arguments[0];
    _argcopy[1] = _arguments[1];
    _isParamsUpdateStarted = true;
    _isParamsUpdateDone    = false;
    _params[_arguments[0]] = _arguments[1];
    _state = UPDATE_PARAMS_STATE;
    if (_params[_DEBUG])
      sendMessage("Param " + String(_arguments[0]) + " → " + String(_arguments[1]));
    return true;
  }
  return false;
}

bool checkUpdateComplete() {
  if (_command == 'O') {
    _isParamsUpdateStarted = false;
    _isParamsUpdateDone    = true;
    _state = _prevState;
    _hold  = false;
    if (_params[_DEBUG]) sendMessage("Param update complete.");
    return true;
  }
  return false;
}
