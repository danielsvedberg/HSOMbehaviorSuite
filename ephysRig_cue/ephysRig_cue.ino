/*********************************************************************
  Arduino state machine
  classical conditioning task - tone and licking
*********************************************************************/

// Digital OUT
#define PIN_REWARD      24
#define PIN_OPTO        5
#define PIN_CUE_MIR     7
#define PIN_REW_MIR     9
#define PIN_LICK_MIR    6
#define PIN_LICK_PWR    23
#define PIN_CAM        8
#define PIN_LAMP      11

// PWM OUT
#define PIN_SPEAKER     4

// Digital IN
#define PIN_LICK    26 //10 //26
#define PIN_LICK_PHOTO    20


/*****************************************************
  Enums - DEFINE States
*****************************************************/
// All the states
enum State
{
  _STATE_INIT,      // (Private) Initial state used on first loop.
  STATE_IDLE,       // Idle state. Wait for go signal from host.
  STATE_PRE_CUE,
  STATE_POST_CUE,
  _NUM_STATES       // (Private) Used to count number of states
};

// State names stored as strings, will be sent to host
// Names cannot contain spaces!!!
static const char *_stateNames[] =
{
  "_INIT",
  "IDLE",
  "PRE_CUE",
  "POST_CUE",
};

// Define which states accept parameter update from MATLAB
static const int _stateCanUpdateParams[] = {0, 1, 1, 0};

/*****************************************************
  Event Markers
*****************************************************/
enum EventMarker
{
  EVENT_PRE_CUE_ON,               // Begin pre cue period
  EVENT_POST_CUE_ON,
  EVENT_CUE_ON,         // Begin tone/LED cue presentation
  EVENT_CUE_OFF,          // End tone/LED cue presentation
  EVENT_REWARD_ON,        // Reward, juice valve on
  EVENT_REWARD_OFF,       // Reward, juice valve off
  EVENT_LICK,           // Lick onset
  EVENT_LICK_OFF,         // Lick offset
  EVENT_OPTO_ON,          // Begin optogenetic stim (single pulse start)
  EVENT_OPTO_OFF,         // End optogenetic stim (single pulse end)
  EVENT_LICK_PHOTO,           // Lick onset
  EVENT_LICK_OFF_PHOTO,         // Lick offset
  EVENT_CAM_TRIG_1_ON,
  EVENT_CAM_TRIG_2_ON,
  EVENT_CAM_TRIG_1_OFF,
  EVENT_CAM_TRIG_2_OFF,
  _NUM_OF_EVENT_MARKERS
};

static const char *_eventMarkerNames[] =
{
  "PRE_CUE_ON",                      // Begin tone cue presentation
  "POST_CUE_ON",
  "CUE_ON",           // Begin tone cue presentation
  "CUE_OFF",            // End tone cue presentation
  "REWARD_ON",          // Reward, juice valve on
  "REWARD_OFF",         // Reward, juice valve off
  "LICK",             // Lick onset
  "LICK_OFF",           // Lick offset
  "OPTO_ON",              // Begin optogenetic stim (single pulse start)
  "OPTO_OFF",             // End optogenetic stim (single pulse end)
  "LICK_PHOTO",           // Lick onset
  "LICK_OFF_PHOTO",
  "CAM_TRIG_1_ON",
  "CAM_TRIG_2_ON",
  "CAM_TRIG_1_OFF",
  "CAM_TRIG_2_OFF"
};

/*****************************************************
  Result codes
*****************************************************/
enum ResultCode
{
  CODE_END_TRIAL,     // End of trial. Need this because of LFH code, otherwise doesnt do much
  _NUM_RESULT_CODES   // (Private) Used to count how many codes there are.
};

// We'll send result code translations to MATLAB at startup
static const char *_resultCodeNames[] =
{
  "END_TRIAL"
};

/*****************************************************
  Parameters that can be updated by HOST
*****************************************************/
// Storing everything in array _params[]. Using enum ParamID as array indices so it's easier to add/remove parameters.
enum ParamID
{
  _DEBUG,             // (Private) 1 to enable debug mode. Default 0.
  POST_CUE_MIN,       // Pre cue duration, min (ms)
  POST_CUE_MAX,       // Pre cue duration, max (ms)
  PRE_CUE_DURATION,           // PRE cue preiod duration
  TONE_ENABLE,
  TONE_VARIABLE,        //if 1 then it is variable wether the tone will turn on (i.e. will encourage spontanous licks)
  TONE_DURATION,        // Duration of the cue tone and LED flash (ms)
  TONE_FREQ,
  REWARD_DURATION,      // Reward duration (ms)
  OPTO,           //whether to do optogenetic stimulation 1 yes, 0 no
  OPTO_TIME,
  OPTO_DURATION,
  OPTO_PERCENT,               // percent of trials on which there is an opto stimulation
  OPTO_DURATION_RAND,
  _NUM_PARAMS         // (Private) Used to count how many parameters there are so we can initialize the param array with the correct size. Insert additional parameters before this.
};

// Store parameter names as strings, will be sent to host
// Names cannot contain spaces!!!
static const char *_paramNames[] =
{
  "_DEBUG",
  "POST_CUE_MIN",
  "POST_CUE_MAX",
  "PRE_CUE_DURATION",
  "TONE_ENABLE",
  "TONE_VARIABLE",
  "TONE_DURATION",
  "TONE_FREQ",
  "REWARD_DURATION",
  "OPTO",
  "OPTO_TIME",
  "OPTO_DURATION",
  "OPTO_PERCENT",
  "OPTO_DURATION_RAND",
};

// Initialize parameters
long _params[_NUM_PARAMS] =
{
  1,    // _DEBUG
  7000,   // POST_CUE_MIN
  15000,  // POST_CUE_MAX
  3000, // PRE_CUE_DURATION
  1,    // TONE_ENABLE
  0,    //TONE_VARIABLE
  10, // TONE_DURATION
  6272,   // TONE_FREQ
  30,   // REWARD_DURATION
  0,    // OPTO
  3000,   // OPTO_TIME
  450,   // OPTO_DURATION
  30,     // OPTO_PERCENT
  1,     //OPTO_DURATION_RAND
};


/*****************************************************
  Other Global Variables
*****************************************************/
// Variables declared here can be carried to the next loop, AND read/written in function scope as well as main scope
// (previously defined):

static long _timeReset        = 0;      // Reset to signedMillis() at every soft reset
static long _timePreCue       = 0;      // determined at pre_cue onset
static long _timeCueOn              = 0;            // Reset to 0 at cue on
static long _timeOptoOn              = 0;            // time for opto on
static int _resultCode        = CODE_END_TRIAL;     // Result code.
static State _state         = _STATE_INIT;  // This variable (current _state) get passed into a _state function, which determines what the next _state should be, and updates it to the next _state.
static State _prevState       = _STATE_INIT;  // Remembers the previous _state from the last loop (actions should only be executed when you enter a _state for the first time, comparing currentState vs _prevState helps us keep track of that).
static char _command        = ' ';      // Command char received from host, resets on each loop
static int _arguments[2]      = {0};      // Two integers received from host , resets on each loop
static bool _isLicking        = false;    // True if the little dude is licking
static bool _isLickOnset      = false;    // True during lick onset
static bool _isLicking2         = false;    // True if the little dude is licking with PHOTO DETECTION
static bool _isLickOnset2      = false;    // True during lick onset with PHOTO DETECTION
static bool _optoStarted            = false;        // True when opto is started pn a trial. reset to false at the begining of a trial
static bool _optoComplete           = true;        // True when opto stim is complete for the trial. rest to false when opto started.
static bool _randOptoGate     = false;
static long _optoDuration     = 600;
static long _optoTime;
static long _optoTime1 = 3000;
static long _optoTime2 = 3000;
static bool _cam = false;
static long _camTime = 0;
/*****************************************************
  Setup
*****************************************************/
void setup()
{
  // Init pins
  pinMode(PIN_SPEAKER, OUTPUT);               // Tone for 'Go' cue
  pinMode(PIN_REWARD, OUTPUT);                // Reward, set to HIGH to open juice valve
  pinMode(PIN_OPTO, OUTPUT);
  pinMode(PIN_LICK, INPUT);                   // Lick detector
  pinMode(PIN_LICK_PHOTO, INPUT);
  pinMode(PIN_LICK_MIR, OUTPUT);
  pinMode(PIN_CUE_MIR, OUTPUT);
  pinMode(PIN_REW_MIR, OUTPUT);
  pinMode(PIN_LICK_PWR, OUTPUT);
  pinMode(PIN_LAMP, OUTPUT);
  pinMode(PIN_CAM, OUTPUT);
  // Serial comms
  Serial.begin(115200);                       // Set up USB communication at 115200 baud
}


void mySetup()
{
  // Reset output
  noTone(PIN_SPEAKER);                    // Cue TONE OFF
  digitalWrite(PIN_OPTO, LOW);
  digitalWrite(PIN_LICK_MIR, LOW);
  digitalWrite(PIN_CUE_MIR, LOW);
  digitalWrite(PIN_REW_MIR, LOW);
  digitalWrite(PIN_LICK_PWR, HIGH);
  digitalWrite(PIN_CAM, LOW);
  digitalWrite(PIN_LAMP, HIGH);

  // Reset variables
  _timeReset        = 0;      // Reset to signedMillis() at every soft reset
  _timeCueOn        = 0;      // Reset to 0 at cue on
  _resultCode         = CODE_END_TRIAL;     // Result code. -1 if there is no result.
  _state          = _STATE_INIT;  // This variable (current _state) get passed into a _state function, which determines what the next _state should be, and updates it to the next _state.
  _prevState        = _STATE_INIT;  // Remembers the previous _state from the last loop (actions should only be executed when you enter a _state for the first time, comparing currentState vs _prevState helps us keep track of that).
  _command        = ' ';      // Command char received from host, resets on each loop
  _arguments[2]     = {0};      // Two integers received from host , resets on each loop
  _isLicking        = false;    // True if the little dude is licking
  _isLickOnset      = false;    // True during lick onset
  _isLicking2        = false;    // True if the little dude is licking
  _isLickOnset2      = false;    // True during lick onset
  _randOptoGate     = false;
  _cam = false;
  // Sends all parameters, states and error codes to Matlab, then tell PC that we're running by sending '~' message:
  hostInit();
}


/*****************************************************
  MAIN LOOP
*****************************************************/
void loop()
{
  // Initialization
  mySetup();

  // Main loop (break out via "soft reset" command)
  while (true)
  {
    // 1) Check USB for MESSAGE from HOST, if available. String is read byte by byte.
    // Initialize usbMessage to empty string, only happens once on first loop (thanks to static!)
    static String usbMessage  = "";
    _command = ' ';
    _arguments[0] = 0;
    _arguments[1] = 0;

    if (Serial.available() > 0)
    {
      char inByte = Serial.read();
      // The pound sign ('#') indicates a complete message
      if (inByte == '#')
      {
        // Parse the string, and updates `_command`, and `_arguments`
        _command = getCommand(usbMessage);
        getArguments(usbMessage, _arguments);
        // Clear message buffer (resets to prepare for next message)
        usbMessage = "";
        // "R" triggers a soft reset
        if (_command == 'R')
        {
          break;
        }
      }
      else
      {
        // append character to message buffer
        usbMessage = usbMessage + inByte;
      }
    }

    // 2) Other onEachLoop routines
    handleLick();     // Check for licks on/offset
    handleLick2();      // Check for licks on/offset with PHOTO DETECTION
    if (getLickState())   // mirror lick on chan 6
    {
      digitalWrite(PIN_LICK_MIR, HIGH);
    }
    else
    {
      digitalWrite(PIN_LICK_MIR, LOW);
    }


    //trigger camera
    if (_cam && signedMillis() - _timePreCue >= _params[PRE_CUE_DURATION] - 1000 && signedMillis() - _timePreCue < _params[PRE_CUE_DURATION] - 1000 + 1)
    {
      digitalWrite(PIN_CAM, HIGH);
      _cam = false;
      _camTime = signedMillis();
      sendEventMarker(EVENT_CAM_TRIG_1_ON, -1);
    }
    if (!_cam && signedMillis() - _camTime >= 2 && signedMillis() - _camTime < 3 )
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_1_OFF, -1);
      _cam = true;
    }

    if (_cam && signedMillis() - _camTime >= 1002 && signedMillis() - _camTime < 1003)
    {
      digitalWrite(PIN_CAM, HIGH);
      sendEventMarker(EVENT_CAM_TRIG_2_ON, -1);
      _cam = false;
    }
    if (!_cam && signedMillis() - _camTime >= 1004 && signedMillis() - _camTime < 1005)
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_2_OFF, -1);
      _cam = true;
    }

    if (_cam && signedMillis() - _camTime >= 2004 && signedMillis() - _camTime < 2005)
    {
      digitalWrite(PIN_CAM, HIGH);
      sendEventMarker(EVENT_CAM_TRIG_2_ON, -1);
      _cam = false;
    }
    if (!_cam && signedMillis() - _camTime >= 2006 && signedMillis() - _camTime < 2007)
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_2_OFF, -1);
      _cam = true;
    }

    if (_cam && signedMillis() - _camTime >= 3006 && signedMillis() - _camTime < 3007)
    {
      digitalWrite(PIN_CAM, HIGH);
      sendEventMarker(EVENT_CAM_TRIG_2_ON, -1);
      _cam = false;
    }
    if (!_cam && signedMillis() - _camTime >= 3008 && signedMillis() - _camTime < 3009)
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_2_OFF, -1);
      _cam = true;
    }
    if (_cam && signedMillis() - _camTime >= 4010 && signedMillis() - _camTime < 4011)
    {
      digitalWrite(PIN_CAM, HIGH);
      sendEventMarker(EVENT_CAM_TRIG_2_ON, -1);
      _cam = false;
    }
    if (!_cam && signedMillis() - _camTime >= 4012 && signedMillis() - _camTime < 4013)
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_2_OFF, -1);
      _cam = true;
    }
    if (_cam && signedMillis() - _camTime >= 5012 && signedMillis() - _camTime < 5013)
    {
      digitalWrite(PIN_CAM, HIGH);
      sendEventMarker(EVENT_CAM_TRIG_2_ON, -1);
      _cam = false;
    }
    if (!_cam && signedMillis() - _camTime >= 5014 && signedMillis() - _camTime < 5015)
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_2_OFF, -1);
      _cam = true;
    }
    if (_cam && signedMillis() - _camTime >= 6016 && signedMillis() - _camTime < 6017)
    {
      digitalWrite(PIN_CAM, HIGH);
      sendEventMarker(EVENT_CAM_TRIG_2_ON, -1);
      _cam = false;
    }
    if (!_cam && signedMillis() - _camTime >= 6018 && signedMillis() - _camTime < 6019)
    {
      digitalWrite(PIN_CAM, LOW);
      sendEventMarker(EVENT_CAM_TRIG_2_OFF, -1);
      _cam = true;
    }


    //optogenetics stimulation on each loop
    //turn on opto
    if (_params[OPTO] == 1 && signedMillis() - _timePreCue >= _optoTime && !_optoStarted && _randOptoGate)
    {
      _timeOptoOn = signedMillis();
      optoLED(true);
      _optoStarted = true;
      _optoComplete = false;
    }
    //turn off opto
    if (_params[OPTO] == 1 && signedMillis() - _timeOptoOn >= _optoDuration && !_optoComplete)
    {
      optoLED(false);
      _optoComplete = true;
    }

    // 3) Update state machine
    // Depending on what state we're in, call the appropriate state function, which will evaluate the transition conditions, and update the `_state` var to what the next state should be
    switch (_state)
    {
      case _STATE_INIT:
        state_idle();
        break;

      case STATE_IDLE:
        state_idle();
        break;

      case STATE_PRE_CUE:
        state_pre_cue();
        break;

      case STATE_POST_CUE:
        state_post_cue();
        break;

    }
  }
}


/*****************************************************
  States for the State Machine
*****************************************************/

/*****************************************************
  IDLE - await GO command from host
*****************************************************/
void state_idle()
{
  /*****************************************************
    ACTION LIST
  *****************************************************/
  if (_state != _prevState)
  {
    // Register new state
    _prevState = _state;
    sendState(_state);

    // Reset output
    noTone(PIN_SPEAKER);
    setReward(false);
  }


  /*****************************************************
    OnEachLoop checks
  *****************************************************/
  // Received new param from host: format "P _paramID _newValue" ('P' for Parameters)
  if (_command == 'P')
  {
    _params[_arguments[0]] = _arguments[1];
  }

  /*****************************************************
    TRANSITION LIST
  *****************************************************/
  // GO signal from host --> STATE_PRE_CUE
  if (_command == 'G')
  {
    _state = STATE_PRE_CUE;
    return;
  }

  _state = STATE_IDLE;
}


/*****************************************************
    STATE_PRE_CUE - Trial started
*****************************************************/
void state_pre_cue()
{
  static bool isParamsUpdateStarted;
  static bool isParamsUpdateDone;
  static long randNumber;
  static long randNumber2;
  static long randDuration;

  /*****************************************************
      ACTION: play sound and deliver reward
  *****************************************************/
  if (_state != _prevState)
  {
    // Register new state
    _prevState = _state;
    sendState(_state);

    // Register pre_cue onset time
    _timePreCue = signedMillis();
    sendEventMarker(EVENT_PRE_CUE_ON, -1);

    _cam = true;

    _optoStarted = false;
    randNumber = random(0, 100);
    randDuration = random(200, 500);
    if (randNumber <= _params[OPTO_PERCENT])
    {
      _randOptoGate     = true;
      _optoTime = _params[OPTO_TIME];
    }
    else
    {
      _randOptoGate     = false;
    }

    if (_params[OPTO_DURATION_RAND] == 0)
    {
      _optoDuration = _params[OPTO_DURATION];

    }
    else if (_params[OPTO_DURATION_RAND] == 1)
    {
      _optoDuration = randDuration;
    }

    //determine opto onset
    //        randNumber2 = random(0,400);
    //        if (randNumber2 < 100)
    //        {
    //
    //        // _optoTime = _optoTime1;
    //        _optoDuration = 450;
    //        }
    //        else if (randNumber2 < 200)
    //        {
    //          //_optoTime = 2950;
    //          _optoDuration = 450;
    //        }
    //        else if (randNumber2 < 300)
    //        {
    //          //_optoTime = 3050;
    //          _optoDuration = 450;
    //        }
    //        else
    //        {
    //          //_optoTime = _optoTime2;
    //          _optoDuration = 450;
    //        }
    //
    // Variables for handling parameter update
    isParamsUpdateStarted = false;
    isParamsUpdateDone = false;
  }

  /*****************************************************
      OnEachLoop checks
  *****************************************************/
  // Received new param from HOST: format "P paramID newValue"
  // Mark transmission start. Don't start next trial until we've finished.
  if (_command == 'P')
  {
    isParamsUpdateStarted = true;
    _params[_arguments[0]] = _arguments[1];
  }

  // Parameter transmission complete:
  if (_command == 'O')
  {
    isParamsUpdateDone = true;
  }


  /*****************************************************
      TRANSITION LIST
  *****************************************************/
  // Quit signal from host --> IDLE
  if (_command == 'Q')
  {
    _state = STATE_IDLE;
    return;
  }


  if (isParamsUpdateDone || !isParamsUpdateStarted)
  {
    if (signedMillis() - _timePreCue >= _params[PRE_CUE_DURATION])
    {

      _state = STATE_POST_CUE;
      return;
    }
  }

}

/*****************************************************
    INTERTRIAL
*****************************************************/
void state_post_cue()
{
  static bool isToneOn;
  static long timeRewardOn;
  static bool isRewardOn;
  static bool isRewardComplete;
  static long timeIntertrial;
  static long randomDelay;
  static long randTone;

  /*****************************************************
      ACTION LIST
  *****************************************************/
  if (_state != _prevState)
  {
    // Register new state
    _prevState = _state;
    sendState(_state);

    // Register events
    sendEventMarker(EVENT_POST_CUE_ON, -1);

    // Register time of state entry

    // Generate random interval length
    randomDelay = random(_params[POST_CUE_MIN], _params[POST_CUE_MAX]);

    // Reset reward variables
    timeRewardOn = 0;
    isRewardOn = false;
    isRewardComplete = false;

    //audio cue
    noTone(PIN_SPEAKER);
    isToneOn = false;
    _timeCueOn = signedMillis(); // Register cue on time


    if (_params[TONE_VARIABLE] == 1)
    {
      randTone     = random(0, 100);
    }
    if (_params[TONE_VARIABLE] == 0)
    {
      randTone     = 100;
    }



    if (_params[TONE_ENABLE] == 1 & randTone > 50 )
    {
      tone(PIN_SPEAKER, _params[TONE_FREQ], _params[TONE_DURATION]);
      sendEventMarker(EVENT_CUE_ON, -1);
      digitalWrite(PIN_CUE_MIR, HIGH);
      isToneOn = true;
    }
  }


  //tell matlab when the tone is off
  if (getTimeSinceCueOn() >= _params[TONE_DURATION] && isToneOn)
  {
    sendEventMarker(EVENT_CUE_OFF, -1);
    digitalWrite(PIN_CUE_MIR, LOW);
    isToneOn = false;
  }

  // Turn on reward
  if (!isRewardOn && !isRewardComplete)
  {
    timeRewardOn = getTimeSinceCueOn();
    isRewardOn = true;
    if (_params[REWARD_DURATION] > 0)
    {
      setReward(true);
      digitalWrite(PIN_REW_MIR, HIGH);
    }
  }

  // Turn off reward when the time comes
  if (isRewardOn && !isRewardComplete && getTimeSinceCueOn() - timeRewardOn >= _params[REWARD_DURATION])
  {
    isRewardOn = false;
    isRewardComplete = true;
    if (_params[REWARD_DURATION] > 0)
    {
      setReward(false);
      digitalWrite(PIN_REW_MIR, LOW);
    }
  }

  /*****************************************************
      TRANSITION LIST
  *****************************************************/
  // Quit signal from host --> IDLE
  if (_command == 'Q')
  {
    _state = STATE_IDLE;
    return;
  }

  // If ITI elapsed --> RANDOM_DELAY
  if (getTimeSinceCueOn() >= randomDelay)
  {
    // Send results
    sendResultCode(_resultCode);

    _state = STATE_PRE_CUE;
    return;
  }
}

/*****************************************************
    HARDWARE CONTROLS
*****************************************************/

// Lick detection
bool getLickState()
{
  if (digitalRead(PIN_LICK) == HIGH)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//with ohoto detection
bool getLickState2()
{
  if (digitalRead(PIN_LICK_PHOTO) == LOW)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Must be called once and only once on each loop. Returns true during lick onset
void handleLick()
{
  if (getLickState() && !_isLicking)
  {
    _isLicking = true;
    _isLickOnset = true;
    sendEventMarker(EVENT_LICK, -1);
  }
  else
  {
    if (!getLickState() && _isLicking)
    {
      _isLicking = false;
      sendEventMarker(EVENT_LICK_OFF, -1);
    }
    _isLickOnset = false;
  }
}

//for lick photo detection
void handleLick2()
{
  if (getLickState2() && !_isLicking2)
  {
    _isLicking2 = true;
    _isLickOnset2 = true;
    sendEventMarker(EVENT_LICK_PHOTO, -1);
  }
  else
  {
    if (!getLickState2() && _isLicking2)
    {
      _isLicking2 = false;
      sendEventMarker(EVENT_LICK_OFF_PHOTO, -1);
    }
    _isLickOnset2 = false;
  }
}


// Toggle juice valve, register event when state is changed
void setReward(bool turnOn)
{
  static bool rewardOn = false;
  if (turnOn)
  {


    digitalWrite(PIN_REWARD, HIGH);
    if (!rewardOn)
    {
      rewardOn = true;
      sendEventMarker(EVENT_REWARD_ON, -1);
    }
  }
  else
  {
    digitalWrite(PIN_REWARD, LOW);
    if (rewardOn)
    {
      rewardOn = false;
      sendEventMarker(EVENT_REWARD_OFF, -1);
    }
  }
}

// Toggle OPTO LED, register event when state is changed
void optoLED(bool turnOn)
{
  static bool ledOn = false;
  if (turnOn)
  {
    digitalWrite(PIN_OPTO, HIGH);
    if (!ledOn)
    {
      ledOn = true;
      sendEventMarker(EVENT_OPTO_ON, -1);
    }
  }
  else
  {
    digitalWrite(PIN_OPTO, LOW);
    if (ledOn)
    {
      ledOn = false;
      sendEventMarker(EVENT_OPTO_OFF, -1);
    }
  }
}



/*****************************************************
    SERIAL COMMUNICATION TO HOST
*****************************************************/

//SEND MESSAGE to HOST
void sendMessage(String message)    // Uses String object from arduino library
{
  Serial.println(message);
}

// Register eventMarker on host.
// If timestamp argument is negative, use current time
void sendEventMarker(EventMarker eventMarker, long timestamp)
{
  if (timestamp == -1)
  {
    sendMessage("&" + String(eventMarker) + " " + String(getTime()));
  }
  else
  {
    sendMessage("&" + String(eventMarker) + " " + String(timestamp));
  }
}

void sendState(State state)
{
  sendMessage("$" + String(state));
}

void sendResultCode(int resultCode)
{
  if (resultCode >= 0)
  {
    sendMessage("`" + String(resultCode));
  }
  else
  {
    sendMessage("ERROR: Invalid result code.");
  }
}

// GET COMMAND FROM HOST (single character)
char getCommand(String message)
{
  message.trim();             // Remove leading and trailing white space
  return message[0];          // 1st character in a message string is the command
}

// GET ARGUMENTS (of the command) from HOST (2 int array)
void getArguments(String message, int *_arguments)
{
  _arguments[0] = 0;
  _arguments[1] = 0;

  message.trim();             // Remove leading and trailing white space

  // Remove command (first character) from string
  String parameters = message;
  parameters.remove(0, 1);
  parameters.trim();

  // Parse first (optional) integer argument if it exists
  String intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0])))
  {
    intString += parameters[0];
    parameters.remove(0, 1);
  }
  _arguments[0] = intString.toInt();


  // Parse second (optional) integer argument if it exists
  parameters.trim();
  intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0])))
  {
    intString += parameters[0];
    parameters.remove(0, 1);
  }
  _arguments[1] = intString.toInt();
}

// Send States, Names/Value of Parameters to host
// Arduino uses 0-based indexing and we're sending the indices as is - they will be converted to 1-based indexing by MATLAB
void hostInit()
{
  // Send state names
  for (int iState = 0; iState < _NUM_STATES; iState++)
  {
    sendMessage("@ " + String(iState) + " " + _stateNames[iState] + " " + String(_stateCanUpdateParams[iState]));
  }

  // Send event marker names
  for (int iCode = 0; iCode < _NUM_OF_EVENT_MARKERS; iCode++)
  {
    sendMessage("+ " + String(iCode) + " " + _eventMarkerNames[iCode]);
  }

  // Send param names and default values
  for (int iParam = 0; iParam < _NUM_PARAMS; iParam++)
  {
    sendMessage("# " + String(iParam) + " " + _paramNames[iParam] + " " + String(_params[iParam]));
  }

  // Send result code names
  for (int iCode = 0; iCode < _NUM_RESULT_CODES; iCode++)
  {
    sendMessage("* " + String(iCode) + " " + _resultCodeNames[iCode]);
  }
  sendMessage("~");   // Tells PC that Arduino is ready
}


/*****************************************************
    MISC
*****************************************************/
// `signed long` version of `millis()`
long signedMillis()
{
  long time = (long)(millis());
  return time;
}

// Returns time since last reset in milliseconds
long getTime()
{
  long time = signedMillis() - _timeReset;
  return time;
}

// Returns time since cue on in milliseconds
long getTimeSinceCueOn()
{
  long time = signedMillis() - _timeCueOn;
  return time;
}
