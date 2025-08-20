/*********************************************************************
  Arduino state machine code for Optogenetics Controller

  Adapted from ahamilos
  Optogenetics Control System - Nigel Hunter (nigel_hunter@hms.harvard.edu )
  Matlab Serial Communication Interface - Ofer Mazor

  created         5/13/2025
  Last Modified   5/13/2025   Version code "MAY 13 2025 NH" */

// === Version ===

static String versionCode = "PULSE_MODE_MAY_2025";

/* Update HX:
  - Converted Duty cycle sytle to pulse style
  - added dedicated state for chrimson and chr2 stimulation
  - added an opto-ladder state for optotagging with different light levels
  ------------------------------------------------------------------
  COMPATIBILITY REPORT:
    Matlab HOST: Matlab 2016a - FileName = OptogeneticsController.m (depends on ArduinoConnection.m)
    Arduino:
      Default: TEENSY
      Others:  UNO, TEENSY, DUE, MEGA
  ------------------------------------------------------------------
  Reserved:

    Event Markers: 0-16
    States:        0-8
    Result Codes:  0-3
    Parameters:    0-24
  ------------------------------------------------------------------
  Task Architecture: 

  --------------------------------------------------------------------
  States:
    0: _INIT                (private) 1st state in init loop, sets up communication to Matlab HOST
    1: IDLE_STATE           Awaiting command from Matlab HOST to begin experiment
  ---------------------------------------------------------------------
  Result Codes:
    0: CODE_CORRECT         First lick within response window               
    1: CODE_EARLY_LICK      Early lick -> Abort (Enforced No-Lick Only)
    2: CODE_LATE_LICK       Late Lick  -> Abort (Operant Only)
    3: CODE_NO_LICK         No Response -> Time Out
  ---------------------------------------------------------------------
  Parameters:
    0:  _DEBUG              (private) 1 to enable debug messages to HOST
    1:  HYBRID              1 to overrule pav/op - is op if before target, pav if target reached
    2:  PAVLOVIAN           1 to enable Pavlovian Mode
    3:  OPERANT             1 to enable Operant Mode
    4:  ENFORCE_NO_LICK     1 to enforce no lick in the pre-window interval
    5:  INTERVAL_MIN        Time to start of reward window (ms)

  ---------------------------------------------------------------------
      Incoming Message Syntax: (received from Matlab HOST)
      "(character)#"        -- a command
      "(character int1 int2)# -- update parameter (int1) to new value (int2)"
      Command characters:
        P  -- parameter
        O# -- HOST has received updated paramters, may resume trial
        Q# -- quit and go to IDLE_STATE
        G# -- begin trial (from IDLE_STATE)
        C  -- CHRIMSON-1 request from HOST
        D  -- CHR2 request from HOST
        L  -- opto ladder
  ---------------------------------------------------------------------
    Outgoing Message Syntax: (delivered to Matlab HOST)
      ONLINE:  
        "~"                           Tells Matlab HOST arduino is running
      STATES:
        "@ (enum num) stateName"      Defines state names for Matlab HOST
        "$(enum num) num num"         State-number, parameter, value
 -                                          param = 0: current time
 -                                          param = 1: result code (enum num)
      EVENT MARKERS:
        "+(enum num) eventMarker"     Defines event markers with string
        "&(enum num) timestamp"       Event Marker with timestamp

      RESULT CODES:
      "* (enum num) result_code_name" Defines result code names with str 
      "` (enum num of result_code)"   Send result code for trial to Matlab HOST

      MESSAGE:
        "string"                      String to Matlab HOST serial monitor (debugging) 

      RECEIPT OF STIM:
        "R"             Arduino has received C from HOST
*********************************************************************/
/*****************************************************
  Global Variables
*****************************************************/
/*****************************************************
Arduino Pin Outs (Mode: TEENSY)
*****************************************************/
// Digital IN -- Receives Opto Request form Boss Arduino (CHRIMSON ONLY)
#define PIN_CHRIMSON_TRIGGER               25       // CHRIMSON Trigger Pin    (DUE = 34)  (MEGA = 34)  (UNO = 5?)  (TEENSY = 6?)
#define PIN_TRIAL_INIT                      5       // Aligns Trial Start  (DUE = 34)  (MEGA = 34)  (UNO = 5?)  (TEENSY = 6?)
#define PIN_CUE_ON                         26       // Aligns Cue On       (DUE = 34)  (MEGA = 34)  (UNO = 5?)  (TEENSY = 6?)
#define PIN_CANCEL                         20       // Cancels ongoing stim if high                             (TEENSY = 6?)

// Digital OUT -- Controls Chrimson LED and Sends Stim Receupt 
#define PIN_CHRIMSON_STIM                   23      // I think this is supposed to control the LED, but i'm going to make it just a digital readout of the Analog 
#define PIN_CHRIMSON_RECEIPT                 7      // Chrimson Receipt Pin    (DUE = 35)  (MEGA = 28)  (UNO =  4)  (TEENSY = 4)
#define PIN_CHR2_STIM                       22      //  Digital Read out of ChR2 analog stimulation -- fed to NIDAQ
#define PIN_CHR2_RECEIPT                    5
#define PIN_PHOTOMETRY_LED                  21      // TTL to photometry LEDS -- Control the Photometry LED Gating

//Analog Output Stuff
#define ANALOG_WRITE_RESOLUTION             12      // 12bits: 0-4095

// ANALOG OUT (DAC)
#define PIN_CHRIMSON_PWR			              A21    // controls Chrimson LED
#define PIN_CHR2_PWR			                  A22    // controls ChR2 LED

/*****************************************************
Enums - DEFINE States
*****************************************************/
// All the states
enum State
{
  _INIT,                                          // (Private) Initial state used on first loop. 
  IDLE_STATE,                                     // Idle state. Wait for go signal from host.
  INIT_EXP,                                       // House lamp OFF, random delay before cue presentation
  WAIT_REQUEST_STATE,                             // waiting for BOSS arduino to tell when to stim
  CHRIMSON_STIM_STATE,                            // In between Trial Chrimson Stimulation
  CHR2_STIM_STATE,
  OPTO_LADDER_STATE,
  DAN_TAG_STATE,                                  // called my matlab contains instructions for Striatum opto ladder -> SNc ->  Opto ladder
  UPDATE_PARAMS_STATE,                            // 
  _NUM_STATES                                     // (Private) Used to count number of states
};


// State names stored as strings, will be sent to host
// Names cannot contain spaces!!!
static const char *_stateNames[] = 
{
  "_INIT",
  "IDLE",
  "INIT_EXP",
  "WAIT_REQUEST", 
  "CHRIMSON_STIM", 
  "CHR2_STIM",        
  "OPTO_LADDER_STIM",
  "DAN_TAG_STIM",
  "UPDATE_PARAMS"
};

// Define which states allow param update
static const int _stateCanUpdateParams[] = {1,1,1,1,1,1,1,1}; 
// Defined to allow Parameter upload from host at ANY time

/*****************************************************
Event Markers
*****************************************************/
enum EventMarkers
/* You may define as many event markers as you like.
    Assign event markers to any IN/OUT event
    Times and trials will be defined by global time, 
    which can be parsed later to validate time measurements */
{ // Allsion Events -- Important for Stim in between STMT Trials
  EVENT_EXP_INIT,       // Begin exp
  EVENT_TRIAL_INIT,         // Begin trial
  EVENT_CUE_ON,             // Begin cue presentation
  
  EVENT_BOSS_CHRIMSON_REQUEST,  // Boss request CHRIMSON
  EVENT_BOSS_CHRIMSON_STIM_BEGIN,
  EVENT_BOSS_CHRIMSON_STIM_END,
 
  EVENT_BOSS_CHR2_REQUEST,       // Boss request CHR2
  EVENT_BOSS_CHR2_STIM_BEGIN,
  EVENT_BOSS_CHR2_STIM_END,
  
  EVENT_UI_CHRIMSON_REQUEST,    // UI request CHRIMSON
  EVENT_UI_CHRIMSON_STIM_BEGIN, 
  EVENT_UI_CHRIMSON_STIM_END,
  EVENT_CHRIMSON_ERROR,
  
  EVENT_UI_CHR2_REQUEST,        // UI request CHR2
  EVENT_UI_CHR2_STIM_BEGIN, 
  EVENT_UI_CHR2_STIM_END,
  EVENT_CHR2_ERROR,
  
  
  EVENT_UI_OPTOTAG,     // Begin optotag ladder

  EVENT_PHOTOMETRY_ON,     // LED on
  EVENT_PHOTOMETRY_OFF,     // LED off

  TOGGLE_LED_TIMER_STARTED,     // started a timer after which photometry LED will toggle

  // LingFeng Events -- Important for Opto-Ladder and SNc Stimulation
  EVENT_CHRIMSON_STIM_ON,			// Begin optogenetic stim (single pulse start)
	EVENT_CHRIMSON_STIM_OFF,			// End optogenetic stim (single pulse end)
	EVENT_CHRIMSON_STIM_START,		// Begin optogenetic stim (pulse train start)
	EVENT_CHRIMSON_STIM_END,			// End optogenetic stim (pulse train end)
	EVENT_CHR2_STIM_ON,			// Begin optogenetic stim (single pulse start)
	EVENT_CHR2_STIM_OFF,			// End optogenetic stim (single pulse end)
	EVENT_CHR2_STIM_START,		// Begin optogenetic stim (pulse train start)
	EVENT_CHR2_STIM_END,			// End optogenetic stim (pulse train end)

  _NUM_OF_EVENT_MARKERS
};

static const char *_eventMarkerNames[] =    // * to define array of strings
{
  "EXP_INIT",
  "TRIAL_INIT",
  "CUE_ON",
  
  "BOSS_CHRIMSON_REQUEST",
  "BOSS_CHRIMSON_STIM_BEGIN",
  "BOSS_CHRIMSON_STIM_END",
  
  "UI_CHRIMSON_REQUEST",
  "UI_CHRIMSON_STIM_BEGIN",
  "UI_CHRIMSON_STIM_END",
  "EVENT_UI_CHR2_REQUEST",
  "EVENT_UI_CHR2_STIM_BEGIN", 
  "EVENT_UI_CHR2_STIM_END",
  "EVENT_CHR2_ERROR",
  "UI_OPTOTAG",

  "EVENT_PHOTOMETRY_ON",
  "EVENT_PHOTOMETRY_OFF",
  "TOGGLE_LED_TIMER_STARTED"

  "CHRIMSON_STIM_ON",		
  "CHRIMSON_STIM_OFF",		
  "CHRIMSON_STIM_START",	
  "CHRIMSON_STIM_END",		
  "CHR2_STIM_ON",		
  "CHR2_STIM_OFF",		
  "CHR2_STIM_START",	
  "CHR2_STIM_END",		
};


/*****************************************************
Result codes
*****************************************************/
enum ResultCode
{
  CODE_CHRIMSON,                // CHRIMSON Stimulated
  CODE_CHR2,                    // CHR2 Stimulated
  CODE_CHRIMSON_CANCEL,         // CHRIMSON in behavior cancelled by event (e.g., first lick)
  CODE_CHR2_CANCEL,
  CODE_UI_CHRIMSON,             // UI-commanded CHRIMSON Stimulated
  CODE_UI_CHR2,                 // UI_CHR2 stimulation
  CODE_OPTOTAG,             // Completed optotagging ladder
  CODE_DANTAG,
  _NUM_RESULT_CODES         // (Private) Used to count how many codes there are.
};

// We'll send result code translations to MATLAB at startup
static const char *_resultCodeNames[] =
{
  "CHRIMSON",
  "CHR2",
  "CHRIMSON_CANCELLED",
  "CHR2_CANCELLED",
  "UI_CHRIMSON",
  "UI_CHR2",
  "OPTOTAG",
  "DANTAG"
};

/*****************************************************
Parameters that can be updated by HOST
*****************************************************/
enum ParamID {
  _DEBUG,                                        // (Private) 1 to enable debug messages from HOST. Default 
  OPTO_ENABLED,                                  // Enables optogenetic from lingfeng two color experiment
  CANCEL_ENABLED,                                // Uses cancellation pin o halt stimulation during trials -- current not in use

  // Pulse-mode parameters for Chrimson
  CHRIMSON_NUM_PULSES,                           //  How many pulses to perform                          
  CHRIMSON_PULSE_WIDTH,                          //  How wide each pulse should be
  CHRIMSON_IPI,                                  //  How long to wait in between pulses 

  // Pulse-mode parameters for ChR2
  CHR2_NUM_PULSES,
  CHR2_PULSE_WIDTH,
  CHR2_IPI,

  CHRIMSON_CHR2_NUM_PULSES,                     // How many pulses to perform in the DAN TAGGING
  CHRIMSON_CHR2_PULSE_WIDTH,                    // How wide each pulse should be
  CHRIMSON_CHR2_IPI,      
  
  // Ladder control
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

                         // Analog output value to use for modulation of LED power

  STIM_TRAIN_DELAY,                             // How long to wait in between trains Chrimson train(s) -(train delay)-> Chr2 train(s) -(train delay)-> Chrimson train(s)
  PHOTOMETRY_ON,                                // Controls photometry LED
  PHOT_TOGGLE_DELAY,
  CHRIMSON_TRAINS_BEFORE_CHR2,
  CHRIMSON_TRAINS_AFTER_CHR2,
  CHR2_NUM_TRAINS,
  _NUM_PARAMS
};

// === Parameter Names (for Host Interface) ===
static const char *_paramNames[] = {
  "_DEBUG",
  "OPTO_ENABLED",
  "CANCEL_ENABLED",
  "CHRIMSON_NUM_PULSES",
  "CHRIMSON_PULSE_WIDTH",
  "CHRIMSON_IPI",
  "CHR2_NUM_PULSES",
  "CHR2_PULSE_WIDTH",
  "CHR2_IPI",
  "CHRIMSON_CHR2_NUM_PULSES",                     // How many pulses to perform in the DAN TAGGING
  "CHRIMSON_CHR2_PULSE_WIDTH",                    // How wide each pulse should be
  "CHRIMSON_CHR2_IPI",   
  "CHRIMSON_AOUT_1_VALUE",
  "CHRIMSON_AOUT_2_VALUE",
  "CHRIMSON_AOUT_3_VALUE",
  "CHRIMSON_AOUT_4_VALUE",
  "CHRIMSON_AOUT_5_VALUE",
  "CHRIMSON_AOUT_6_VALUE",
  "CHR2_AOUT_1_VALUE",
  "CHR2_AOUT_2_VALUE",
  "CHR2_AOUT_3_VALUE",
  "CHR2_AOUT_4_VALUE",
  "CHR2_AOUT_5_VALUE",
  "CHR2_AOUT_6_VALUE",
  "STIM_TRAIN_DELAY",
  "PHOTOMETRY_ON",
  "PHOT_TOGGLE_DELAY",
  "CHRIMSON_TRAINS_BEFORE_CHR2",
  "CHRIMSON_TRAINS_AFTER_CHR2",
  "CHR2_NUM_TRAINS"
};

// === Parameter Values ===
float _params[_NUM_PARAMS] = {
  1,    // _DEBUG
  1,    // OPTO_ENABLED
  1,    // CANCEL_ENABLED

  5,    // CHRIMSON_NUM_PULSES
  10,   // CHRIMSON_PULSE_WIDTH (ms)
  100,  // CHRIMSON_IPI (ms)
  // Joshua Dudman 2021 Cell Reports
  5,    // CHR2_NUM_PULSES
  5,   // CHR2_PULSE_WIDTH (ms)
  35,  // CHR2_IPI (ms)

  200, // CHRIMSON_CHR2_NUM_PULSES,                     // How many pulses to perform in the DAN TAGGING
  10, // CHRIMSON_CHR2_PULSE_WIDTH,                    // How wide each pulse should be
  3000, // CHRIMSON_CHR2_IPI,  

  100,  // CHRIMSON_AOUT_1_VALUE
  200, // CHRIMSON_AOUT_2_VALUE
  400, // CHRIMSON_AOUT_3_VALUE
  800, // CHRIMSON_AOUT_4_VALUE
  1600, // CHRIMSON_AOUT_5_VALUE
  2000, // CHRIMSON_AOUT_6_VALUE
  100,  // CHR2_AOUT_1_VALUE
  200, // CHR2_AOUT_2_VALUE
  400, // CHR2_AOUT_3_VALUE
  800, // CHR2_AOUT_4_VALUE
  1600, // CHR2_AOUT_5_VALUE
  2000, // CHR2_AOUT_6_VALUE

  1000, // STIM_TRAIN_DELAY (ms)
  0,    // PHOTOMETRY_ON
  1000, // PHOT_TOGGLE_DELAY

  2,    // CHRIMSON_TRAINS_BEFORE_CHR2
  2,    // CHRIMSON_TRAINS_AFTER_CHR2
  1     // CHR2_NUM_TRAINS
};

/*****************************************************
Other Global Variables 
*****************************************************/
// === HOST  === //
// Variables declared here can be carried to the next loop, AND read/written in function scope as well as main scope
// (previously defined):
static State _state                   = _INIT;    // This variable (current _state) get passed into a _state function, which determines what the next _state should be, and updates it to the next _state.
static State _prevState               = _INIT;    // Remembers the previous _state from the last loop (actions should only be executed when you enter a _state for the first time, comparing currentState vs _prevState helps us keep track of that).
static char _command                  = ' ';      // Command char received from host, resets on each loop
static int _arguments[2]              = {0};      // Two integers received from host , resets on each loop
static int _argcopy[2]				        = {0};
static int _resultCode                = -1;   // Reset result code

// === Trial Stimulation Logic  === //
// variables here tell communicates between boss and this arduino of when to start/end stim and what type of stim is happening
static signed long _exp_timer           = 0;
static bool _CHRIMSON_trigger_on          = false;
static bool _trial_is_stimulated      = false;
static bool _stimulation_requested    = false;
static bool _CHRIMSON_receipt_received    = false;
static bool _BOSS_CHRIMSON_stim_in_progress = false;
static bool _BOSS_CHR2_stim_in_progress = false;
static bool _UI_CHRIMSON_stim_in_progress = false;
static bool _UI_CHR2_stim_in_progress = false;
static bool singleTrainMode = false;

// === Pulse Logic  === //
unsigned long pulseTimer = 0;
bool inPulse = false;
int pulseIndex = 0;
int currentTrain = 0;
int currentAout = 0;
bool delayAfterPhase = false;
unsigned long delayStartTime = 0;
int phase = 0;
unsigned long ipiTimer = 0;
unsigned long chR2ipiTimer = 0;
float chrimsonLadder[5];
float chr2Ladder[5];
bool dingdong = false; // toggle for ChR2 and Chrimson

// === Parameter Updates === //
static bool _hold                     = false;
static bool _isParamsUpdateStarted    = false;
static bool _isParamsUpdateDone       = true;
static bool _ready_for_next_trial     = true;
static bool _ready_for_cue            = true;
static long _timeReset				= 0;			// Reset to signedMillis() at every soft reset

// === Photometry Controls === //
static signed long _photometry_toggle_timer = 0;
static bool _photometry_LED_enabled = true;


/*****************************************************
  INITIALIZATION LOOP
*****************************************************/
void setup()
{
  //--------------------I/O initialization------------------//
  // OUTPUTS
  pinMode(PIN_CHRIMSON_STIM, OUTPUT);                       // Digital Record of Chrimson stimulation
  pinMode(PIN_CHRIMSON_RECEIPT, OUTPUT);                    // SEND message to BOSS arduino that stim over
  pinMode(PIN_CHR2_STIM, OUTPUT);                           // Digital Record of CHR2
  pinMode(PIN_CHR2_RECEIPT, OUTPUT);                    // SEND message to BOSS arduino that stim over
  pinMode(PIN_PHOTOMETRY_LED, OUTPUT);                      // Toggles Photometry
  // INPUTS
  pinMode(PIN_CHRIMSON_TRIGGER, INPUT);                     // Receives Stim Request from Boss
  pinMode(PIN_TRIAL_INIT, INPUT);                           // Aligns Trial Start -> T with house lamp on Boss
  pinMode(PIN_CUE_ON, INPUT);                               // Aligns Cue -> T with Cue on Boss
  pinMode(PIN_CANCEL, INPUT);                               // Turns off STIM
  // ANALOG
	analogWriteResolution(ANALOG_WRITE_RESOLUTION);           // Allows analogwrite??
  pinMode(PIN_CHRIMSON_PWR,OUTPUT);                         // Controls Chrimson LED power
  pinMode(PIN_CHR2_PWR,OUTPUT);                             // Controls CHR2 LED power
  //--------------------------------------------------------//

  //------------------------Serial Comms--------------------//
  Serial.begin(115200);                       // Set up USB communication at 115200 baud 

} // End Initialization Loop -----------------------------------------------------------------------------------------------------

void mySetup()
{

  //--------------Set ititial OUTPUTS----------------//
  setChrimsonLED(false);           //turn off chrimson LED
  setChrimsonReceipt(false);       // Reset Chrisom stim receipt flag
  setChR2LED(false);               // turn off Chr2 LED
  setChR2Receipt(false);       // Reset ChR2 stim receipt flag
  
  //---------------------------Reset a bunch of variables---------------------------//
  _state                    = _INIT;    // This variable (current _state) get passed into a _state function, which determines what the next _state should be, and updates it to the next _state.
  _prevState                = _INIT;    // Remembers the previous _state from the last loop (actions should only be executed when you enter a _state for the first time, comparing currentState vs _prevState helps us keep track of that).
  _command                  = ' ';      // Command char received from host, resets on each loop
  _arguments[0]             = 0;        // Two integers received from host , resets on each loop
  _arguments[1]             = 0;        // Two integers received from host , resets on each loop
  _resultCode               = -1;     // Reset result code

  // Reset global variables
  _exp_timer        = 0;
  //_time_CHRIMSON_stimulated   = 0;
  //_CHRIMSON_refractory_clock  = 0;
  //_duty_timer_CHRIMSON        = 0;
  _photometry_toggle_timer  = 0;

  if (_params[_DEBUG]) {
    sendMessage("Initial CHRIMSON pulse width = " + String(_params[CHRIMSON_PULSE_WIDTH]) + " ms, IPI = " + String(_params[CHRIMSON_IPI]) + " ms.");
  }
  _CHRIMSON_trigger_on        = false;
  _photometry_LED_enabled   = true;
  _trial_is_stimulated    = false;
  _stimulation_requested  = false;
  _CHRIMSON_receipt_received  = false;
  _BOSS_CHRIMSON_stim_in_progress= false;
  _UI_CHRIMSON_stim_in_progress= false;
  //_duty_state_CHRIMSON        = false;

  _hold                   = false;
  _isParamsUpdateStarted  = false;
  _isParamsUpdateDone     = true;
  _ready_for_next_trial   = true;
  _ready_for_cue          = true;
  

  // Tell PC that we're running by sending '~' message:
  hostInit();                         // Sends all parameters, states and error codes to Matlab (LF Function)    

}
/*****************************************************
  MAIN LOOP
*****************************************************/
void loop()
{
 // Initialization
  mySetup();

  // SET 3.3V line to high
  // digitalWrite(PIN_3_3, HIGH);

  // Main loop (R# resets it)
  while (true)
  {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      Step 1: Read USB MESSAGE from HOST (if available)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // 1) Check USB for MESSAGE from HOST, if available. String is read byte by byte. (Each character is a byte, so reads e/a character)

    static String usbMessage  = "";             // Initialize usbMessage to empty string, only happens once on first loop (thanks to static!)
    _command = ' ';                              // Initialize _command to a SPACE
    _arguments[0] = 0;                           // Initialize 1st integer argument
    _arguments[1] = 0;                           // Initialize 2nd integer argument

    if (Serial.available() > 0)  {              // If there's something in the SERIAL INPUT BUFFER (i.e., if another character from host is waiting in the queue to be read)
      char inByte = Serial.read();                  // Read next character
      
      // The pound sign ('#') indicates a complete message!------------------------
      if (inByte == '#')  {                         // If # received, terminate the message
        // Parse the string, and updates `_command`, and `_arguments`
        _command = getCommand(usbMessage);               // getCommand pulls out the character from the message for the _command         
        getArguments(usbMessage, _arguments);            // getArguments pulls out the integer values from the usbMessage
        usbMessage = "";                                // Clear message buffer (resets to prepare for next message)
        if (_command == 'R') {
          break;
        }
      }
      else {
        // append character to message buffer
        usbMessage = usbMessage + inByte;       // Appends the next character from the queue to the usbMessage string
      }
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      Step 1b: OnEachLoop routines
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    handleAnalogOutput();	// Write to DAC channels to modulate laser pwoer

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      Step 2: Update the State Machine
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // Depending on what _state we're in , call the appropriate _state function, which will evaluate the transition conditions, and update `_state` to what the next _state should be
    switch (_state) {
      case _INIT:
        idle_state();
        break;

      case IDLE_STATE:
        idle_state();
        break;
      
      case INIT_EXP:
        init_exp();
        break;

      case WAIT_REQUEST_STATE:
        wait_request_state();
        break;

      case CHRIMSON_STIM_STATE:
        Chrimson_stim_state();
        break;
      case CHR2_STIM_STATE:
        Chr2_stim_state();
        break;
      case OPTO_LADDER_STATE:
        opto_ladder_state();
        break;

      case DAN_TAG_STATE:
        dan_tag_state();
        break;
      case UPDATE_PARAMS_STATE:
        update_params_state();
        break;
    } // End switch statement--------------------------
  }
} // End main loop-------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  IDLE STATE - awaiting start cue from Matlab HOST
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void idle_state() {
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {                      // If ENTERTING IDLE_STATE:
    sendMessage("-");
    sendMessage("Optogenetics_Controller4");
    sendMessage("Version Code: " + versionCode);
    _prevState = _state;                             // Assign _prevState to idle _state
    sendMessage("$" + String(_state));               // Send a message to host upon _state entry -- $1 (Idle State)
    
    // Reset Outputs
    setChrimsonReceipt(false);                       // Turns off receipt pin if on
    setChR2Receipt(false);
    setAnalogOutput(1,0);
    setAnalogOutput(2,0);
    setChrimsonLED(false);
    setChR2LED(false);
    // Reset state variables
    _resultCode = -1;                              // Clear previously registered result code

    //------------------------DEBUG MODE--------------------------//
    if (_params[_DEBUG]) {
      sendMessage("Idle.");
    }  
    //----------------------end DEBUG MODE------------------------//
  }
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TRANSITION LIST -- checks conditions, moves to next state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  // If we get Go -> INIT_EXP where we wait for stim request from boss arduino
  if (_command == 'G') {                           // If Received GO signal from HOST ---transition to---> READY (waiting for request)
    _state = INIT_EXP;                             // State set to INIT_EXP
    return;                                        // Exit function
  }
   // If we get L -> CHRIMSON STIM STATE where we do trains of Chrimson then ChR2 then Chrimson again
  	if (_command == 'C')
  	{
  		_state = CHRIMSON_STIM_STATE;                    // Ladder & SNc State
  		return;
  	}
   // If we get L -> CHRIMSON STIM STATE where we do trains of Chrimson then ChR2 then Chrimson again
  	if (_command == 'D')
  	{
  		_state = CHR2_STIM_STATE;                    // Ladder & SNc State
  		return;
  	}
  
  // If we get L -> OPTO_LADDER STATE where we do trains of Chrimson then ChR2 then Chrimson again
  	if (_command == 'L')
  	{
  		_state = OPTO_LADDER_STATE;                    // Ladder & SNc State
  		return;
    }
    
  // if we get 'M' -> DAN_TAG_STATE where we do trains of Chrimson then ChR2 then Chrimson again}
    if (_command == 'M')
    {
      _state = DAN_TAG_STATE;                    // Ladder & SNc State
      return;
    }

  checkPhotometryLED();
  if (checkParamUpdate()){return;};                // Sends us to the update param state until finished updating.   
  _state = IDLE_STATE;                             // Return to IDLE_STATE
} // End IDLE_STATE ------------------------------------------------------------------------------------------------------------------------------------------



/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  INIT_EXP - Initialized exp, awaiting instruction from Boss arduino

  The only purpose of this state is to (re-)initialize the experiment's timer for plotting and other ref events
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void init_exp() {
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {                       // If ENTERTING READY STATE:
    //-----------------INIT CLOCKS--------------//
    _exp_timer = signedMillis();

    // Send event marker (trial_init) to HOST with timestamp
    sendMessage("&" + String(EVENT_EXP_INIT) + " " + String(signedMillis() - _exp_timer));
    _prevState = _state;                                // Assign _prevState to READY _state
    sendMessage("$" + String(_state));                  // Send  HOST _state entry -- $2 (Ready State)
    
    if (_params[_DEBUG]) {sendMessage("Experiment Started.");}

    _state = WAIT_REQUEST_STATE;            // Go to waiting state
  }
} // End INIT_EXP STATE ------------------------------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  WAIT_REQUEST_STATE - Awaiting instruction from Boss arduino

  We mostly sit in this state, waiting for new instructions from UI or from Boss
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void wait_request_state() {
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {                                                   // If ENTERTING WAITING STATE:
    _prevState = _state;                                                        // Assign _prevState to READY _state
    sendMessage("$" + String(_state));                                          // Send  HOST _state entry
    if (_params[_DEBUG]) {sendMessage("Awaiting command from BOSS arduino.");}
  }
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TRANSITION LIST -- checks conditions, moves to next state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (checkQuit()) {return;}                                                    // If true, quit
  checkTrialStart();                                                            // Check if we're ready for next trial (PIN_TRIAL_INIT is off)
  checkReferenceEvent() ;                                                       // Check if the Cue has gone off
  if (checkParamUpdate()) {return;}                                             // If true, we jump to param update state and hold there till done, at expense of some error in timing of stimulation events, which we will record and warn USER thereof
  if (checkChrimsonRequest()) {return;}                                         // If true, jump to CHRIMSON stimulator state. Check for a new stimulation request. If ongoing stim in progress, this overwrites previous**
  if(checkChr2Request()){return;}                                               // Allows me to do ChR2 stim from UI               
  checkPhotometryLED();   
  if (_command == 'L')
  	{
  		_state = OPTO_LADDER_STATE;                    // Ladder & SNc State
  		return;
  	}                                   
  _state = WAIT_REQUEST_STATE;                                                  // Continue waiting state
} // End WAIT_REQUEST_STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  CHRIMSON_STIM_STATE - Begin stim, send receipt -- Check how stimulation occurs, i'm assuming it looks at the duty cycle and stuff, but since this will be controlled through analog pwr I need to adapt it as such
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void Chrimson_stim_state() {
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {                       // If ENTERTING WAITING STATE:
    _prevState = _state;                                // Assign _prevState to READY _state
    sendMessage("$" + String(_state));                  // Send  HOST _state entry
    if (_params[_DEBUG]) {sendMessage("Stimulating CHRIMSON and sending receipt to BOSS arduino.");}
  }
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TRANSITION LIST -- checks conditions, moves to next state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (checkQuit()) {return;}        // If true, exit
  checkStimChrimson();            // Turns off ongoing stim if time's up
  if (checkChrimsonRequest()) {return;}   // If true, means start new stimulation, overwriting previous
  checkPhotometryLED();
} // End CHRIMSON_STIM_STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  CHR2_STIM_STATE - Begin stim, send receipt -- Check how stimulation occurs, i'm assuming it looks at the duty cycle and stuff, but since this will be controlled through analog pwr I need to adapt it as such
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void Chr2_stim_state() {
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {                       // If ENTERTING WAITING STATE:
    _prevState = _state;                                // Assign _prevState to READY _state
    sendMessage("$" + String(_state));                  // Send  HOST _state entry
    if (_params[_DEBUG]) {sendMessage("Stimulating CHR2 and sending receipt to BOSS arduino.");}
  }
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TRANSITION LIST -- checks conditions, moves to next state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (checkQuit()) {return;}        // If true, exit
  checkStimChR2();            // Turns off ongoing stim if time's up
  if (checkChr2Request()) {return;}   // If true, means start new stimulation, overwriting previous
  checkPhotometryLED();
} // End CHR2_STIM_STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  UPDATE_PARAMS_STATE - Hold the controller until update complete
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void update_params_state() {
  /* Only exit once parameters completed update. Give a warning if stim in progress */
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    if (_hold != true) {
    sendMessage("$" + String(_state));
    _hold = true;
    int id = _argcopy[0];
    float value = _argcopy[1];
    _params[id] = value;
    if (_params[_DEBUG]) sendMessage("Parameter " + String(id) + " updated to " + String(value));
  }
  if (_command == 'O') {
    _isParamsUpdateStarted = false;
    _isParamsUpdateDone = true;
    _state = _prevState;
    _hold = false;
    if (_params[_DEBUG]) sendMessage("Parameter update complete.");
  }
 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    TRANSITION LIST -- checks conditions, moves to next state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (checkQuit()) {return;}
  if (checkUpdateComplete()) {return;}
  _state = UPDATE_PARAMS_STATE;           // If not done, return to the waiting state

} // End UPDATE_PARAMS_STATE ------------------------------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
OPTO_LADDER_STATE -- TRAIN OF 5 PULSES OF INCREASING CHRIMSON LIGHT LEVELS -> CHR2 train -> TRAIN OF CHRIMSON
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void opto_ladder_state() {
  /******************************************************
     VARIABLE DECLARATIONS
  ******************************************************/
  static int phase = 0;
  static bool delayAfterPhase = false;
  static unsigned long delayStartTime = 0;
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {
    _prevState = _state;
    sendMessage("$" + String(_state));
    pulseIndex = 0;
    currentTrain = 0;
    currentAout = 0;
    inPulse = false;
    phase = 0;
    delayAfterPhase = false;

    // Initialize ladder amplitudes
    chrimsonLadder[0] = _params[CHRIMSON_AOUT_1_VALUE];
    chrimsonLadder[1] = _params[CHRIMSON_AOUT_2_VALUE];
    chrimsonLadder[2] = _params[CHRIMSON_AOUT_3_VALUE];
    chrimsonLadder[3] = _params[CHRIMSON_AOUT_4_VALUE];
    chrimsonLadder[4] = _params[CHRIMSON_AOUT_5_VALUE];

    if (_params[_DEBUG]) sendMessage("Performing OptoLadder Experiment.");
  }

  // == Transition between trains logic === //
  switch (phase) {
    case 0:
      if (currentTrain < _params[CHRIMSON_TRAINS_BEFORE_CHR2]) {
        runChrimsonTrain();
      } else if (!delayAfterPhase) {
        delayStartTime = millis();
        delayAfterPhase = true;
        if (_params[_DEBUG]) sendMessage("Phase 0 complete. Delay before Phase 1.");
      } else if ((unsigned long)(millis() - delayStartTime) >= _params[STIM_TRAIN_DELAY]) {
        phase++;
        currentTrain = 0;
        pulseIndex = 0;
        delayAfterPhase = false;
      }
      break;

    case 1:
      if (currentTrain < _params[CHR2_NUM_TRAINS]) {
        runChR2Train();
      } else if (!delayAfterPhase) {
        delayStartTime = millis();
        delayAfterPhase = true;
        if (_params[_DEBUG]) sendMessage("Phase 1 complete. Delay before Phase 2.");
      } else if ((unsigned long)(millis() - delayStartTime) >= _params[STIM_TRAIN_DELAY]) {
        phase++;
        currentTrain = 0;
        pulseIndex = 0;
        delayAfterPhase = false;
      }
      break;

    case 2:
      if (currentTrain < _params[CHRIMSON_TRAINS_AFTER_CHR2]) {
        runChrimsonTrain();
      } else {
        sendMessage("R");
        if (_params[_DEBUG]) sendMessage("Optotagging complete.");
        _state = WAIT_REQUEST_STATE;
        _resultCode = CODE_OPTOTAG;
        sendMessage("`" + String(_resultCode)); 
        if(_params[_DEBUG]) sendMessage("Moving to WAIT_REQUEST_STATE");
      }
      break;
  }
}

void dan_tag_state() {
  /******************************************************
     VARIABLE DECLARATIONS
  ******************************************************/
  static int phase = 0;
  static bool delayAfterPhase = false;
  static unsigned long delayStartTime = 0;
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      ACTION LIST -- initialize the new state
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  if (_state != _prevState) {
    _prevState = _state;
    sendMessage("$" + String(_state));
    pulseIndex = 0;
    currentAout = 0;
    inPulse = false;
    phase = 0;
    delayAfterPhase = false;

        // Initialize ladder amplitudes
    chrimsonLadder[0] = _params[CHRIMSON_AOUT_1_VALUE];
    chrimsonLadder[1] = _params[CHRIMSON_AOUT_2_VALUE];
    chrimsonLadder[2] = _params[CHRIMSON_AOUT_3_VALUE];
    chrimsonLadder[3] = _params[CHRIMSON_AOUT_4_VALUE];
    chrimsonLadder[4] = _params[CHRIMSON_AOUT_5_VALUE];
    chrimsonLadder[5] = _params[CHRIMSON_AOUT_6_VALUE];

    chr2Ladder[0] = _params[CHR2_AOUT_1_VALUE];
    chr2Ladder[1] = _params[CHR2_AOUT_2_VALUE];
    chr2Ladder[2] = _params[CHR2_AOUT_3_VALUE];
    chr2Ladder[3] = _params[CHR2_AOUT_4_VALUE];
    chr2Ladder[4] = _params[CHR2_AOUT_5_VALUE];
    chr2Ladder[5] = _params[CHR2_AOUT_6_VALUE];


    if (_params[_DEBUG]) sendMessage("Performing opto tagging dan params Experiment.");
  }


  if (pulseIndex < _params[CHRIMSON_CHR2_NUM_PULSES]) {
    runChR2ChrimsonPulses();
  } else if (!delayAfterPhase) {
    delayStartTime = millis();
    delayAfterPhase = true;
    if (_params[_DEBUG]) sendMessage("Phase 1 complete. Delay before Phase 2.");
  } else if ((unsigned long)(millis() - delayStartTime) >= _params[STIM_TRAIN_DELAY]) {
    phase++;
    pulseIndex = 0;
    delayAfterPhase = false;
  }

}

/**********************************************************************************************************
  STIMULATION CHECKS -- For In Trial Stimulation + Making Sure Photometry is ON
**********************************************************************************************************/
// === Function: checkChrimsonRequest === //
bool checkChrimsonRequest() {
    if (digitalRead(PIN_CHRIMSON_TRIGGER)) {
      if (_params[_DEBUG]) {sendMessage("Received CHRIMSON request from BOSS arduino. Sending receipt.");}
      sendMessage("&" + String(EVENT_BOSS_CHRIMSON_REQUEST) + " " + String(signedMillis() - _exp_timer));
      _UI_CHRIMSON_stim_in_progress = false;
      _BOSS_CHRIMSON_stim_in_progress = true;
      stimulateChrimson();
      //setChrimsonReceipt(true);
      //sendMessage("&" + String(EVENT_BOSS_CHRIMSON_STIM_BEGIN) + " " + String(_time_CHRIMSON_stimulated - _exp_timer));

      _state = CHRIMSON_STIM_STATE;
      setChrimsonReceipt(true);
      return true;
    }
    else if (_command == 'C') {
      if (_params[_DEBUG]) {sendMessage("Received CHRIMSON request from HOST. Sending receipt to BOSS.");}
      sendMessage("&" + String(EVENT_UI_CHRIMSON_REQUEST) + " " + String(signedMillis() - _exp_timer));
      _UI_CHRIMSON_stim_in_progress = true;
      _BOSS_CHRIMSON_stim_in_progress = false;
      //setChrimsonReceipt(true);
      stimulateChrimson();
      //sendMessage("&" + String(EVENT_UI_CHRIMSON_STIM_BEGIN) + " " + String(_time_CHRIMSON_stimulated - _exp_timer));
      sendMessage("R"); // kill UI request
      if (_params[_DEBUG]) {sendMessage("Killed HOST request with R message.");}
      _command = ' '; // clear it to avoid repeat
      _state = CHRIMSON_STIM_STATE;
      setChrimsonReceipt(true);
      return true;
    }
    else {return false;}
  } 

// === Function: checkChR2Request === //
bool checkChr2Request() {
  if (_command == 'D') {
    if (_params[_DEBUG]) sendMessage("Received CHR2 request from HOST. Sending receipt to BOSS.");
    sendMessage("&" + String(EVENT_UI_CHR2_REQUEST) + " " + String(signedMillis() - _exp_timer));
    _UI_CHR2_stim_in_progress = true;
    _BOSS_CHR2_stim_in_progress = false;
    setChR2Receipt(true);
    stimulateChR2();
    sendMessage("R");
    if (_params[_DEBUG]) sendMessage("Killed HOST request with R message.");
    _command = ' ';  // ✅ CLEAR COMMAND to prevent repeat
    _state = CHR2_STIM_STATE;
    return true;
  }

  return false;
}


// === Function: checkStimChrimson === //
void checkStimChrimson() {
  runChrimsonTrain();
  if (currentTrain >= 1) {
    currentTrain = 0;
    pulseIndex = 0;
    _state = WAIT_REQUEST_STATE;
  }
}

// === Function: checkStimChr2 === //
void checkStimChR2() {
  runChR2Train();
  if (currentTrain >= 1) {
    currentTrain = 0;
    pulseIndex = 0;
    _state = WAIT_REQUEST_STATE;
  }
}

// === Function: checkPhotometryLED ===
void checkPhotometryLED() {
  // first check if we should toggle
  if (_command == 'Y') { // this says switch the params
    if (_params[_DEBUG]) {sendMessage("Received command to toggle LEDs. Attempting to toggle params.");}
    if (_params[PHOTOMETRY_ON] == 1) {
      _params[PHOTOMETRY_ON] = int(0);
      if (_params[_DEBUG]) {sendMessage("New PHOTOMETRY_ON should be 0. It is =" + String(_params[PHOTOMETRY_ON]));}
    }
    else {
      _params[PHOTOMETRY_ON] = int(1);
      if (_params[_DEBUG]) {sendMessage("New PHOTOMETRY_ON should be 1. It is =" + String(_params[PHOTOMETRY_ON]));}
    }
    // update params
    sendMessage("^ " + String(PHOTOMETRY_ON) + " PHOTOMETRY_ON " + String(_params[PHOTOMETRY_ON]));    
    if (_params[_DEBUG]) {sendMessage("Parameter " + String(PHOTOMETRY_ON) + " changed to " + String(_params[PHOTOMETRY_ON]));} 
  }
  else {setPhotometryLED(_params[PHOTOMETRY_ON] == 1, true);}
}

/*****************************************************************
	HARDWARE CONTROLS - turn LEDs on and off & Controls AnalogPower
******************************************************************/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Set Photometry LED (ON/OFF)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setPhotometryLED(bool turnOn, bool isParamChange) {
  // if this set command is 2/2 a param change, ensure the delay has elapsed
  if (isParamChange && signedMillis() - _photometry_toggle_timer < _params[PHOT_TOGGLE_DELAY]) {return;}
  // if delay complete OR if was demanded from MATLAB UI, proceed
  if (turnOn) {
    digitalWrite(PIN_PHOTOMETRY_LED,HIGH);//************** CHANGE HERE//PIN_PHOTOMETRY_LED, HIGH);
    // sendMessage("&" + String(EVENT_PHOTOMETRY_ON) + " " + String(signedMillis() - _exp_timer));
    _photometry_LED_enabled = true;
  }
  else {
    digitalWrite(PIN_PHOTOMETRY_LED, LOW);//************** CHANGE HERE
    // sendMessage("&" + String(EVENT_PHOTOMETRY_OFF) + " " + String(signedMillis() - _exp_timer));
    _photometry_LED_enabled = false;
  }
} 
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Handles AnalogOutput -- When we get command to change power level change it 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void handleAnalogOutput(){
	if (_command == 'A')
	{
		setAnalogOutput(_arguments[0], _arguments[1]);
	}
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Set CHRIMSON/ CHR2 Analog Power
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setAnalogOutput(int channel, int value){
	switch (channel)
	{
	    case 1:
	    	analogWrite(PIN_CHRIMSON_PWR, value);
	    	break;
	    case 2:
	    	analogWrite(PIN_CHR2_PWR, value);
	    	break;
	}
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Set CHRIMSON LED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setChrimsonLED(bool turnOn)
{
  if (turnOn) {
    analogWrite(PIN_CHRIMSON_PWR, _params[CHRIMSON_AOUT_VALUE]);
    digitalWrite(PIN_CHRIMSON_STIM,HIGH);
    //sendEventMarker(EVENT_CHRIMSON_STIM_ON, -1);
  }
  else {
    analogWrite(PIN_CHRIMSON_PWR, 0);
    digitalWrite(PIN_CHRIMSON_STIM,LOW);
    //sendEventMarker(EVENT_CHRIMSON_STIM_OFF, -1);
  }
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Set CHR2 LED
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setChR2LED(bool turnOn){
  if (turnOn) {
    analogWrite(PIN_CHR2_PWR, _params[CHR2_AOUT_VALUE]);
    digitalWrite(PIN_CHR2_STIM, HIGH);
    //sendEventMarker(EVENT_CHR2_STIM_ON, -1);
  }
  else {
    analogWrite(PIN_CHR2_PWR, 0);
    digitalWrite(PIN_CHR2_STIM, LOW);
    //sendEventMarker(EVENT_CHR2_STIM_OFF, -1);
  }
}
/**********************************************************************************************************
  STIMULATION FUNCTIONS 
**********************************************************************************************************/

void runChR2Pulse() {
  // === Start pulse ===
  if (!inPulse && pulseIndex < _params[CHR2_NUM_PULSES] && (millis() - chR2ipiTimer >= _params[CHR2_IPI])) {
    setChR2LED(true);
    pulseTimer = millis();
    inPulse = true;
    if (_params[_DEBUG]) sendMessage("ChR2 Pulse " + String(pulseIndex + 1) + " ON");
  }
  
  // === End pulse ===
  if (inPulse && (millis() - pulseTimer >= _params[CHR2_PULSE_WIDTH])) {
    setChR2LED(false);
    inPulse = false;
    chR2ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("ChR2 Pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;
  }
}

void runChrimsonPulse() {
  // === Start pulse ===
  if (!inPulse && pulseIndex < _params[CHRIMSON_NUM_PULSES] && (millis() - ipiTimer >= (_params[CHRIMSON_IPI]))) {
    setChrimsonLED(true);
    pulseTimer = millis();
    inPulse = true;
    if (_params[_DEBUG]) sendMessage("Chrimson Pulse " + String(pulseIndex + 1) + " ON");
  }
  
  // === End pulse ===  
  if (inPulse && millis() - pulseTimer >= _params[CHRIMSON_PULSE_WIDTH]) {
    setChrimsonLED(false);
    inPulse = false;
    ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("Chrimson Pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;  // only increment after full pulse
  }
}

// === Function: runChR2ChrimsonPulses ===
void runChR2ChrimsonPulses() {
  static int chrimsonidx = 0;  // Analog output index for Chrimson and ChR2
  static int chr2idx = 0;       // Analog output index for ChR2
  
  if (chrimsonidx >= 6) chrimsonidx = 0; // Reset Chrimson index if it exceeds the number of aouts
  if (chr2idx >= 6) chr2idx = 0; // Reset ChR2 index if it exceeds the number of aouts

  // === Start pulse ===
  if (!inPulse && pulseIndex < _params[CHRIMSON_CHR2_NUM_PULSES] && (millis() - ipiTimer >= (_params[CHRIMSON_CHR2_IPI]))) {
    // Alternate between Chr2 and Chrimson

    if (dingdong) {
      setAnalogOutput(1, chrimsonLadder[chrimsonidx]);  // Chrimson
      chrimsonidx++;
      dingdong = false;
    } else {
      setAnalogOutput(2, chr2Ladder[chr2idx]);  // ChR2
      chr2idx++;
      dingdong = true;
    }
    pulseTimer = millis();
    inPulse = true;
    if (_params[_DEBUG]) sendMessage("Chrimson/ChR2 Pulse " + String(pulseIndex + 1) + " ON");
  }

  // === End pulse ===
  if (inPulse && millis() - pulseTimer >= _params[CHRIMSON_CHR2_PULSE_WIDTH]) {
    setChrimsonLED(false);
    setChR2LED(false);
    inPulse = false;
    ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("Chrimson/ChR2 Pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;  // only increment after full pulse
  }

  // === Train complete ===
  if (pulseIndex >= _params[CHRIMSON_CHR2_NUM_PULSES]) {
    pulseIndex = 0;
    currentAout = constrain(currentTrain, 0, 4);
    if (_params[_DEBUG]) {
      sendMessage("Completed Chrimson ChR2 pulses");
      sendMessage("Exiting Chrimson ChR2 Stim State.");
    }
    
     // When doing single Train i.e when in between trials or UI called we only send the receipt after the train is done and we're waiting for the next
    if (singleTrainMode) {
      if (_UI_CHRIMSON_stim_in_progress && !_BOSS_CHRIMSON_stim_in_progress) {
        _resultCode = CODE_UI_CHRIMSON;
      }
      else if (_BOSS_CHRIMSON_stim_in_progress && !_UI_CHRIMSON_stim_in_progress) {
        _resultCode = CODE_CHRIMSON;
      }
      else {
        // Ambiguous: both true or both false
        _resultCode = CODE_CHRIMSON;  // or CODE_UI_CHRIMSON
        sendMessage("WARNING: Ambiguous stim source — defaulting to CHRIMSON");
      }
      sendMessage("`" + String(_resultCode));
      _UI_CHRIMSON_stim_in_progress = false;
      _BOSS_CHRIMSON_stim_in_progress = false;
      _resultCode = -1;
      _state = WAIT_REQUEST_STATE;
      singleTrainMode = false;
    }
  }
}

// === Function: runChrimsonTrain ===
void runChrimsonTrain() {
  // === Start pulse ===
  if (!inPulse && pulseIndex < _params[CHRIMSON_NUM_PULSES] && (millis() - ipiTimer >= (_params[CHRIMSON_IPI]))) {
    setChrimsonLED(true);
    pulseTimer = millis();
    inPulse = true;
    if (_params[_DEBUG]) sendMessage("Chrimson Pulse " + String(pulseIndex + 1) + " ON");
  }

  // === End pulse ===
  if (inPulse && millis() - pulseTimer >= _params[CHRIMSON_PULSE_WIDTH]) {
    setChrimsonLED(false);
    inPulse = false;
    ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("Chrimson Pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;  // only increment after full pulse
  }

  // === Train complete ===
  if (pulseIndex >= _params[CHRIMSON_NUM_PULSES]) {
    pulseIndex = 0;
    currentTrain++;
    currentAout = constrain(currentTrain, 0, 4);
    if (_params[_DEBUG]) {
      sendMessage("Completed Chrimson Train " + String(currentTrain));
      sendMessage("Exiting Chrimson Stim State.");
    }
    
     // When doing single Train i.e when in between trials or UI called we only send the receipt after the train is done and we're waiting for the next

    if (singleTrainMode) {
      if (_UI_CHRIMSON_stim_in_progress && !_BOSS_CHRIMSON_stim_in_progress) {
        _resultCode = CODE_UI_CHRIMSON;
      }
      else if (_BOSS_CHRIMSON_stim_in_progress && !_UI_CHRIMSON_stim_in_progress) {
        _resultCode = CODE_CHRIMSON;
      }
      else {
        // Ambiguous: both true or both false
        _resultCode = CODE_CHRIMSON;  // or CODE_UI_CHRIMSON
        sendMessage("WARNING: Ambiguous stim source — defaulting to CHRIMSON");
      }
      sendMessage("`" + String(_resultCode));
      _UI_CHRIMSON_stim_in_progress = false;
      _BOSS_CHRIMSON_stim_in_progress = false;
      _resultCode = -1;
      _state = WAIT_REQUEST_STATE;
      singleTrainMode = false;
    }
  }
}

// === Function: runChR2Train ===
void runChR2Train() {
  // === Start pulse ===
  if (!inPulse && pulseIndex < _params[CHR2_NUM_PULSES] && (millis() - chR2ipiTimer >= _params[CHR2_IPI])) {
    setChR2LED(true);
    pulseTimer = millis();
    inPulse = true;
    if (_params[_DEBUG]) sendMessage("ChR2 Pulse " + String(pulseIndex + 1) + " ON");
  }

  // === End pulse ===
  if (inPulse && (millis() - pulseTimer >= _params[CHR2_PULSE_WIDTH])) {
    setChR2LED(false);
    inPulse = false;
    chR2ipiTimer = millis();
    if (_params[_DEBUG]) sendMessage("ChR2 Pulse " + String(pulseIndex + 1) + " OFF");
    pulseIndex++;
  }

  // === Train complete ===
  if (pulseIndex >= _params[CHR2_NUM_PULSES]) {
    pulseIndex = 0;
    currentTrain++;

    if (_params[_DEBUG]) sendMessage("Completed ChR2 Train " + String(currentTrain));

    if (singleTrainMode) {
      // === Disambiguate UI vs BOSS trigger
      if (_UI_CHR2_stim_in_progress && !_BOSS_CHR2_stim_in_progress) {
        _resultCode = CODE_UI_CHR2;
      }
      else if (_BOSS_CHR2_stim_in_progress && !_UI_CHR2_stim_in_progress) {
        _resultCode = CODE_CHR2;
      }
      else {
        _resultCode = CODE_CHR2;
        sendMessage("WARNING: Ambiguous ChR2 stim source — defaulting to CHR2");
      }

      sendMessage("`" + String(_resultCode));

      _UI_CHR2_stim_in_progress = false;
      _BOSS_CHR2_stim_in_progress = false;
      _resultCode = -1;
      _state = WAIT_REQUEST_STATE;
      singleTrainMode = false;
    }
  }
}


void stimulateChrimson() {
  currentTrain = 0;
  pulseIndex = 0;
  inPulse = false;
  currentAout = 0;
  singleTrainMode = true; // Flag that this is a one train manual stim
  if (_params[_DEBUG]) sendMessage("Starting CHRIMSON stimulation.");
}

void stimulateChR2() {
  currentTrain = 0;
  pulseIndex = 0;
  inPulse = false;
  singleTrainMode = true;  // <- NEW
  if (_params[_DEBUG]) sendMessage("Starting CHR2 stimulation.");
}


void resetTrain() {
  pulseIndex = 0;
  currentTrain = 0;
  currentAout = 0;
  inPulse = false;
  pulseTimer = millis();
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Set Chrimson Receipt (ON/OFF)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setChrimsonReceipt(bool turnOn) {
  if (turnOn) {
    digitalWrite(PIN_CHRIMSON_RECEIPT, HIGH);
  }
  else {
    digitalWrite(PIN_CHRIMSON_RECEIPT, LOW);
  }
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Set Chrimson Receipt (ON/OFF)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setChR2Receipt(bool turnOn) {
  if (turnOn) {
    digitalWrite(PIN_CHR2_RECEIPT, HIGH);
  }
  else {
    digitalWrite(PIN_CHR2_RECEIPT, LOW);
  }
}
/*****************************************************
  SERIAL COMMUNICATION TO HOST
*****************************************************/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  SEND MESSAGE to HOST
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void sendMessage(String message)   // Capital (String) because is defining message as an object of type String from arduino library
{
  Serial.println(message);
} // end Send Message---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  GET COMMAND FROM HOST (single character)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
char getCommand(String message)
{
  message.trim();                 // Remove leading and trailing white space
  return message[0];              // Parse message string for 1st character (the command)
} // end Get Command---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  GET ARGUMENTS (of the command) from HOST (2 int array)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void getArguments(String message, int *_arguments)  // * to initialize array of strings(?)
{
  _arguments[0] = 0;              // Init Arg 0 to 0 (reset)
  _arguments[1] = 0;              // Init Arg 1 to 0 (reset)

  message.trim();                 // Remove leading and trailing white space from MESSAGE

  //----Remove command (first character) from string:-----//
  String parameters = message;    // The remaining part of message is now "parameters"
  parameters.remove(0,1);         // Remove the command character and # (e.g., "P#")
  parameters.trim();              // Remove any spaces before next char

  //----Parse first (optional) integer argument-----------//
  String intString = "";          // init intString as a String object. intString concatenates the arguments as a string
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) 
  {                               // while the first argument in parameters has digits left in it unparced...
    intString += parameters[0];       // concatenate the next digit to intString
    parameters.remove(0,1);           // delete the added digit from "parameters"
  }
  _arguments[0] = intString.toInt();  // transform the intString into integer and assign to first argument (Arg 0)


  //----Parse second (optional) integer argument----------//
  parameters.trim();              // trim the space off of parameters
  intString = "";                 // reinitialize intString
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) 
  {                               // while the second argument in parameters has digits left in it unparced...
    intString += parameters[0];       // concatenate the next digit to intString
    parameters.remove(0,1);           // delete the added digit from "parameters"
  }
  _arguments[1] = intString.toInt();  // transform the intString into integer and assign to second argument (Arg 1)
} // end Get Arguments---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  INIT HOST (send States, Names/Value of Parameters to HOST)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void hostInit()
{
  //------Send state names and which states allow parameter update-------//
  for (int iState = 0; iState < _NUM_STATES; iState++)
  {// For each state, send "@ (number of state) (state name) (0/1 can update params)"
      sendMessage("@ " + String(iState) + " " + _stateNames[iState] + " " + String(_stateCanUpdateParams[iState]));
  }

  //-------Send event marker codes---------------------------------------//
  /* Note: "&" reserved for uploading new event marker and timestamp. "+" is reserved for initially sending event marker names */
  for (int iCode = 0; iCode < _NUM_OF_EVENT_MARKERS; iCode++)
  {// For each state, send "+ (number of event marker) (event marker name)"
      sendMessage("+ " + String(iCode) + " " + _eventMarkerNames[iCode]); // Matlab adds 1 to each code # to index from 1-n rather than 0-n
  }

  //-------Send param names and default values---------------------------//
  for (int iParam = 0; iParam < _NUM_PARAMS; iParam++)
  {// For each param, send "# (number of param) (param names) (param init value)"
      sendMessage("# " + String(iParam) + " " + _paramNames[iParam] + " " + String(_params[iParam]));
  }
  //--------Send result code interpretations.-----------------------------//
  for (int iCode = 0; iCode < _NUM_RESULT_CODES; iCode++)
  {// For each result code, send "* (number of result code) (result code name)"
      sendMessage("* " + String(iCode) + " " + _resultCodeNames[iCode]);
  }
  sendMessage("~");                           // Tells PC that Arduino is on (Send Message is a LF Function)
}

long signedMillis()
{
  double time = (double)(millis());
  return time;
}

/*****************************************************
  DEFINE TRANSITION-STATE REDUNDANCY FXS
*****************************************************/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Check for Quit Command
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
bool checkQuit() {
  if (_command == 'Q')  {                          // HOST: "QUIT" -> IDLE_STATE
    _state = IDLE_STATE;                             // Set IDLE_STATE
    return true;                                 // Exit Fx
  }
  else {return false;}
} // end checkQuit---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Check for Trial Start (lamp off)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void checkTrialStart() {
  if (_ready_for_next_trial && !digitalRead(PIN_TRIAL_INIT))  {
    _ready_for_next_trial = false;
    _ready_for_cue = true;
    sendMessage("&" + String(EVENT_TRIAL_INIT) + " " + String(signedMillis() - _exp_timer));
  }
  else if (!_ready_for_next_trial && digitalRead(PIN_TRIAL_INIT))  {
    _ready_for_next_trial = true;
  }
} // end checkTrialStart---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Check for Reference Event (cue)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void checkReferenceEvent() {
  if (_ready_for_cue && digitalRead(PIN_CUE_ON))  {
    sendMessage("&" + String(EVENT_CUE_ON) + " " + String(signedMillis() - _exp_timer));
    _ready_for_cue = false;
  }
} // end checkRefrence---------------------------------------------------------------------------------------------------------------------

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Check Warnings (is stim ongoing during param update?)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void checkWarnings() {
  if (_BOSS_CHRIMSON_stim_in_progress || _UI_CHRIMSON_stim_in_progress) {
    sendMessage("WARNING: Updated params during ongoing Chrimson stimulation");
    // Send event marker (abort) to HOST with timestamp
    sendMessage("&" + String(EVENT_CHRIMSON_ERROR) + " " + String(signedMillis() - _exp_timer));
  }
  // if (checkARCHTStim()) {
  //  sendMessage("WARNING: Updated params during ongoing ARCHT stimulation");
  //  // Send event marker (abort) to HOST with timestamp
  //  sendMessage("&" + String(EVENT_ARCHT_ERROR) + " " + String(signedMillis() - _exp_timer));
  // }
} // end checkQuit---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Check for Param Update
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
bool checkParamUpdate() {
  if (_command == 'P') {                          // Received new param from HOST: format "P _paramID _newValue" ('P' for Parameters)
  	// Make a copy of the arguments:
  	_argcopy[0] = _arguments[0];
  	_argcopy[1] = _arguments[1];

    _isParamsUpdateStarted = true;                  // Mark transmission start. Don't start next trial until we've finished.
    _isParamsUpdateDone = false;
    _params[_arguments[0]] = _arguments[1];         // Update parameter. Serial input "P 0 1000" changes the 1st parameter to 1000.
    _state = UPDATE_PARAMS_STATE;                            // Return -> ITI
    if (_params[_DEBUG]) {sendMessage("Parameter " + String(_arguments[0]) + " changed to " + String(_arguments[1]));} 
    return true;
  }
  else {return false;}
} // end checkParamUpdate---------------------------------------------------------------------------------------------------------------------


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Check for Param Update Complete
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
bool checkUpdateComplete() {
  if (_command == 'O') {                          // HOST transmission complete: HOST sends 'O' for Over.
    _isParamsUpdateStarted = false;
    _isParamsUpdateDone = true;                     // Mark transmission complete.
    _state = _prevState;                            // Return -> ITI
    _hold = false;
    if (_params[_DEBUG]) {sendMessage("Parameter update complete.");}
    return true;                                         // Exit Fx
  }
  else {return false;}
} // end checkParamUpdate---------------------------------------------------------------------------------------------------------------------


