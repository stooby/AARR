//-------Autonomous Audio-Reactive Robot: MotorBot-------
//project details, videos & photos online: http://scott-tooby.com/sonic-automata.html

//----PIN SETUP ---------
//Motor, Button, RotPots, Mic
const byte buttonPin = 2;
const byte motorPin = 3;
const byte pot1Pin = A0;
const byte pot2Pin = A1;
const byte pot3Pin = A2;
const byte micPin = A3;

//RGB LED pins
const byte redPin = 9;
const byte greenPin = 10;
const byte bluPin = 11;

//----VARIABLE INIT----------
//int pot1Val = 0; //not really using these at the moment....
//int pot2Val = 0; //not really using these at the moment....
//int pot3Val = 0; //not really using these at the moment....

//...MICROPHONE VARIABLES
//int micVal; //not currently using
const int sampleWindow = 50; // Sample window width in mS (50 mS = detecting down to 20Hz)
unsigned int sample;
unsigned int soundThresh = 330; //peakTopeak threshold level for detecting a "sound"

//...BUTTON VAR
boolean buttonState = LOW;
boolean lastButtonState = LOW;
/*volatile*/ byte buttonMode = 0; //use this for interrupt?
byte finalMode = 5; //the last (final) mode in the cycle of modes
unsigned long lastDebounceTime = 0;  // for monitoring the last time the button input pin state has changed
const byte debounceDelay = 50;    // debounce monitor timespan (time wait to confirm button state change) increase if input flickers / is not stable

//...MOTOR VAR
unsigned int motorVal = 0;
boolean motorActive = false;
//boolean motorValSet = false;

//...PERFORMANCE MODE VAR
unsigned long currentPulseActMillis = 0;
unsigned long prevPulseActMillis = 0; 
unsigned long prevSeqStopMillis = 0;
unsigned long newSeqStartMillis = 0;
unsigned long performanceStartMillis = 0; //for keeping track of extended performance periods and preventing continuous performance after set timespans
unsigned long idleStateStartTime = 0;

//time span threshold to denote continuity of a single "performance period" (in millis)
const unsigned int seqGapTimeThresh = 4000; //(millis) if a new pattern begins after a previous pattern terminates within this time span, the "performance period" is continuous, otherwise a new "performance period begins"
const unsigned int maxSilenceTimeThresh = 17; //time span threshold to trigger new pulse sequence after periods of no performance (in seconds)
const unsigned int maxPerformanceTimeLimit = 20; //max allowed continuous performance episode timespan (in sec)
const unsigned int idleStateMaxTimeLim = 8; //max allowed time duration of "Idle" state (in sec)

boolean haveHeardSound = false; //for keeping track of extended performance periods and preventing continuous performance after set timespans 
boolean perfModePulseAct = false;

boolean idleState = false;
boolean idleStateEntered = false;

boolean perfModeSusAct = false;
boolean motorSusAct = false;
boolean motorSusPerfEntered = false;
bool motorSusMaxDurReached = false;
bool motorSusMinDurReached = false;
const unsigned int motorSusMinDur = 500;
const unsigned int motorSusMaxDur = 2000;
int motorSusDur;

//NEW pulsePattern sequences
byte pulsePosition = 0;

const byte pulsePattern3length = 2; //number of matrix addresses for pulse pattern (i.e. total # of pulses minus 1)
int pulsePattern3dur[3] = {8, 9, 10};

const byte pulsePattern4length = 3;
int pulsePattern4dur[4] = {8, 8, 7, 6};

const byte pulsePattern5length = 4;
int pulsePattern5dur[5] = {7, 9, 10, 11, 12};

const byte pulsePattern6length = 5;
int pulsePattern6dur[6] = {6, 7, 8, 10, 12, 12};

const byte pulsePattern7length = 6;
int pulsePattern7dur[7] = {8, 9, 10, 11, 12, 8, 8};

const byte pulsePattern8length = 7; 
int pulsePattern8dur[8] = {5, 6, 7, 8, 9, 12, 12, 12};

const byte pulsePattern9length = 8;
int pulsePattern9dur[9] = {8, 8, 12, 12, 12, 11, 10, 9, 8};

const byte pulsePattern10length = 9;
int pulsePattern10dur[10] = {8, 8, 8, 12, 12, 8, 9, 10, 9, 8};

const byte pulsePattern11length = 10;
int pulsePattern11dur[11] = {8, 7, 6, 5, 6, 7, 8, 8, 8, 7, 6};

const byte pulsePattern12length = 11;
int pulsePattern12dur[12] = {8, 8, 12, 12, 12, 11, 10, 9, 8, 6, 2, 2};

//boolean pulsePatternActive = false;
boolean pulsePatternComplete = true;

int patternSeqVal;
int patternSeqValMax = 10;
int patternSeqValMin = 1;
bool patternSeqValMaxReached = false;
bool patternSeqValMinReached = false;
//*************

//...LED VAR
byte redVal = 0;
byte greenVal = 0;
byte bluVal = 0;
boolean LEDactive = false;
boolean LEDvalSet = false;
unsigned long prevMillisLED = 0;
unsigned long currentMillisLED = 0;
int idleStateEyeColor[3] = {250, 20, 0};
int black[3] = {0, 0, 0};
int idleStateTargetColor[3] = {idleStateEyeColor[0], idleStateEyeColor[1], idleStateEyeColor[2]};
int transColor[3] = {idleStateEyeColor[0], idleStateEyeColor[1], idleStateEyeColor[2]};
//int LEDval[3] = {redVal, greenVal, bluVal};

//...MISC VAR
unsigned int rate = 1000;



void setup()
{
  //Serial.begin(9600);  //for debugging
  pinMode(buttonPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
  pinMode(pot3Pin, INPUT);
  pinMode(micPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluPin, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), changeMode, FALLING);
  randomSeed(analogRead(A4)); //generate random seed value from random floating value of unconnected analog pin
  initRandPatternSeqVal(); //generate a random patternSeqVal
  initRandMotorSusDur(); //generate a random motorSusDur starting value
  stopMotor();
}

void loop()
{
  
  pollButton(); //check the button state to see if it's been pressed or not
  soundThresh = map(analogRead(pot1Pin), 0, 1023, 330, 700);
  rate = map(analogRead(pot2Pin), 0, 1023, 0, 4000);
  //motorVal = map(analogRead(pot3Pin), 10, 980, 0, 255);
  motorVal = map(analogRead(pot3Pin), 0, 1023, 0, 255);
  //Serial.println(perfModePulseAct);
  //Serial.println(pulsePatternComplete);

  switch (buttonMode)
  {
    case 0: //set LED color
    {
      if (LEDvalSet == false) //this conditional will prevent RGB values from being changed after rot pots are adjusted during "performance mode"
      {
        potToRGB();
        writeToLED(redVal, greenVal, bluVal);
      }
      else if ((analogRead(pot2Pin) == 0) && (analogRead(pot3Pin) == 0)) {LEDvalSet = false;} //can only reset RGB values by resetting all rotPots 2 and 3 to "0" position
      else {writeToLED(redVal, greenVal, bluVal);}
      /*
      if (perfModePulseAct == true || perfModeSusAct == true) //reset perfModePulseAct flag and pulsePattern in prep for the next time through the mode cycle - to prevent pattern when initializing performance mode
      {
        perfModePulseAct = false;
        perfModeSusAct = false;
        pulsePatternComplete = true;
        //idleState = false;
      } */
    }  
    break;

    case 1: //set motor amplitude
    {
      if (LEDvalSet == false || perfModePulseAct == true || perfModeSusAct == true) {resetModeStates();} //reset these these states if wrapping around through the modes multiple times to prevent bugs when returning to performance modes
      //motorVal = map(analogRead(pot3Pin), 0, 1023, 0, 255);

      writeToLED(map(motorVal, 0, 255, 0, redVal), map(motorVal, 0, 255, 0, greenVal), map(motorVal, 0, 255, 0, bluVal));
      activateMotor(motorVal);
      /* THINKING ABOUT IMPLEMENTING SAME FORM OF PROTECTION OF motorVal after RGBval alteration, but it makes code more verbose....I'll revisit this if time, but not totally necessary
      if (motorValSet == false)
      {
        motorVal = map(analogRead(pot3Pin), 0, 1023, 0, 255);
        //unsigned int motorValMeter = constrain((motorVal*1.2),0, 255);
        writeToLED(map(motorVal, 0, 255, 0, redVal), map(motorVal, 0, 255, 0, greenVal), map(motorVal, 0, 255, 0, bluVal));
        activateMotor(motorVal);
      //Serial.println(motorActive);
      }
      else if (analogRead(pot3Pin) < 3)
      {
        writeToLED(250,250,250);
        motorValSet = false;
      }
      else if (motorValSet == true)
      {
        unsigned int testMotorVal = map(analogRead(pot3Pin), 0, 1023, 0, 255);
        activateMotor(testMotorVal);
      }
      */
    }
    break;

    case 2: //set internal rate (and motor amplitude)
    {  
      currentMillisLED = millis(); //get the current time
      unsigned int timeElapsed = currentMillisLED - prevMillisLED; //get how much time has passed since we updated the animation

      if (timeElapsed > (rate/8))
      {
        cycleLED(redVal, greenVal, bluVal);
        pulseMotor();
        prevMillisLED = currentMillisLED;
      }
      //Serial.println(motorActive);
    }
    break;

    case 3: //set soundThresh
    {
      unsigned int testSoundLevel = monitorMic();
      if (testSoundLevel > soundThresh) {writeToLED(redVal, greenVal, bluVal);}
      else {writeToLED(0,0,0);}
    }
    break;

    case 4: //SOUND TRIGGER PERF MODE - PULSE PATTERNS
    {
      while (perfModePulseAct == false) //reset monitoring flags to init state when first switching to perf mode (in case this is second+ time through mode cycle)
      {
        //prevSeqStopMillis = millis(); //doing this so that a pattern isn't triggered immediately when this mode is activated (since prevSeqStopMillis = 0 upon initialization)
        //performanceStartMillis = millis(); //????is this necessary or is there a better way????
        //haveHeardSound = false;
        //idleStateEntered = false;
        //pulsePatternComplete = true;
        //pulsePosition = 0;
        //idleState = false;
        initPerfModeFlags();
        perfModePulseAct = true;
        delay(300); //extra button 'debounce' time to avoid button  sound from triggering pattern on init of perf mode
        break;
      }

      switch (idleState)
      {
        case false:
        {
          if (idleStateEntered == true) //if have just returned to performance state after being in idle state, update state & perfStart tracking time
          {
            idleStateEntered = false;
            writeToLED(0,0,0);
            //performanceStartMillis = millis(); //doing this to avoid triggering idle state after having returned from idle state (since it'll have been awhile since the performanceStartMillis value was reset)
            prevSeqStopMillis = millis(); //to avoid having pattern triggered immediately upon re-entering performance state due to perfPulseSeqIfQuietLong() function
          }
          else //else check continuous performance episode time span
          { 
            if ((motorActive == false) && ((millis() - prevSeqStopMillis) > seqGapTimeThresh))
            {
                //performanceStartMillis = millis(); //could do this, but I think below line is simpler to prevent false idle switches after priods of inactivity...
                haveHeardSound = false;
            }
            else if (((millis() - performanceStartMillis) > (maxPerformanceTimeLimit*1000)) && (haveHeardSound == true) && (motorActive == true)) //SWITCH to "idle mode" if have been engaged in a continuous performance period for more than a set time limit
            {
              haveHeardSound = false;
              idleState = true; //trigger idle state
              break;
            }
          }

          if (monitorMic() > soundThresh && pulsePatternComplete == true) //BEGIN new pulse pattern sequence if a "sound" is heard AND not in the middle of a pulse pattern sequence
          {performPulseSequence();
            //if (haveHeardSound == false) //if this is the first sound heard during performance mode
            //{
              //haveHeardSound = true;
            //performPulseSequence();
            //}
            //else {performPulseSequence();} //if this is NOT the first sound heard during performance mode
          } 
          else if (pulsePatternComplete == false) {performPulseSequence();} //CONTINUE pulse pattern sequence if not finished
          
          perfPulseSeqIfQuietLong(maxSilenceTimeThresh); //if haven't played a pattern in 25 seconds (haven't heard a sound in 25 seconds), make a sound
        }
        break;

        case true: //IDLE STATE ACTIVE
        {
          stopMotor();
          if (idleStateEntered == false) //if just entered idle state
          {
            idleStateStartTime = millis();
            idleStateEntered = true;
            pulsePatternComplete = true; // to prevent triggering pattern when exiting idle mode
            pulsePosition = 0;
          }
          else //if have been in idle state
          {
            if((millis() - idleStateStartTime) > (idleStateMaxTimeLim*1000))//if idle state has been active longer than max allowed time limit, switch back to non-idle state & commence performance
            {
              writeToLED(0,0,0);
              idleState = false; 
            }
            else //AutoFADE LED
            { 
              autoFlashLED(250,0,0);
              
              break;

                //DEBUG THIS AUTOFADE SHIIIITTTTTT IF TIMMEEEE
              currentMillisLED = millis();
              int thisTimeElapsed = currentMillisLED - prevMillisLED;
              if (thisTimeElapsed > (rate / 16))
              {
                if(transColor[0] == idleStateEyeColor[0] && transColor[1] == idleStateEyeColor[1] && transColor[2] == idleStateEyeColor[2])
                {
                  idleStateTargetColor[0] = 0;
                  idleStateTargetColor[1] = 0;
                  idleStateTargetColor[2] = 0;
                }
                else if (transColor[0] == 0 && transColor[1] == 0 && transColor[2] == 0)
                {
                  idleStateTargetColor[0] = idleStateEyeColor[0];
                  idleStateTargetColor[1] = idleStateEyeColor[1];
                  idleStateTargetColor[2] = idleStateEyeColor[2];
                }
                crossFade(transColor, idleStateTargetColor);
                prevMillisLED = currentMillisLED; //set the last time we advanced the state of the animation 
              } 
              
            } 
          }
        }
        break;
      }
      //WORKING PERF MODE CODE////////////////
      //Serial.println(perfModePulseAct);
      /*
      while (perfModePulseAct == false)
      {
        prevSeqStopMillis = millis(); //doing this so that a pattern isn't triggered immediately when this mode is activated (since prevSeqStopMillis = 0 upon initialization)
        perfModePulseAct = true;
        delay(75);
        break;
      }

      perfPulseSeqIfQuietLong(maxSilenceTimeThresh); //if haven't played a pattern in 25 seconds (haven't heard a sound in 25 seconds), make a sound

      if (monitorMic() > soundThresh && pulsePatternComplete == true) {performPulseSequence();} //if a "sound" is heard AND not in the middle of a pulse pattern sequence (idle), begin a new sequence
      else if (pulsePatternComplete == false) {performPulseSequence();} //if in the middle of a sequence, continue performing until it's finished
      //Serial.println(motorActive);
      */
      
    }
    break;

    case 5: //SOUND TRIGGER PERF MODE - SUSTAIN SEQUENCE
    {
      while (perfModeSusAct == false) //reset monitoring flags to init state when first switching to this perf mode (in case this is second+ time through mode cycle)
      {
        //prevSeqStopMillis = millis(); //doing this so that a pattern isn't triggered immediately when this mode is activated (since prevSeqStopMillis = 0 upon initialization)
        //performanceStartMillis = millis(); //????is this necessary or is there a better way????
        //haveHeardSound = false;
        //idleStateEntered = false;
        //pulsePatternComplete = true;
        //pulsePosition = 0;
        //motorSusAct = false;
        //idleState = false;
        initPerfModeFlags();
        stopMotor();
        perfModePulseAct = false;
        perfModeSusAct = true;
        delay(300); //extra button 'debounce' time to prevent button sound from triggering pattern on init of perf mode
        break;
      }
      /*
      Serial.print("perfModeSusAct= ");
      Serial.print(perfModeSusAct);
      Serial.print(" |haveHeardSound= ");
      Serial.print(haveHeardSound);
      Serial.print(" |motorSusAct= ");
      Serial.print(motorSusAct);
      Serial.print(" |motorSusPerfEnter= ");
      Serial.println(motorSusPerfEntered);
      Serial.print("idleState= ");
      Serial.print(idleState);
      Serial.print(" |idleStateEnter= ");
      Serial.println(idleStateEntered);
      */
      switch (idleState)
      {
        case false:
        {
          if (idleStateEntered == true) //if have just returned to performance state after being in idle state, update state & perfStart tracking time
          {
            idleStateEntered = false;
            prevSeqStopMillis = millis(); //to avoid having pattern triggered immediately upon re-entering performance state due to perfPulseSeqIfQuietLong() function
          }
          else //else check continuous performance episode time span
          { 
            if ((motorActive == false) && ((millis() - prevSeqStopMillis) > seqGapTimeThresh)) {haveHeardSound = false;}
            else if (((millis() - performanceStartMillis) > (maxPerformanceTimeLimit*1000)) && (haveHeardSound == true) && (motorActive == true)) //SWITCH to "idle mode" if have been engaged in a continuous performance period for more than a set time limit
            {
              haveHeardSound = false; //put this here or in idle case?
              idleState = true; //trigger idle state
              break;
            }
          }

          if (monitorMic() > soundThresh && motorSusAct == false) //BEGIN new sustain sequence if a "sound" is heard
          {
            perfMotorSusSeq();
          } 
          else if (motorSusAct == true) {perfMotorSusSeq();} //CONTINUE sustain sequence for remainder of its duration  
          
          perfSusSeqIfQuietLong(maxSilenceTimeThresh); //if haven't played a pattern in 25 seconds (haven't heard a sound in 25 seconds), make a sound
        }
        break;

        case true: //IDLE STATE ACTIVE
        {
          stopMotor();
          if (idleStateEntered == false) //if just entered idle state
          {
            idleStateStartTime = millis();
            idleStateEntered = true;
            motorSusAct = false; // to prevent triggering sustain seq when exiting idle mode
            motorSusPerfEntered = false;
          }
          else //if have been in idle state
          {
            if((millis() - idleStateStartTime) > (idleStateMaxTimeLim*1000))//if idle state has been active longer than max allowed time limit, switch back to non-idle state & commence performance
            {
              writeToLED(0,0,0);
              idleState = false; 
            }
            else {autoFlashLED(250, 0, 0);} //flash LED 
          }
        }
        break;
      }
    }
    break;  
  }  
}


//*******Functions******//

void initPerfModeFlags()
{
  //LEDvalSet = true;
  prevSeqStopMillis = millis(); 
  performanceStartMillis = millis();
  haveHeardSound = false;
  idleStateEntered = false;
  pulsePatternComplete = true;
  pulsePosition = 0;
  motorSusAct = false;
  motorSusPerfEntered = false;
  idleState = false;
  writeToLED(0, 0, 0);
  //perfModePulseAct = false;
  //perfModeSusAct = false;
}

void resetModeStates()
{
  LEDvalSet = true;
  perfModePulseAct = false;
  perfModeSusAct = false;
}

void pollButton() //polls button state w/ debounce functionality
{
  boolean reading = digitalRead(buttonPin); // read the state of the switch into a local variable
  if (reading != lastButtonState) // If the switch changed, due to noise or pressing
    {lastDebounceTime = millis();} // reset the debouncing timer

  if ((millis() - lastDebounceTime) > debounceDelay) //if reading has lasted longer than debounce time...
  { 
    if (reading != buttonState) // ..and if the button state has changed...
    {
      buttonState = reading;  //accept the button state reading
      if (buttonState == HIGH) //if the button has been depressed...
      {
        stopMotor(); //if the motor was active during the current mode, stop it when transitioning to the next mode
        changeMode(); //advance the bots "mode" by one state
      } 
    } 
  }
  lastButtonState = reading;
}

void changeMode()
{
  if (buttonMode == finalMode) {buttonMode = 0;} //if the bot is in its final mode, cycle back to the first mode
  else {buttonMode++;} //otherwise advance the mode by one
  //Serial.println(buttonMode);
}

void writeToLED(byte r, byte g, byte b)
{
  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluPin, b);
}

void potToRGB()
{
  redVal = map(analogRead(pot1Pin), 0, 1023, 0, 255);
  greenVal = map(analogRead(pot2Pin), 0, 1023, 0, 255);
  bluVal = map(analogRead(pot3Pin), 0, 1023, 0, 255);
}

void autoFlashLED(byte thisRed, byte thisGreen, byte thisBlue)
{
  currentMillisLED = millis(); //get the current time
  unsigned int timeElapsed = currentMillisLED - prevMillisLED; //get how much time has passed since we updated the animation

  if (timeElapsed > (rate/8))
  {
    cycleLED(thisRed, thisGreen, thisBlue);
    prevMillisLED = currentMillisLED;
  }
}

void cycleLED(byte thisRed, byte thisGreen, byte thisBlue)
{
  if (LEDactive == false) {writeToLED(thisRed, thisGreen, thisBlue);}
  else {writeToLED(0,0,0);}
  LEDactive = !LEDactive;
}

void motorLightLED()
{
  if (motorActive == true) {writeToLED(redVal, greenVal, bluVal);}
  else {writeToLED(0,0,0);}
}

void activateMotor(byte val)
{
  analogWrite(motorPin, val);
  motorActive = true;
}

void stopMotor()
{
  analogWrite(motorPin, 0);
  motorActive = false;
}

void pulseMotor()
{
  if (motorActive == false) {activateMotor(motorVal);}
  else {stopMotor();}
  //motorActive = !motorActive;
}

void playPulsePattern(int thisPatternArray[], int thisPatternLength)
{
  if (pulsePosition > thisPatternLength) //if we've gone past the last "note" of the pattern
  {
    //Serial.println(pulsePosition);
    pulsePatternComplete = true;
    pulsePosition = 0;
    prevSeqStopMillis = millis(); //set time marker of when this most recent pattern stopped playing (for long term memory)
    //Serial.println(pulsePosition);
  }

  else //for all other "notes" of the pattern (first through last)
  {
    pulsePatternComplete = false;
    currentPulseActMillis = millis();
    //pulsePatternActive = true;
    unsigned int pulseDuration = rate/thisPatternArray[pulsePosition]; // to calculate the note duration take rate (time increment) value and divide by pulse(note) type (e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.)
    unsigned int pauseBetweenPulses = pulseDuration * 1.30;  // to distinguish the pulses, set a minimum time between them. the pulse's duration + 30% seems to work well.   
    unsigned int timeElapsed = currentPulseActMillis - prevPulseActMillis;
    unsigned int maxPulseEventLength = pulseDuration + pauseBetweenPulses;

    if (timeElapsed <= pulseDuration)
    {
      //Serial.println(pulsePosition);
      //pulseActive = true; //don't think this is necessary b/c of motorActive boolean flag
      activateMotor(motorVal);       
    }  
    else if (timeElapsed > pulseDuration && timeElapsed <= maxPulseEventLength)
    {
      //pulseActive = false;//don't think this is necessary b/c of motorActive boolean flag
      stopMotor(); //silence for pause time 
    }    
    else
    {
      prevPulseActMillis = currentPulseActMillis;
      pulsePosition = pulsePosition + 1;
    }
  }  
}

void performPulseSequence() //NEW VERSION
{
    if (pulsePatternComplete == true) //if beginning a new pattern sequence
    {
      randPatternSeqVal(); //adjust the patternSeqVal by a random amount (adding or subtracting to it)
      //Serial.println(patternSeqVal);
      newSeqStartMillis = millis(); //log time that this new pattern sequence has begun for long term memory

      if (haveHeardSound == false)
      {
        performanceStartMillis = newSeqStartMillis;
        haveHeardSound = true;
      }
      else 
      { //if the time between patterns exceeds threshold, a new "performance period" has begun
        if ((newSeqStartMillis - prevSeqStopMillis) > seqGapTimeThresh) {performanceStartMillis = newSeqStartMillis;}
      }

      if (patternSeqVal == 1) //based on the randomly summed patternSeqVal, select a pulsePattern for playback
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern3dur, pulsePattern3length);
        motorLightLED();
      }
      else if (patternSeqVal == 2)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern4dur, pulsePattern4length);
        motorLightLED();
      }
      else if (patternSeqVal == 3)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern5dur, pulsePattern5length);
        motorLightLED();
      }
      else if (patternSeqVal == 4)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern6dur, pulsePattern6length);
        motorLightLED();
      }
      else if (patternSeqVal == 5)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern7dur, pulsePattern7length);
        motorLightLED();
      }
      else if (patternSeqVal == 6)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern8dur, pulsePattern8length);
        motorLightLED();
      }
      else if (patternSeqVal == 7)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern9dur, pulsePattern9length);
        motorLightLED();
      }
      else if (patternSeqVal == 8)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern10dur, pulsePattern10length);
        motorLightLED();
      }
      else if (patternSeqVal == 9)
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern11dur, pulsePattern11length);
        motorLightLED();
      }
      else 
      {
        prevPulseActMillis = millis();
        playPulsePattern(pulsePattern12dur, pulsePattern12length);
        motorLightLED();
      }   
    }
    else if (pulsePatternComplete == false) //if continuing an on-going pattern sequence
    {
      if (patternSeqVal == 1) //based on the randomly summed patternSeqVal, select a pulsePattern for playback
      {
        playPulsePattern(pulsePattern3dur, pulsePattern3length);
        motorLightLED();
      }
      else if (patternSeqVal == 2)
      {
        playPulsePattern(pulsePattern4dur, pulsePattern4length);
        motorLightLED();
      }
      else if (patternSeqVal == 3)
      {
        playPulsePattern(pulsePattern5dur, pulsePattern5length);
        motorLightLED();
      }
      else if (patternSeqVal == 4)
      {
        playPulsePattern(pulsePattern6dur, pulsePattern6length);
        motorLightLED();
      }
      else if (patternSeqVal == 5)
      {
        playPulsePattern(pulsePattern7dur, pulsePattern7length);
        motorLightLED();
      }
      else if (patternSeqVal == 6)
      {
        playPulsePattern(pulsePattern8dur, pulsePattern8length);
        motorLightLED();
      }
      else if (patternSeqVal == 7)
      {
        playPulsePattern(pulsePattern9dur, pulsePattern9length);
        motorLightLED();
      }
      else if (patternSeqVal == 8)
      {
        playPulsePattern(pulsePattern10dur, pulsePattern10length);
        motorLightLED();
      }
      else if (patternSeqVal == 9)
      {
        playPulsePattern(pulsePattern11dur, pulsePattern11length);
        motorLightLED();
      }
      else 
      {
        playPulsePattern(pulsePattern12dur, pulsePattern12length);
        motorLightLED();
      }
    }
}

void perfMotorSusSeq()
{
  if (motorSusAct == false)
  {
    motorSusAct = true;
    randMotorSusDur();
    //Serial.print("randMotorSusDur = ");
    //Serial.println(motorSusDur);
    motorSustain(motorSusDur);
  }
  else {motorSustain(motorSusDur);}
}

void motorSustain(int duration)
{
  if (motorSusPerfEntered == false) //when first entering a motorSus performance sequence: log time and assess prev perf history)
  { 
    motorSusPerfEntered = true; 
    newSeqStartMillis = millis();  //log time 

    if (haveHeardSound == false) //assess prev seq performance history
    {
      performanceStartMillis = newSeqStartMillis;
      haveHeardSound = true;
    }
    else 
    { //if the time between patterns exceeds threshold, a new "performance period" has begun
      if ((newSeqStartMillis - prevSeqStopMillis) > seqGapTimeThresh) {performanceStartMillis = newSeqStartMillis;}
    }
    activateMotor(motorVal);
    //autoFlashLED(redVal, greenVal, bluVal);
  }

  else
  { //proceed with motor performance after it's started
    unsigned int timeElapsed = millis() - newSeqStartMillis;
    if (timeElapsed <= duration)
    {
      activateMotor(motorVal);
      autoFlashLED(redVal, greenVal, bluVal);
    }
    else
    {
      stopMotor();
      writeToLED(0, 0, 0);
      prevSeqStopMillis = millis();
      motorSusAct = false;
      motorSusPerfEntered = false;
      delay(500);//a slight delay to allow the motor to come to rest before re-entering loop and monitoring (prevents internal feedback)
      //haveHeardSound = false; //????????
      //servoActivated = false; //could cut this? and incorporate into loop w/ condition that resets this after function terminates to permit memory of having been performing servoSustain(seq)
      //triggerServo = false;
      //isSilent = false; //to avoid resetting servoSustain immediately after terminating function and resuming larger loop
    }
  }
}


void perfPulseSeqIfQuietLong(int timeSpanInSeconds)
{
  if (millis() - prevSeqStopMillis > (timeSpanInSeconds*1000)) {performPulseSequence();}
}

void perfSusSeqIfQuietLong(int timeSpanInSeconds)
{
  if (millis() - prevSeqStopMillis > (timeSpanInSeconds*1000)) {perfMotorSusSeq();}
}

unsigned int monitorMic()
{
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
 
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
 
      // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(micPin);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax) {signalMax = sample;} // save just the max levels
      else if (sample < signalMin) {signalMin = sample;}  // save just the min levels
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  return peakToPeak;
  //double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
}

void initRandMotorSusDur()
{
  //int thisRandVal = random(1, ((motorSusMaxDur/1000)+1));
  //motorSusDur = thisRandVal*1000;
  motorSusDur = random(motorSusMinDur, (motorSusMaxDur+1));
}

void randMotorSusDur()
{
  if (motorSusMaxDurReached == true)
  {
    motorSusMaxDurReached = false;
    motorSusMinDurReached = true;
    motorSusDur = motorSusMinDur;
  }

  else if (motorSusMinDurReached == true)
  {
    motorSusMinDurReached = false;
    int thisRandVal = random (1,4);
    if (thisRandVal == 1) {motorSusDur = motorSusDur + motorSusMinDur;}
    else if (thisRandVal == 2) {motorSusDur = motorSusDur + (.375 * motorSusMaxDur);}
    else {motorSusDur = motorSusDur + (.5 * motorSusMaxDur);}
  }

  else
  {
    int thisRandVal = random(1, 11);
    if (thisRandVal == 1) {motorSusDur = motorSusDur - (.5 * motorSusMaxDur);}
    else if (thisRandVal == 2 || thisRandVal == 3) {motorSusDur = motorSusDur - (.25 * motorSusMaxDur);}
    else if (thisRandVal >= 7 && thisRandVal <= 9) {motorSusDur = motorSusDur + (.375 * motorSusMaxDur);}
    else if (thisRandVal == 10) {motorSusDur = motorSusDur + (.5 * motorSusMaxDur);}

    if (motorSusDur >= motorSusMaxDur)
    {
      motorSusDur = motorSusMaxDur;
      motorSusMaxDurReached = true;
    }
    else if (motorSusDur <= motorSusMinDur)
    {
      motorSusDur = motorSusMinDur;
      motorSusMinDurReached = true;
    }
  }
}

void initRandPatternSeqVal()
{
  //int thisRandVal = random(0, ((patternSeqValMax/10)+1));
  patternSeqVal = random(patternSeqValMin, (patternSeqValMax+1));
}

void randPatternSeqVal()
{
  if (patternSeqValMaxReached == true)
  {
    patternSeqValMaxReached = false;
    patternSeqValMinReached = true;
    patternSeqVal = patternSeqValMin;
  }

  else if (patternSeqValMinReached == true)
  {
    patternSeqValMinReached = false;
    int thisRandVal = random (1,4);
    if (thisRandVal == 1) {patternSeqVal = patternSeqVal + 1;}
    else if (thisRandVal == 2) {patternSeqVal = patternSeqVal + 2;}
    else {patternSeqVal = patternSeqVal + 3;}
  }

  else
  {
    int thisRandVal = random(1, 11);
    if (thisRandVal == 1) {patternSeqVal = patternSeqVal - 3;}
    else if (thisRandVal == 2 || thisRandVal == 3) {patternSeqVal = patternSeqVal - 1;}
    else if (thisRandVal == 6 || thisRandVal == 7) {patternSeqVal = patternSeqVal + 1;}
    else if (thisRandVal == 8 || thisRandVal == 9) {patternSeqVal = patternSeqVal + 2;}
    else if (thisRandVal == 10) {patternSeqVal = patternSeqVal + 4;}

    if (patternSeqVal >= patternSeqValMax)
    {
      patternSeqVal = patternSeqValMax;
      patternSeqValMaxReached = true;
    }
    else if (patternSeqVal <= patternSeqValMin)
    {
      patternSeqVal = patternSeqValMin;
      patternSeqValMinReached = true;
    }
  }
}

int calculateStep(int prevValue, int endValue)
{
  int stepSize = endValue - prevValue;
  return stepSize;
}

int calculateVal(int stepSize, int val)
{
  if (stepSize) // If step is non-zero
  { 
    if (stepSize > 0) {val += 1;}  //increment the value if step is positive...             
    else if (stepSize < 0) {val -= 1;}  //...or decrement it if step is negative           
  }
  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {val = 255;} 
  else if (val < 0) {val = 0;}
  return val;
}

void crossFade(int color1[3], int color2[3])
{
  //writeToLED....SHOW color1 first!
  writeToLED(color1[0], color1[1], color1[2]);
  
  //then calculate distance between color1 and color2  
  int stepR = calculateStep(color1[0], color2[0]);
  int stepG = calculateStep(color1[1], color2[1]); 
  int stepB = calculateStep(color1[2], color2[2]);
  
  //then increment color1 RGB values '1' closer to color2 values
  transColor[0] = calculateVal(stepR, color1[0]);
  transColor[1] = calculateVal(stepG, color1[1]);
  transColor[2] = calculateVal(stepB, color1[2]);
}

//current_color is transColor
bool fadeTo(int current_color[3], int target_color[3])
{  
  if( current_color[0] == target_color[0] &&
  current_color[1] == target_color[1] &&
  current_color[2] == target_color[2])
    return true;
  
  crossFade(current_color, target_color);
  return false;
}
