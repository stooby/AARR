//-------Autonomous Audio-Reactive Robot: ServoBot-------
//project details, videos & photos online: http://scott-tooby.com/sonic-automata.html

#include <Servo.h> 
Servo servo1;  // create servo object to control a servo 

//----PIN SETUP ---------
const byte buttonPin = 2;
const byte servoPin = 9;
const byte pot1Pin = A0;
const byte pot2Pin = A1;
const byte pot3Pin = A2;
const byte micPin = A3;
//const byte servoGroundPin = 12;

//RGB LED pins
const byte redPin = 3;
const byte greenPin = 5;
const byte bluPin = 11;


//----VARIABLE INIT-------
//...BUTTON VAR
boolean buttonState = LOW;
boolean lastButtonState = LOW;
/*volatile*/ byte buttonMode = 0; //use volatile if using as interrupt
byte finalMode = 5; //the last (final) mode in the cycle of modes
unsigned long lastDebounceTime = 0;  // for monitoring the last time the button input pin state has changed
const byte debounceDelay = 50;    // debounce monitor timespan (time wait to confirm button state change) increase if input flickers / is not stable


//...LED VAR
byte redVal = 0;
byte greenVal = 0;
byte bluVal = 0;
boolean LEDvalSet = false;
boolean LEDactive = false;
unsigned long prevMillisLED = 0;
unsigned long currentMillisLED;

int eyeColorHeardSound[3] = {250, 20, 0};
int eyeColorListening[3] = {240,0,50};
int eyeColorServoMvmt[3] = {0, 250, 40};

//...MICROPHONE/SOUND VAR
//unsigned long previousSoundMillis = 0; //store last sound event update - UNUSED
const int sampleWindow = 50; // Sample window width in mS (50 mS = detecting down to 20Hz)
unsigned int sample;
unsigned int soundThresh = 330; //peakTopeak threshold level for detecting a "sound"
unsigned int silenceMonitorDur = 7000; //(millis) time duration to listen after period of silence begins (and also between periods of soundHeard...)
bool silenceMonitorDurSet = false;
unsigned long prevMillisSilenceMonDurSet = 0;
unsigned int soundPeriodStartMonitorDur = 3000; //(millis) time duration to dilineate new periods of sound activity...
unsigned long silenceStartMillis = 0;
unsigned int soundHeardDelayDur = 700; //(millis) time duration of cessation of sound monitoring after hearing sound (causes LED denoting heardSound to stay illuminated for a more noticeable duration)
unsigned long newSoundStartTime = 0;
unsigned long prevSoundStopTime = 0;
unsigned long soundPeriodStartTime;
unsigned int micVal;
bool isSilent = false;
bool haveHeardSound = false;

//...SERVO VAR
unsigned long prevMillisServo = 0;
unsigned long currentMillis = 0;

unsigned long servoSeqStartMillis = 0;
unsigned long servoSeqStopMillis = 0;
//const unsigned int interval = 100;

int servoMinDegrees = 0; // minimum degree limit of servo
//const unsigned int servoMaxStaticDegrees = 40; //not currently being used
int servoMaxDegrees = 40; //transient var for servoMax
int servoIdleMinDegrees = 100;
int servoIdleMaxDegrees = 140;
int servoIdleSpeedInterval = 30; //millis
//const unsigned int servoMaxUprLim = 60;
//const unsigned int servoMaxLwrLim = 20;
int servoPosition = 40;     // the current angle of the servo - starting at MaxDegreelimit.
int prevServoPosition = servoPosition;
int servoDegrees = 2; // amount servo moves at each step

//unsigned int servoSlowInterval = 80; // millisecs between servo moves - ???INTEGRATE INTO RANDOM GYRATINONS???
unsigned int servoFastInterval = 3;
unsigned int servoInterval = servoFastInterval; // initial millisecs between servo moves -- equivalent to 'rate' variable of Motor bot sketches
bool servoActivated = false;
bool servoMvmtUp = false;
//bool servoPerfEnd = false;
bool triggerServo = false;
//bool servoMaxWild = false;
//bool servoMinReached = false;
//byte servoMaxFlux = 10;

bool servoSeqMaxDurReached = false;
bool servoSeqMinDurReached = false;

const unsigned int servoSeqMinDur = 1200;
const unsigned int servoSeqMaxDur = 5500;
//int randServoDurSelector;
//int thisRandVal; //for debugging
int servoSeqDur;

//...PERFORMANCE MODE VAR
//unsigned long currentPulseActMillis = 0;
//unsigned long prevPulseActMillis = 0; 
//unsigned long prevPatternStopMillis = 0;
//unsigned long newPatternStartMillis = 0;
unsigned long performanceStartMillis = 0; //for keeping track of extended performance periods and preventing continuous performance after set timespans
unsigned long idleStateStartTime = 0;

//time span threshold to denote continuity of a single "performance period" (in millis)
const unsigned int patternGapTimeThresh = 4000; //(millis) if a new pattern begins after a previous pattern terminates within this time span, the "performance period" is continuous, otherwise a new "performance period begins"
//const unsigned int soundGapTimeThresh = 3000; //(millis)
const unsigned int maxSoundTimeThresh = 8; //(sec) max time span threshold to trigger idleServo movements during extended periods of sound
const unsigned int maxSilenceTimeThresh = 17; //time span threshold to trigger new pulse sequence after periods of no performance (in seconds)
const unsigned int maxPerformanceTimeLimit = 20; //max allowed continuous performance episode timespan (in sec)
const unsigned int idleStateMaxTimeLim = 8; //max allowed time duration of "Idle" state (in sec)

boolean perfMode1Act = false;
//boolean haveHeardSound = false; //for keeping track of extended performance periods and preventing continuous performance after set timespans 
boolean idleState = false;
boolean idleStateEntered = false;


//...MISC VAR
//int randVal;
unsigned int pot1PinVal;

/*
void initPerfModeFlags()
{
  //LEDvalSet = true;
  servoSeqStopMillis = millis();
  soundPeriodStartTime = millis();
  //prevSeqStopMillis = millis(); 
  //performanceStartMillis = millis();
  haveHeardSound = false;
  idleStateEntered = false;
  //pulsePatternComplete = true;
  //pulsePosition = 0;
  //motorSusAct = false;
  //motorSusPerfEntered = false;
  idleState = false;
  //writeToLED(0, 0, 0);
  //perfModePulseAct = false;
  //perfModeSusAct = false;
}*/

//////////////////////////
void setup() 
{ 
  //Serial.begin(9600);

  pinMode(buttonPin, INPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
  pinMode(pot3Pin, INPUT);
  pinMode(micPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluPin, OUTPUT);
  //pinMode(servoGroundPin, OUTPUT);
  //digitalWrite(servoGroundPin, LOW);
  servo1.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  randomSeed(analogRead(A4)); //generate random seed value from random floating value of unconnected analog pin

  initRandServoSeqDur();
  writeToLED(0,0,0);
} 
 
void loop() 
{ 
  currentMillis = millis();
  pollButton(); //check the button state to see if it's been pressed or not
  soundThresh = map(analogRead(pot1Pin), 0, 1023, 300, 900);
  servoInterval = map(analogRead(pot2Pin), 0, 1023, 2, 20); //a.k.a. 'rate' in motor bot sketches
  servoMaxDegrees = map(analogRead(pot3Pin), 0, 900, 0, 180);

  /*
  if (silenceMonitorDurSet == true)
  {
    Serial.print("silenceMonDurIsSet > ");
    Serial.println(silenceMonitorDur);
  }
  */
/* //servo debug....
  Serial.print("servoInterval = ");
  Serial.print(servoInterval);
  Serial.print(" | servoMaxDeg = ");
  Serial.println(servoMaxDegrees);
  Serial.print("servoPosition = ");
  Serial.print(servoPosition);
  Serial.print(" | servoMvmtUp = ");
  Serial.println(servoMvmtUp);
  Serial.println("----");
*/
  //soundThresh = 1000; //for debugging
  //debugMicTest1(); 

  switch(buttonMode)
  {
    case 0: //CONFIGURE LED MODE
    {
      if (LEDvalSet == false) //this conditional will prevent RGB values from being changed after rot pots are adjusted during "performance mode"
      {
        potToRGB();
        writeToLED(redVal, greenVal, bluVal);
        eyeColorServoMvmt[0] = redVal;
        eyeColorServoMvmt[1] = greenVal;
        eyeColorServoMvmt[2] = bluVal;
      }

      //SET OTHER EYE COLORS HERE TOO!!!!!!?????????

      else if ((analogRead(pot2Pin) == 0) && (analogRead(pot3Pin) == 0)) {LEDvalSet = false;} //can only reset RGB values by resetting all rotPots 2 and 3 to "0" position
      else {writeToLED(eyeColorServoMvmt[0], eyeColorServoMvmt[1], eyeColorServoMvmt[2]);}

      if (perfMode1Act == true) //(imported from MotorBot code...) __ reset perfMode1Act flag and pulsePattern in prep for the next time through the mode cycle - to prevent pattern when initializing performance mode
      {
        perfMode1Act = false;
      }
    }
    break;

    case 1: //SET MAX SERVO DEGREE MODE
    {
      if (LEDvalSet == false) {LEDvalSet = true;}
      //servoMaxDegrees = map(analogRead(pot3Pin), 0, 900, 0, 180);
      servoPosition = servoMaxDegrees;
      servo1.write(servoMaxDegrees);
    }
    break;

    case 2: //TEST SERVO ACTION MODE (ADJUST RATE? && MAX SERVO DEGREE)
    {
      //servoPosition = servoMaxDegrees;
      servoSweep();
      servoLightLED();
    }
    break;

    case 3: //ADJUST SILENCE MONITOR PERIOD MODE
    {
      if (silenceMonitorDurSet == false)
      {
        silenceMonitorDur = map(analogRead(pot1Pin), 0, 1023, 3000, 12000); //pot1pin sets silenceMonitorDur
        if (currentMillis - prevMillisSilenceMonDurSet >= silenceMonitorDur) //turn LED on at the start of the monitor duration period
        {
          writeToLED(240,240,240); 
          prevMillisSilenceMonDurSet = currentMillis;
        }
        if (currentMillis - prevMillisSilenceMonDurSet >= 250) {writeToLED(0,0,0);} //turn LED off 250 ms later (effectively there's a cycling LED pulse indicating the rate/duration of monitoring cycles)
        
        //Serial.print("silenceMonDurNotSet > ");
        //Serial.println(silenceMonitorDur);
      }
      else if (analogRead(pot1Pin) == 0) {silenceMonitorDurSet = false;}
      else
      {
        if (currentMillis - prevMillisSilenceMonDurSet >= silenceMonitorDur) //same code as above (pulse LED at the rate of each monitoring duration cycle)
        {
          writeToLED(250,250,250);
          prevMillisSilenceMonDurSet = currentMillis;
        }
        if (currentMillis - prevMillisSilenceMonDurSet >= 250) {writeToLED(0,0,0);}
      }
      //Serial.print("silenceMonDur= ");
      
    }
    break;

    case 4: //TEST MIC SENSITIVITY THRESHOLD MODE
    {
      if (silenceMonitorDurSet == false) {silenceMonitorDurSet = true;}

      servoSeqStop();
      unsigned int testSoundLevel = monitorMic();
      //Serial.println(testSoundLevel);
      if (testSoundLevel > soundThresh) {writeToLED(eyeColorHeardSound[0], eyeColorHeardSound[1], eyeColorHeardSound[2]);}
      else {writeToLED(0,0,0);}
    }
    break;

    case 5: //PERFORMANCE MODE (1)
    {
      while (perfMode1Act == false)
      {
        perfMode1Act = true;
        idleState = false;
        soundPeriodStartTime = millis();
      }

      if (triggerServo == false)
      {
          if (monitorMic() > soundThresh)
          {
            isSilent = false;
            autoFlashLEDServo(eyeColorHeardSound[0],eyeColorHeardSound[1],eyeColorHeardSound[2], 20);
            //newSoundStartTime = millis();
            //writeToLED(eyeColorHeardSound[0],eyeColorHeardSound[1],eyeColorHeardSound[2]);
            if (haveHeardSound == false)
            {
              haveHeardSound = true;
              newSoundStartTime = millis();
            }
            if ((newSoundStartTime - prevSoundStopTime) > soundPeriodStartMonitorDur) {soundPeriodStartTime = millis();}
          }
          else if(haveHeardSound == true) //this condition causes LED to stay illuminated for a specified duration (1500ms) after having heard a sound, so it's more noticeable
          {
            autoFlashLEDServo(eyeColorHeardSound[0],eyeColorHeardSound[1],eyeColorHeardSound[2], 20);
            if ((millis() - newSoundStartTime) > soundHeardDelayDur)
            {
              //writeToLED(eyeColorListening[0],eyeColorListening[1],eyeColorListening[2]);
              prevSoundStopTime = millis();
              haveHeardSound = false;
            }
            if ((millis() - soundPeriodStartTime) > (maxSoundTimeThresh * 1000))
            {
              //servoSweepIdle(servoIdleMinDegrees, servoIdleMaxDegrees, servoIdleSpeedInterval);
              idleStateStartTime = millis();
              triggerServo = true;
              idleState = true;
            }
          }
          else if (monitorMic() < soundThresh && isSilent == false && haveHeardSound == false) //log silence start time (listening for continued silence/disturbance)
          {
            silenceStartMillis = millis();
            isSilent = true;
            writeToLED(eyeColorListening[0],eyeColorListening[1],eyeColorListening[2]);
          }
          else if (monitorMic() < soundThresh && isSilent == true)
          {
            unsigned int thisSilenceTimeElapsed = millis() - silenceStartMillis; //keep track of how long it's been silent
            if (thisSilenceTimeElapsed > silenceMonitorDur) //if it's been silent for longer than a pre-determined duration
            { //then transition to servo performance mode
              writeToLED(eyeColorServoMvmt[0],eyeColorServoMvmt[1],eyeColorServoMvmt[2]);
              randServoSeqDur(); //assigns a randomly selected time value to 'servoSeqDur'
              triggerServo = true;
              //Serial.println(servoSeqDur);
            }
          }
      }
      else
      {
        if (idleState == false)
        {
          servoSustain(servoSeqDur);
          servoLightLED();
        }
        else
        {
          autoFlashLEDServo(eyeColorHeardSound[0],eyeColorHeardSound[1],eyeColorHeardSound[2], 20);
          servoSweepIdle(servoIdleMinDegrees, servoIdleMaxDegrees, servoIdleSpeedInterval);
          if ((millis() - idleStateStartTime) > (idleStateMaxTimeLim * 1000))
          {
            if (monitorMic() > soundThresh) {idleStateStartTime = millis();}
            else
            {
              idleState = false;
              triggerServo = false;
              haveHeardSound = false;
              soundPeriodStartTime = millis();
            }
          }
        } 
      }
                                    
    }
    break; //break from case 4
  }
} 

//*******Functions******//

void initRandServoSeqDur()
{
  //int thisRandVal = random(1, ((servoSeqMaxDur/1000)+1));
  //servoSeqDur = thisRandVal*1000;
  servoSeqDur = random(servoSeqMinDur, (servoSeqMaxDur + 1));
}

void randServoSeqDur()
{
  if (servoSeqMaxDurReached == true)
  {
    servoSeqMaxDurReached = false;
    servoSeqMinDurReached = true;
    servoSeqDur = servoSeqMinDur;
    //return servoSeqDur;
  }

  else if (servoSeqMinDurReached == true)
  {
    servoSeqMinDurReached = false;
    int thisRandVal = random (1,4);
    if (thisRandVal == 1) {servoSeqDur = servoSeqDur + (.1 * servoSeqMaxDur);}
    else if (thisRandVal == 2) {servoSeqDur = servoSeqDur + (.2 * servoSeqMaxDur);}
    else {servoSeqDur = servoSeqDur + (.3 * servoSeqMaxDur);}
    //return servoSeqDur;
  }

  else
  {
    int thisRandVal = random(1, 11);
    if (thisRandVal == 1) {servoSeqDur = servoSeqDur - (.5 * servoSeqMaxDur);}
    else if (thisRandVal == 2 || thisRandVal == 3) {servoSeqDur = servoSeqDur - (.14 * servoSeqMaxDur);}
    else if (thisRandVal >= 7 && thisRandVal <= 9) {servoSeqDur = servoSeqDur + (.15 * servoSeqMaxDur);}
    else if (thisRandVal == 10) {servoSeqDur = servoSeqDur + (.5 * servoSeqMaxDur);}

    if (servoSeqDur >= servoSeqMaxDur)
    {
      servoSeqDur = servoSeqMaxDur;
      servoSeqMaxDurReached = true;
    }
    else if (servoSeqDur <= servoSeqMinDur)
    {
      servoSeqDur = servoSeqMinDur;
      servoSeqMinDurReached = true;
    }
    //return servoSeqDur;
  }
}

void servoSustain(int duration)
{
  if (servoActivated == false)
  { //begin servo performance
    servoActivated = true; 
    servoSeqStartMillis = millis();
    servoSweep();
  }
  else
  { //proceed with servo performance after it's started
    unsigned int timeElapsed = millis() - servoSeqStartMillis;
    if (timeElapsed < duration) {servoSweep();}
    else
    {
      servoSeqStop();
      //servo1.write(servoMaxStaticDegrees);
      //servoActivated = false; //could cut this? and incorporate into loop w/ condition that resets this after function terminates to permit memory of having been performing servoSustain(seq)
      //triggerServo = false;
      //isSilent = false; //to avoid resetting servoSustain immediately after terminating function and resuming larger loop
      //servoSeqStopMillis = millis();
    }
  }
}

void servoSeqStop()
{
  servo1.write(servoMaxDegrees);
  servoActivated = false; //could cut this? and incorporate into loop w/ condition that resets this after function terminates to permit memory of having been performing servoSustain(seq)
  triggerServo = false;
  isSilent = false; //to avoid resetting servoSustain immediately after terminating function and resuming larger loop
  servoSeqStopMillis = millis();
}

void servoLightLED()
{
  //if (servoMvmtUp == false) {writeToLED(redVal, greenVal, bluVal);}
  if (servoMvmtUp == false) {writeToLED(eyeColorServoMvmt[0], eyeColorServoMvmt[1], eyeColorServoMvmt[2]);}
  else {writeToLED(0,0,0);}
}

/* ORIG servoSustain (works)
void servoSustain(int duration)
{
  unsigned int timeElapsed = millis() - servoSeqStartMillis;
  if (timeElapsed < duration) {servoSweep();}
  else
  {
    servo1.write(servoMaxStaticDegrees);
    servoActivated = false;
    triggerServo = false;
    //servoPerfEnd = true;
  }
}
*/

void servoSweepIdle(int thisMinDegree, int thisMaxDegree, int stepSpeedInterval)
{
  if (currentMillis - prevMillisServo >= stepSpeedInterval)
  {
    prevMillisServo = currentMillis;
    prevServoPosition = servoPosition; //save the last recorded servoPosition into memory as prevServoPosition
    servoPosition = prevServoPosition + servoDegrees; //advance the most current servoPosition by servoDegrees

    if (servoPosition >= thisMaxDegree) //if servo surpasses current MaxDegree value
    {
      servoPosition = thisMaxDegree; //then set the servoPosition to current MaxDegree value
      servoDegrees = -1 * servoDegrees; //and reverse the direction of the servo
      servoMvmtUp = false;
    }
    else if (servoPosition <= thisMinDegree) //if servoPos surpasses MinDegree value (this doesn't fluctuate)
    {
      servoPosition = thisMinDegree;
      servoDegrees = -1 * servoDegrees;
      servoMvmtUp = true;
    }

    servo1.write(servoPosition);
    //Serial.println(servoPosition);
  }
}

void servoSweep()
{
  /*
  //adding random fluctuations to upper degree limit -- debug, gets stuck in loops where it only descends after reaching randomized MaxVal....
  if (currentMillis - prevMillisServo >= servoFastInterval)
  {
    prevMillisServo = currentMillis;
    servoPosition = servoPosition + servoDegrees;

    if (servoPosition >= servoMaxDegrees) //if servo surpasses current MaxDegree value
    {
      servoPosition = servoMaxDegrees; //then set the servoPosition to current MaxDegree value
      servoDegrees = -servoDegrees; //and reverse the direction of the servo 
      //servoMaxReached = !servoMaxReached;
    }

    if (servoPosition <= servoMinDegrees) //if servoPos surpasses MinDegree value (this doesn't fluctuate)
    {
      servoPosition = servoMinDegrees;
      servoDegrees = -servoDegrees;
      randVal = random(0, 10); //pick a val between 0-9
      if (randVal >= 8) //if randVal 8-9 (20% chance out of range from 0-9) 
      {
        //Serial.println("!!randValTrig!!");
        //servoMaxWild = true; //servoMax 'wild state' enabled (slightly randomized upper servo limit generated for next cycle upwards)
        servoMaxDegrees = servoMaxStaticDegrees + (random(20, 41)); //randomly add 8-20 degrees to servoMax limit on next upswing
        //Serial.println("servoMaxDegrees--");
        //Serial.println(servoMaxDegrees);
      }
      else //if randVal is 0-7, servoMax limit will be standard 'MaxStaticDegrees' value without any randomly added value
      {
        //servoMaxWild = false;
        servoMaxDegrees = servoMaxStaticDegrees;
      }
      //servoMinReached = true;
      //Serial.println(servoMinDegrees);
    }

    servo1.write(servoPosition);
    //Serial.println("pos:");
    Serial.println(servoPosition);
  }
*/

 
  //my refined version of servo sweep code from Internet  
  if (currentMillis - prevMillisServo >= servoInterval)
  {
    prevMillisServo = currentMillis;
    prevServoPosition = servoPosition; //save the last recorded servoPosition into memory as prevServoPosition
    servoPosition = prevServoPosition + servoDegrees; //advance the most current servoPosition by servoDegrees

    if (servoPosition >= servoMaxDegrees) //if servo surpasses current MaxDegree value
    {
      servoPosition = servoMaxDegrees; //then set the servoPosition to current MaxDegree value
      servoDegrees = -1 * servoDegrees; //and reverse the direction of the servo
      servoMvmtUp = false;
    }
    else if (servoPosition <= servoMinDegrees) //if servoPos surpasses MinDegree value (this doesn't fluctuate)
    {
      servoPosition = servoMinDegrees;
      servoDegrees = -1 * servoDegrees;
      servoMvmtUp = true;
    }

    servo1.write(servoPosition);
    //Serial.println(servoPosition);
  }
}

unsigned int monitorMic()
{
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
 
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
 
  while (millis() - startMillis < sampleWindow) // collect data for 50 mS
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

/*
unsigned int monitorMicAlt()
{
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
 
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
 
  if (millis() - startMillis < sampleWindow) // collect data for 50 mS
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
*/

void writeToLED(byte r, byte g, byte b)
{
  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluPin, b);
}

void autoFlashLEDServo(byte thisRed, byte thisGreen, byte thisBlue, int rateMultiplier)
{
  currentMillisLED = millis(); //get the current time
  unsigned int timeElapsed = currentMillisLED - prevMillisLED; //get how much time has passed since we updated the animation

  if (timeElapsed > (servoInterval * rateMultiplier))
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

void potToRGB()
{
  redVal = map(analogRead(pot1Pin), 0, 1023, 0, 255);
  greenVal = map(analogRead(pot2Pin), 0, 1023, 0, 255);
  bluVal = map(analogRead(pot3Pin), 0, 1023, 0, 255);
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
        if (servoActivated == true)
        { //return servo to maxDegree position and reset state flags if button pressed in the middle of servoSustain sequence
          servoSeqStop(); 
        } 
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


/*
void debugRandomSeqDurTest()
{
  randServoSeqDur();
  Serial.print(" | servoSeqDur = ");
  Serial.println(servoSeqDur);
  //Serial.print(" | randVal = ");
  //Serial.println(thisRandVal);
  delay(450);
}

void debugMicTest1()
{
  micVal = monitorMic();
  Serial.print("soundThres: ");
  Serial.print(soundThresh);
  Serial.print(" micVal: ");
  Serial.println(micVal);
  if (monitorMic() < 345) {writeToLED(0,0,250);}
  else if (monitorMic() >= 345 && monitorMic() <= 700) {writeToLED(50,250,0);}
  else {writeToLED(250,0,0);}
}
*/
