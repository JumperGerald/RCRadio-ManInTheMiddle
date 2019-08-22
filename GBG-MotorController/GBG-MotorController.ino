// Modified Servo.h REFRESH_INTERVAL from 20000 to 16666. This will set the Frequency from 50hz to 60hz.
#include <Servo.h>
#include <SimpleTimer.h>

Servo MotorController; // Output for Motor Controller
SimpleTimer t; // Timer used for acceleration and deceleration over time
int timerID;

// Pin Alocations
const int throttle = 3; // trinket pro interrupt pin 3 // Uno interrupt pin 2  //
const int BBButton = 10; // trinket pro pin 10// Uno pin 4 //
const int motor = 5; // Trinket pro pin 5 // Uno pin 3 //
const int BBB_mode = 6; // trinket pro pin 6 // Uno pin 6 //
const int remote_mode = 8; // trinket pro pin 8 // Uno pin 7 //
const int audioToggle = 11; // Check pin# at later time. Toggles Relay for the music amplifier to play music.

// Used to record data from radio pwm signal
volatile int radio_pwm_value = 1250;
volatile int radio_prev_time = 0;


// Stores max speed from Acceleration Table Lookup 0 - 45
int maxSpeedIterator = 45;

// Acceleration and deceleration intervals
const int Acceleration_Speed_Value = 300;
const int Deceleraation_Speed_Value = 100;

// Stores BBButton State
int BBButtonState = 0;
int BBB_mode_state = 0;
int remote_mode_state = 0;

// Used to keep track of steps on acceleration curve // Hard coded instead of calculated in real time on boot to reduce total calculations at boot.
int accelerationIterator = 0;
int accelerationTable[] = {77,77,77,90,90,90,93,93,93,96,96,96,99,99,99,100,100,100,101,101,101,102,102,102,105,105,105,106,106,106,107,110,113,114,115,116,117,118,119,120,121,122,123,124,125,126};

void setup()
{
 // Instantiation of Pins
 attachInterrupt(digitalPinToInterrupt(3), Radio_PWM_Change, CHANGE); // 0 in this function means PIN 3
 pinMode(BBB_mode, INPUT);
 pinMode(remote_mode, INPUT);
 pinMode(BBButton, INPUT);
 pinMode(audioToggle, OUTPUT);

 // When the motor is attached it appears to write a value causing the car to move without input
 MotorController.attach(motor); // Attach Servo Library to motor pin

 // Writing 77 to the motor till the motor to stop after it is attached
 MotorController.write(77);

 pinMode(LED_BUILTIN, OUTPUT);
 

 timerID = t.setInterval(1000, accelerationCurve);
 delay(3000);
 
 Serial.begin(9600); // setup Serial Port
}

void loop()
{
  Serial.println(radio_pwm_value);
  t.run();
}


// Outputs to Motor Controller using modified servo library
void writeToMotorController(int speedVal){

  // Fix for reverse bug
  if (speedVal >= 0 ){
    MotorController.write(accelerationTable[speedVal]);
  }
}


// Gradually iterate through lookup table until max speed.
void accelerationCurve()
{
  // Get current value of mode select switch
  BBB_mode_state = digitalRead(BBB_mode);
  remote_mode_state = digitalRead(remote_mode);
  
  // BBB_mode == LOW && remote_mode == HIGH; Fully remote control (No Big Blue Button)
  // BBB_mode == HIGH && remote_mode == LOW; Big Blue Button Throttle control
  // BBB_mode == LOW && remote_mode == LOW; Diagnostic Mode
  

  if ( BBB_mode_state == HIGH && remote_mode_state == LOW){
    // Fully remote control mode

    //Serial.println(radio_pwm_value);
    // If trigger is pushed (reverse)
    if (radio_pwm_value < 900 && radio_pwm_value >= 0){
      MotorController.write(65);      accelerationIterator = 3f;

    // If trigger is pulled (accelerate)
    } else if (radio_pwm_value > 1700) {
      accelerate();
      digitalWrite(audioToggle, HIGH);
      // If trigger is resting (slow down to stop)
    } else {
      decelerate();
    }
  }else if (BBB_mode_state == LOW && remote_mode_state == HIGH){
    // Use acceleration curve to gradually increase speed over time
    //Serial.println("Remote Mode");
    BBButtonState = digitalRead(BBButton);
    //Serial.println(BBButtonState);
    if (radio_pwm_value < 900 && radio_pwm_value >= 0){
      MotorController.write(65);
      accelerationIterator = 3;
    } else if (radio_pwm_value < 1700){
      // BBButtonState == HIGH; iterate up though acceleration table until end then maintain speed
      // BBButtonState == LOW; iterate down through acceleration table until stop then maintain speed
      if (BBButtonState == HIGH)
      {
        accelerate();
      }
      else if (BBButtonState == LOW) 
      {
        decelerate();
      }
    } else {
      decelerate();
    }
  } else if ( BBB_mode_state == LOW && remote_mode_state == LOW) {
    // Print out all variables to serial to see live output
    DiagnosticMode();
  } else {
    Serial.println("Mode Select Error! Check Mode Switch!");
  } 
}

void accelerate(){
  Serial.println("accelerating");
  // Bounds check to verify iterator doesn't exceed the bounds of the array
    digitalWrite(audioToggle, HIGH);
    if (accelerationIterator <= 44){
      if (accelerationIterator < maxSpeedIterator){
        accelerationIterator += 1;
      } 
      writeToMotorController(accelerationIterator);
      t.deleteTimer(timerID);
      timerID = t.setInterval(Acceleration_Speed_Value, accelerationCurve);
    } else {
      Serial.println("Iterator Out of Bounds ++");
    }
}

void decelerate(){
  //Serial.println("decelerating");
  if (accelerationIterator >= 0){
      if (accelerationIterator >= 1){
        accelerationIterator -= 1;
      }
      writeToMotorController(accelerationIterator);
      t.deleteTimer(timerID);
      timerID = t.setInterval(Deceleraation_Speed_Value, accelerationCurve);
    } else {
      Serial.println("Iterator Out of Bounds --");
    }

  if (accelerationIterator == 0 ) {
      digitalWrite(audioToggle, LOW);
  }
}

// y=0.08276545x-36.33122 linear. Y == output to motor controller
// Functioned used to calculate the control speed from the given pwm value of the radio.
double calculateLinearAcceleration(int radio_pwm_value){
  double controlSpeed = 0.08276545*radio_pwm_value-36.33122;
  if (controlSpeed > 0){
    return controlSpeed;
  } else {
    return 0;
  }
}


// When PWM signal from Radio is HIGH
void Radio_PWM_Rising(){
  attachInterrupt(digitalPinToInterrupt(3), Radio_PWM_Falling, FALLING);
  radio_prev_time = micros();
}

// When PWM signal from Radio is LOW
void Radio_PWM_Falling(){
  attachInterrupt(digitalPinToInterrupt(3), Radio_PWM_Rising, RISING);
  radio_pwm_value = micros()-radio_prev_time;
  Serial.println("RIP");
}

//   attachInterrupt(digitalPinToInterrupt(3), Radio_PWM_Change, Change);
void Radio_PWM_Change(){
  
  if ( radio_prev_time == 0){
    radio_prev_time = micros();
  } else {
    volatile int pwm_val = 0;
    pwm_val = micros()-radio_prev_time;
    //Serial.println(pwm_val);
    if (pwm_val > 1400 || pwm_val < 1200){
     if (pwm_val < 2000){
          radio_pwm_value = pwm_val;
     }
     //Serial.println(radio_pwm_value);
     radio_prev_time = micros();
  } else {
    radio_prev_time = micros();
  }
    
  }
}

void DiagnosticMode(){

  // Get current value of BBButton
  BBButtonState = digitalRead(BBButton);

  if (BBButtonState == HIGH){
      digitalWrite(audioToggle, HIGH);
  } else {
      digitalWrite(audioToggle, LOW);
  }

  
  
  Serial.println("Diagnostic Mode");
  Serial.println("Remote Control Mode Settings:");
  Serial.println("-------------------------------------");
  Serial.println("Throttle Value from radio: " + radio_pwm_value);
  Serial.println("LinearAcceleration Conversion Value: " + String(calculateLinearAcceleration(radio_pwm_value)));
  Serial.println("-------------------------------------");
  Serial.println("Semi-Remote Control Mode Settings:");
  Serial.println("-------------------------------------");
  Serial.println("Child Throttle Button Value: " + BBButtonState);
  Serial.println("Max Speed Value: " + String(accelerationTable[maxSpeedIterator]));
  Serial.println("Acceleration delay in miliseconds between steps: " + Acceleration_Speed_Value);
  Serial.println("Deceleration delay in miliseconds between steps: " + Deceleraation_Speed_Value);
  Serial.println("-------------------------------------");
 
}
