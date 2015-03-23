#include <TimerOne.h>
#include <Wire.h>
#include <Time.h>
#include <Adafruit_PWMServoDriver.h>


#define MOTOR_0        204
#define MOTOR_100      408
#define MOTOR_50       ((MOTOR_0+MOTOR_100)/2)
#define MOTOR_25       ((MOTOR_0+MOTOR_50)/2)
#define MOTOR_12       ((MOTOR_0+MOTOR_25)/2)

volatile unsigned count = 0;

#define LED_1G    39
#define LED_2A    41
#define LED_3G    43
#define LED_4R    45
#define LED_5R    47
#define LED_6R    49
#define LED_ON(led)    digitalWrite(led, LOW)
#define LED_OFF(led)   digitalWrite(led, HIGH)
#define LED_ALLON()    LED_ON(LED_1G); LED_ON(LED_2A); LED_ON(LED_3G); LED_ON(LED_4R); LED_ON(LED_5R); LED_ON(LED_6R)
#define LED_ALLOFF()   LED_OFF(LED_1G); LED_OFF(LED_2A); LED_OFF(LED_3G); LED_OFF(LED_4R); LED_OFF(LED_5R); LED_OFF(LED_6R)
#define LED_ERROR(led) LED_OFF(LED_1G); LED_OFF(LED_2A); LED_OFF(LED_3G); LED_ON(led)

#define GREEN_TOGGLE         30

#define KNOB1         A11
#define KNOB2         A10
#define KNOB3         A9
#define KNOB4         A8


#define BELT_WAIT               A8
#define BELT_STEP               2
#define BELT_DIRECTION          5
#define BELT_ENCODER            A13

volatile double next_beltpos = 0;
volatile unsigned long belt_watchdog = 0;
volatile int pumpDivCount = 0;
int pumpSpeedDividor =5;


#define STATION_SEAL_MOTOR      14
#define STATION_SEAL_HOTWIRE    11
#define HOTWIRE_ON              218
#define HOTWIRE_OFF             200
#define STATION_SEAL_SPEED      A9
#define STATION_SEAL_HALL       28
#define STATION_SEAL_HEAT       25
#define STATION_SEAL_WIRE       11
#define STATION_SEAL_TEMP       A12
#define TEMP_SETPOINT           678 //nessary because of problems with knob
//#define STATION_SEAL_TEMP_SET()   (analogRead(A10)/2+200)
#define STATION_SEAL_TEMP_ERROR() (seal_temp_set - seal_temp)
#define empty_count_max         11

volatile int seal_temp_set = 200;
volatile int seal_temp = 210;
//seal power now bang bang 
volatile int seal_temp_power = 200;
int empty_count = 0;
int station_time = 6000;

volatile enum {
  SEAL_TURN_1,
  SEAL_SLOW_SPEED,
  SEAL_TURN_2,
  SEAL_DONE,
  SEAL_OFF,
} seal_state = SEAL_OFF;

volatile unsigned seal_temperature = 0;


#define STATION_FILL_SERVO      13
#define STATION_FILL_SWITCH     27
#define STATION_FILL_UP         235
#define STATION_FILL_CHECK      165
#define STATION_FILL_CHECK_UPPER 190
#define STATION_FILL_CHECK_LOWER 140
#define STATION_FILL_TORQUE_THRESHHOLD 381
#define STATION_FILL_DOWN       114
#define STATION_FILL_STEP       3
#define STATION_FILL_DIRECTION  6
#define STATION_FILL_STEPS      1025
#define STATION_FILL_SENSOR     A14

volatile enum {
  FILL_RESET,
  FILL_CHECK_1,
  FILL_CHECK_2,
  FILL_PUMP,
  FILL_SLOW_UP,
  FILL_DONE,
  FILL_OFF,
} fill_state = FILL_OFF;

volatile int fill_stepsleft = 0;


#define STATION_CUT_MOTOR     15
#define STATION_CUT_HALL      24

volatile enum {
  CUT_TURN_1,
  CUT_TURN_2,
  CUT_DONE,
  CUT_OFF,
} cut_state = CUT_OFF;



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


volatile enum {
  INIT,
  WAIT,
  MOVE_BELT,
  STATIONS,
  BELT_ERROR,
  FILL_ERROR,
} global_state = INIT;

#define IS_ERROR()  (global_state == BELT_ERROR || global_state == FILL_ERROR)


int debounceRead(int pin) {
  unsigned value = digitalRead(pin);
  unsigned last_debounce = millis();
  
  while ((millis() - last_debounce) < 50) {
    delay(1);
    int newvalue = digitalRead(pin);
    if (newvalue != value) {
      value = newvalue;
      last_debounce = millis();
    }
  }
  
  return value;
}

void setup()
{
  Serial.begin(9600);
  
  // Initialize the user interface
  {
    // Set LEDs to out (active low)
    pinMode(LED_1G, OUTPUT);
    pinMode(LED_2A, OUTPUT);
    pinMode(LED_3G, OUTPUT);
    pinMode(LED_4R, OUTPUT);
    pinMode(LED_5R, OUTPUT);
    pinMode(LED_6R, OUTPUT);
    
    // Set up big green button
    pinMode(GREEN_TOGGLE, INPUT_PULLUP);
  }
  
  LED_ALLON();
  
  // Init I2C PWM board to 50Hz
  {
    pwm.begin();
    pwm.setPWMFreq(50);
    
    // Set PWM to OFF, wait a bit for register
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
    pwm.setPWM(STATION_SEAL_HOTWIRE, 0, HOTWIRE_OFF);
    delay(1000);
  }
  
  // Initialize the cutter
  {
    pinMode(STATION_CUT_HALL, INPUT_PULLUP);
    
    // Home cutter motor
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_50);
    while (digitalRead(STATION_CUT_HALL) == 0);
    while (digitalRead(STATION_CUT_HALL) == 1);
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
    Serial.println("cutter homed");
  }
  
  // Initialize the filler
  {
    // Pump stepper
    pinMode(STATION_FILL_STEP, OUTPUT);
    pinMode(STATION_FILL_DIRECTION, OUTPUT);
    digitalWrite(STATION_FILL_DIRECTION, 0);
    
    // Fill check switch
    pinMode(STATION_FILL_SWITCH, INPUT_PULLUP);

    // Put pump servo in upward position
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    delay(250);
  }
  
  // Initialize sealer
  {
    // Configure heater
    pinMode(STATION_SEAL_HALL, INPUT_PULLUP);
    pinMode(STATION_SEAL_HEAT, OUTPUT);
    digitalWrite(STATION_SEAL_HEAT, 1);
    Serial.println("heater configured");
    //seal_temperature = analogRead(STATION_SEAL_TEMP);
    
    // Home sealer motor
    Serial.println("sealer homeing 0");
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_50);
    while (digitalRead(STATION_SEAL_HALL) == 0);
    Serial.println("sealer homeing 1");
    while (digitalRead(STATION_SEAL_HALL) == 1){Serial.print(".");}
    Serial.println();
    Serial.println("sealer homeing 2");
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
    Serial.println("sealer homed");
  }
  delay(200);
  Serial.println("delayed");
  // Initialize belt
  {
    // Belt stepper
    pinMode(BELT_STEP, OUTPUT);
    pinMode(BELT_DIRECTION, OUTPUT);
    digitalWrite(BELT_DIRECTION, 0);
    
    // Get encoder value, set belt home position
    //next_beltpos = 697.0;
    next_beltpos = analogRead(BELT_ENCODER);
    next_beltpos += 102.4 - fmod(next_beltpos, 102.4);
  }
  
  // Initialize CNC shield
  {
    // Set CNC shield ENable (active low)
    pinMode(8, OUTPUT);
    digitalWrite(8, 0);
  }
  delay(200);
  Serial.println("delayed");
  // Configure stepper timer interrupt
  Timer1.initialize(300);
  Timer1.attachInterrupt(timerint);
  /* heater removed ->
  // Wait for temperature
  LED_ALLOFF(); LED_ON(LED_2A);
  seal_temp_set = TEMP_SETPOINT;
  Serial.print("temp set: ");
  Serial.println(seal_temp_set);
  while (seal_temp_set - seal_temp > 0) {
    seal_temp_set = STATION_SEAL_TEMP_SET();
    pwm.setPWM(STATION_SEAL_WIRE, 0, seal_temp_power);
    Serial.print("time: ");
    Serial.print(millis()/1000);
    Serial.print(" temp set: ");
    Serial.print(seal_temp_set);
    Serial.print(", temp: ");
    Serial.print(seal_temp);
    Serial.print(" setPWM:");
    Serial.println(seal_temp_power);
    delay(100);
  }
  Serial.println("here 1");
 <- heater removed */
  // Wait for green button to depress
  LED_ALLOFF(); LED_ON(LED_3G);
  while (debounceRead(GREEN_TOGGLE) == 0) {
    //int redKnobVal = analogRead(A10);
    //seal_temp_set = (redKnobVal/2)+ 200;
//  seal_temp_set = (analogRead(A10)/2+200);
    Serial.println("here 2");
    pwm.setPWM(STATION_SEAL_WIRE, 0, seal_temp_power); //seal temp set read in timer1 (no glitch here)
    Serial.println(seal_temp_power); //seal temp set read in timer1 (no glitch here)));
    delay(100);
  }
  
  //global_state = WAIT;
}

void loop()
{
 pwm.setPWM(STATION_SEAL_WIRE, 0, seal_temp_power); //seal temp set read in timer1 (no glitch here)
 Serial.println(seal_temp_power); //seal temp set read in timer1 (no glitch here)
  // Check for errors
  while (IS_ERROR() && debounceRead(GREEN_TOGGLE) == 0) {
    //seal_temp_set = (analogRead(A10)/2+200);
    delay(100);
     pwm.setPWM(STATION_SEAL_WIRE, 0, seal_temp_power); //seal temp set read in timer1 (no glitch here)
     Serial.print("ERROR,   ");
      Serial.print("seal temp power:");
     Serial.println(seal_temp_power); //seal temp set read in timer1 (no glitch here)
  }
  LED_OFF(LED_4R); LED_OFF(LED_5R); LED_OFF(LED_6R);

handle_pause:

  global_state = WAIT;
  
  //handle empty count over max
  Serial.print(" number empty: ");
  Serial.println(empty_count);
  while (empty_count >= empty_count_max && debounceRead(GREEN_TOGGLE) == 0) {
    LED_OFF(LED_1G); LED_ON(LED_3G);
    delay(10);
    //seal_temp_set = (analogRead(A10)/2+200);
        //Serial.print("knob value:");
    Serial.println("stopped: too many empty");
    Serial.print("temp set: ");
    Serial.print(seal_temp_set);
    Serial.print(", temp: ");
    Serial.print(seal_temp);
    Serial.print(" heater Power: ");
    Serial.println(seal_temp_power);
    //STATION_SEAL_TEMP_SET));
    pwm.setPWM(STATION_SEAL_WIRE, 0, seal_temp_power);//seal_temp_set is knob A10 read in timer1 (needed to stop glitching)
    delay(100);
  }
  // Handle pause
  while (debounceRead(GREEN_TOGGLE) == 1) {
    LED_OFF(LED_1G); LED_ON(LED_3G);
    empty_count = 0;
    delay(10);
    //seal_temp_set = (analogRead(A10)/2+200);
        //Serial.print("knob value:");
    // seal_temp_set = analogRead(A10);
    Serial.print("temp set: ");
    Serial.print(seal_temp_set);
    Serial.print(", temp: ");
    Serial.print(seal_temp);
    Serial.print(" Heater power: ");
    Serial.println(seal_temp_power);//seal_temp_set is knob A10 read in timer1(needed to stop glitching)
    //STATION_SEAL_TEMP_SET));
    pwm.setPWM(STATION_SEAL_WIRE, 0, seal_temp_power);//seal_temp_set is knob A10 read in timer1 (needed to stop glitching)
    delay(100);
  }
  LED_OFF(LED_3G);
  
 //radiant heater  /* heater removed ->
  // Handle heater
  LED_OFF(LED_1G);
  if (!IS_ERROR() && STATION_SEAL_TEMP_ERROR() > 78) {
    while (!IS_ERROR() && STATION_SEAL_TEMP_ERROR() > 20) {
      delay(100);
    }
    
    LED_ON(LED_3G);
    while (debounceRead(GREEN_TOGGLE) == 0) {
      delay(100);
    }
    
    goto handle_pause;
  }
  //radiant heater <- heater removed */
  LED_ON(LED_1G);
  
  if (global_state == WAIT) {
    double old_beltpos = next_beltpos;
    
    // Advance belt to next position
    next_beltpos = fmod(next_beltpos + 102.4, 1024);
    belt_watchdog = millis();
    global_state = MOVE_BELT;
    
    // Wait for state change
    while (global_state == MOVE_BELT) {
      delay(10);
    }
    
    if (IS_ERROR()) {
      // Revert old belt position on error
      next_beltpos = old_beltpos;
    }
  }
  
  if (global_state == STATIONS) {
    // Cut station
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_100);
    cut_state = CUT_TURN_1;
    
    // Fill station
    unsigned long fill_start = millis();
    unsigned long station_start = millis();
    unsigned fill_tries = 0;
    unsigned fill_servopos = 0;
    double fill_sensorval = 0;
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_CHECK_UPPER);
    fill_state = FILL_CHECK_1;
    
    // Seal station
   pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_50);
   seal_state = SEAL_TURN_1;
    
    // Wait for state change
    while ((cut_state != CUT_OFF || fill_state != FILL_OFF || seal_state != SEAL_OFF)) {
      if (cut_state == CUT_DONE) {
        pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
        cut_state = CUT_OFF;
      }
      
      unsigned long now = millis();
      #define FILL_TIME()            (now - fill_start)
      #define FILL_SENSE()           (fill_sensorval = analogRead(STATION_FILL_SENSOR))
      #define FILL_TRANS(state) { fill_start = millis(); fill_state = state; }
      if (fill_state == FILL_RESET && FILL_TIME() > 500) {
        
        // 3 strikes and you're out
        if (fill_tries >= 4) {
          fill_state = FILL_OFF;
          //global_state = FILL_ERROR;  does not stop for fill error 
          LED_ERROR(LED_5R);
        } else {
          pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_CHECK_UPPER);
          FILL_TRANS(FILL_CHECK_1);
        }
      } else if (fill_state == FILL_CHECK_1/* && FILL_TIME() > 500*/) {
        
        //pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_DOWN);
        fill_servopos = STATION_FILL_CHECK_UPPER;
        FILL_TRANS(FILL_CHECK_2);
        
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        //delay(100); FILL_SENSE();
        
      } else if (fill_state == FILL_CHECK_2) {
        //fill torque//
        if (fill_servopos > STATION_FILL_DOWN) {
          if (FILL_SENSE() < STATION_FILL_TORQUE_THRESHHOLD && fill_servopos > STATION_FILL_CHECK_LOWER && fill_servopos < STATION_FILL_CHECK) {
            Serial.print("fail 1 ");
            Serial.println(fill_sensorval);
           if (FILL_SENSE() < STATION_FILL_TORQUE_THRESHHOLD){
            Serial.print("fail 2 ");
            Serial.println(fill_sensorval);
            // Fill switch contacted something, bad
            fill_tries += 1;
            pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
            FILL_TRANS(FILL_RESET);
           }
          } else {
            Serial.println(fill_sensorval);
            fill_servopos -= 1;
            pwm.setPWM(STATION_FILL_SERVO, 0, fill_servopos);
            delay(9);
          }
        
        } else {   
          if (digitalRead(STATION_FILL_SWITCH) == 1) {
            // Fill switch contacted nothing, empty
            empty_count ++ ;
            FILL_TRANS(FILL_DONE);
          } else {
            // Fill switch engaged
            fill_servopos = STATION_FILL_DOWN;
            fill_stepsleft = STATION_FILL_STEPS;
            FILL_TRANS(FILL_PUMP);
            empty_count = 0;
          }
        }
      } else if (fill_state == FILL_SLOW_UP) {
        
        if(fill_servopos < STATION_FILL_UP){
          fill_servopos += 1;
          pwm.setPWM(STATION_FILL_SERVO, 0, fill_servopos);
          delay(9);
          
        } else {
          FILL_TRANS(FILL_OFF);
        }
        
      } else if (fill_state == FILL_DONE) {
        pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
        fill_state = FILL_OFF;
        //Serial.print("fill time:");
        //Serial.print(FILL_TIME());
        //Serial.print(", ");
        //Serial.println(millis()-fill_start);
        
      }
      
      if (seal_state == SEAL_SLOW_SPEED) {
        pwm.setPWM(STATION_SEAL_MOTOR, 0, 381);//map(analogRead(STATION_SEAL_SPEED), 0, 1023, map(10, 0, 100, MOTOR_0, MOTOR_100), map(25, 0, 100, MOTOR_0, MOTOR_100)));
        seal_state = SEAL_TURN_2;
      } else if (seal_state == SEAL_DONE) {
        pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
        seal_state = SEAL_OFF;
      }
      /* Serial.print(count);  
       Serial.print("-- "); 
      Serial.print(analogRead(A11)); 
  Serial.print(", "); 
  Serial.print(analogRead(A10));
 Serial.print(", ");  
  Serial.print(analogRead(A9));
 Serial.print(", ");  
  Serial.println(analogRead(A8)); 
      delay(1); */
    }
    
    
    // Reset all pwms (in case error happened)
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
    pwm.setPWM(STATION_SEAL_HOTWIRE, 0, seal_temp_power); //seal temp set read in timer1 (no glitch here)
    cut_state = CUT_OFF; fill_state = FILL_OFF; seal_state = SEAL_OFF;
    
   
   //knobed pause now normalised to start time
    station_time = map(analogRead(BELT_WAIT), 0, 1023, 2000, 9000);
    Serial.print("VAR station time: ");
    Serial.println(station_time);
    Serial.print("PW station time: ");
    Serial.println(millis() - station_start);
    //wait for station time so that radiant heater has time to work 
    while(millis()< station_start + station_time){
      delay(10);
    }
    Serial.print("AW station time: ");
    Serial.println(millis() - station_start);
  }
  
  // Handle knobbed pause
  //knobed pause now normalised to start time(above) delay(map(analogRead(BELT_WAIT), 0, 1023, 0, 5000));
}

//volatile unsigned count = 0;
void timerint()
{
  //heater is now bang bang seal_temp_set = analogRead(A10);//(needed to stop glitching)
  
  if (global_state == MOVE_BELT || global_state == INIT) {
    if(fabs((double)analogRead(BELT_ENCODER) - next_beltpos) > 10.0) {
      digitalWrite(BELT_STEP, digitalRead(BELT_STEP) ^ 1);
    } else if (global_state == MOVE_BELT) {
      // Next step
      global_state = STATIONS;
    }
  }
  
  if (global_state == WAIT && digitalRead(GREEN_TOGGLE) == 1 && analogRead(KNOB1) > 512 ){
    //->
    if ( pumpDivCount >= pumpSpeedDividor) {
        digitalWrite(STATION_FILL_STEP, digitalRead(STATION_FILL_STEP) ^ 1);
        pumpDivCount = 0;
      } 
      else {
        pumpDivCount++;
      }
    //<- digitalWrite(STATION_FILL_STEP, digitalRead(STATION_FILL_STEP) ^ 1);
  }
  
  /*if (global_state == STATIONS)*/ {
    // Check cutter station
    if (cut_state == CUT_TURN_1 || cut_state == CUT_TURN_2) {
      unsigned hall = digitalRead(STATION_CUT_HALL);
      if (cut_state == CUT_TURN_1 && hall == 1) {
        cut_state = CUT_TURN_2;
      } else if (cut_state == CUT_TURN_2 && hall == 0) {
        cut_state = CUT_DONE;
      }
    }
    
    // Check fill station
    if (fill_state == FILL_PUMP) {
      if (fill_stepsleft > 0 && pumpDivCount >= pumpSpeedDividor) {
        digitalWrite(STATION_FILL_STEP, digitalRead(STATION_FILL_STEP) ^ 1);
        fill_stepsleft -= 1;
        pumpDivCount = 0;
      } 
      else if (fill_stepsleft > 0) {
        pumpDivCount++;
      }
      else {
        fill_state = FILL_SLOW_UP;
      }
    }
    
    // Check seal station
   if (seal_state == SEAL_TURN_1 || seal_state == SEAL_TURN_2) {
      unsigned hall = digitalRead(STATION_SEAL_HALL);
      if (seal_state == SEAL_TURN_1 && hall == 1) {
        seal_state = SEAL_SLOW_SPEED;
      } else if (seal_state == SEAL_TURN_2 && hall == 0) {
        seal_state = SEAL_DONE;
      }
    } 
  }
  
  //radiant heater /* heater removerd ->
  
  if ((count % 2) == 1) {
    seal_temp = analogRead(STATION_SEAL_TEMP);
  } else {  
    seal_temp_set = analogRead(A10)/2+200; 
  }
 
  if ((count % 1000) == 0) {
    int err = STATION_SEAL_TEMP_ERROR();
    
    if( err > 0) {// "!IS_ERROR() &&" deleted
      seal_temp_power = 444;//digitalWrite(STATION_SEAL_HEAT, 0);
    } else if( err > -20 ){
      seal_temp_power = 399;
    } else{
      seal_temp_power = 200;//digitalWrite(STATION_SEAL_HEAT, 1);
    }
    
    if ( err > 72) { // "!IS_ERROR() &&" deleted
      LED_ON(LED_2A);
    } else {
      LED_OFF(LED_2A);
    }
  } 
//radiant heater  <- heater removed */
  
  // Check watchdogs
  if (global_state == MOVE_BELT && (millis() - belt_watchdog) > 1000) {
    global_state = BELT_ERROR;
    LED_ERROR(LED_4R);
  }
  
  count += 1;
  
}

