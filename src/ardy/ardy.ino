
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <MegaEncoderCounter.h>


int speedSetpointL = 0;
int speedSetpointR = 0;

int motorOutputL = 0;
int motorOutputR = 0;

#define ENCODER_RA_PIN 4
#define ENCODER_RB_PIN 5
#define ENCODER_RY_PIN 2
#define ENCODER_LA_PIN 6
#define ENCODER_LB_PIN 7
#define ENCODER_LY_PIN 3

#define MOT_L_DIR_PIN 8
#define MOT_L_PWM_PIN 9
#define MOT_R_DIR_PIN 10
#define MOT_R_PWM_PIN 11

#define LCD_OUT_PIN 14
#define LCD_IN_PIN 15

#define IR_SENSOR1_PIN A0
#define IR_SENSOR2_PIN A1
#define IR_SENSOR3_PIN A2
#define BATTERY_VOLTAGE_PIN A5

//#define RXLED_PIN 17
//#define SONAR_L_PING_PIN 15
//#define SONAR_L_ECHO_PIN 4
//#define SONAR_R_PING_PIN 14
//#define SONAR_R_ECHO_PIN 16


// encoder support
MegaEncoderCounter encoder(1);

long encoderRPos = 0;
long encoderLPos = 0;

long prevEncoderRPos = 0;
long prevEncoderLPos = 0;

int speedL = 0;
int speedR = 0;


// motor PID loops
double setpointL = 0.0, inputL = 0.0, outputL = 0.0;
double setpointR = 0.0, inputR = 0.0, outputR = 0.0;

#define KP 0.5
#define KI 1.0
#define KD 0.0

PID pidL(&inputL, &outputL, &setpointL, KP, KI, KD, DIRECT);
PID pidR(&inputR, &outputR, &setpointR, KP, KI, KD, DIRECT);


// IR sensors
int ir_sensor1_dist = 0;
int ir_sensor2_dist = 0;
int ir_sensor3_dist = 0;

int read_gp2d12_range(byte pin) {
  int tmp;
  int result;

  tmp = analogRead(pin);
  if (tmp < 3) {
    return 9999; // invalid value
  }

  result = (6787 / (tmp - 3)) - 4;
  
  if (result < 1) {
    return 9998;
  }
  
  return result;
} 


// battery monitoring
// analog inputs: 1024 counts = 5v
// A5 is hooked to external voltage divider: 277.6Kohm / 38.6Kohm (so 25v ~~ 3.5v)
#define BATTERY_COUNTS_TO_VOLTS (0.034)
int battery_counts = 0;
float battery_voltage = 0.0;



// sonar support

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 148;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 58;
}

long sonarTimeout = 50000;

long readSonar(int pingPin, int echoPin) {
  long duration;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH, sonarTimeout);
  delayMicroseconds(sonarTimeout - duration);
  return microsecondsToInches(duration);
}

long leftDist, rightDist;


// looping control
#define LOOP_INTERVAL 10
static unsigned long nextTime = 0;


// heartbeat LED
int led_value = 0;

// user input values
int inputValue = 0;
int sign = 1;


// one-time setup

void setup()
{
  pinMode(ENCODER_LA_PIN, INPUT); 
  pinMode(ENCODER_LB_PIN, INPUT);
  pinMode(ENCODER_RA_PIN, INPUT);
  pinMode(ENCODER_RB_PIN, INPUT);
  pinMode(MOT_L_DIR_PIN, OUTPUT);
  pinMode(MOT_R_DIR_PIN, OUTPUT);
  pinMode(IR_SENSOR1_PIN, INPUT);
  pinMode(IR_SENSOR2_PIN, INPUT);
  pinMode(IR_SENSOR3_PIN, INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  
//  pinMode(SONAR_L_PING_PIN, OUTPUT);
//  pinMode(SONAR_L_ECHO_PIN, INPUT);
//  pinMode(SONAR_R_PING_PIN, OUTPUT);
//  pinMode(SONAR_R_ECHO_PIN, INPUT);
   
  Serial.begin (57600);
    
  pidL.SetOutputLimits(-250.0, 250.0);
  pidR.SetOutputLimits(-250.0, 250.0);
  pidL.SetSampleTime(LOOP_INTERVAL);
  pidR.SetSampleTime(LOOP_INTERVAL);
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  
  nextTime = millis() + LOOP_INTERVAL;
}

// main loop

void loop() { 
  int c;
  long t;
  
 if ((t = (long) (millis() - nextTime)) >= 0) {
  nextTime += LOOP_INTERVAL;
  
  led_value = 1 - led_value;
//  digitalWrite(RXLED_PIN, led_value);

  ir_sensor1_dist = read_gp2d12_range(IR_SENSOR1_PIN);
  ir_sensor2_dist = read_gp2d12_range(IR_SENSOR2_PIN);
  ir_sensor3_dist = read_gp2d12_range(IR_SENSOR3_PIN);
  
  battery_counts = analogRead(BATTERY_VOLTAGE_PIN);
  battery_voltage = (float)battery_counts * BATTERY_COUNTS_TO_VOLTS;

  if(led_value) {
  //  leftDist = readSonar(SONAR_L_PING_PIN, SONAR_L_ECHO_PIN);
  }
  else {
  //  rightDist = readSonar(SONAR_R_PING_PIN, SONAR_R_ECHO_PIN);
  }
  
  encoderLPos = encoder.XAxisGetCount();
  speedL = encoderLPos - prevEncoderLPos;
  prevEncoderLPos = encoderLPos;
  
  encoderRPos = -encoder.YAxisGetCount();
  speedR = encoderRPos - prevEncoderRPos;
  prevEncoderRPos = encoderRPos;
  
  inputL = (double)speedL;
  inputR = (double)speedR;
  
  pidL.Compute();
  pidR.Compute();
  
  motorOutputL = (int)outputL;
  motorOutputR = (int)outputR;
  
  //motorOutputL = speedSetpointL;
  //motorOutputR = speedSetpointR;

  if (motorOutputL < 0)
  {
    digitalWrite(MOT_L_DIR_PIN, 0);
    analogWrite(MOT_L_PWM_PIN, -motorOutputL);
  }
  else
  {
    digitalWrite(MOT_L_DIR_PIN, 1);
    analogWrite(MOT_L_PWM_PIN, motorOutputL);
  }
  
  if (motorOutputR < 0)
  {
    digitalWrite(MOT_R_DIR_PIN, 0);
    analogWrite(MOT_R_PWM_PIN, -motorOutputR);
  }
  else
  {
    digitalWrite(MOT_R_DIR_PIN, 1);
    analogWrite(MOT_R_PWM_PIN, motorOutputR);
  }
  
  Serial.print("!MOT,");
  Serial.print(ir_sensor3_dist, DEC);
  Serial.print(",");
  Serial.print(ir_sensor2_dist, DEC);
  Serial.print(",");
  Serial.print(ir_sensor1_dist, DEC);
  Serial.print(",");
  Serial.print(leftDist, DEC);
  Serial.print(",");
  Serial.print(rightDist, DEC);
  Serial.print(",");
  Serial.print(encoderLPos, DEC);
  Serial.print(",");
  Serial.print(encoderRPos, DEC);
  Serial.print(",");
  Serial.print(speedSetpointL, DEC);
  Serial.print(",");
  Serial.print(speedSetpointR, DEC);
  Serial.print(",");
  Serial.print(speedL, DEC);
  Serial.print(",");
  Serial.print(speedR, DEC);
  Serial.print(",");
  Serial.print(motorOutputL, DEC);
  Serial.print(",");
  Serial.print(motorOutputR, DEC);
  Serial.print(",");
  Serial.println(battery_voltage, 2);
  
  while (Serial.available() > 0) {
    c = Serial.read();
    switch(c) {
      case 'p':
        Serial.print("!MOT,");
        Serial.print(encoderLPos, DEC);
        Serial.print(",");
        Serial.print(encoderRPos, DEC);
        Serial.print(",");
        Serial.print(speedSetpointL, DEC);
        Serial.print(",");
        Serial.print(speedSetpointR, DEC);
        Serial.print(",");
        Serial.print(speedL, DEC);
        Serial.print(",");
        Serial.print(speedR, DEC);
        Serial.print(",");
        Serial.print(motorOutputL, DEC);
        Serial.print(",");
        Serial.println(motorOutputR, DEC);
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        inputValue = inputValue * 10 + (c - '0');
        break;
      case '-':
        sign = -1;
        break;
      case 'l':
        speedSetpointL = inputValue * sign;
        setpointL = (double)speedSetpointL;
        inputValue = 0;
        sign = 1;
        break;
      case 'r':
        speedSetpointR = inputValue * sign;
        setpointR = (double)speedSetpointR;
        inputValue = 0;
        sign = 1;
        break;
      case 10: // LF
      case 13: // CR
      case 27: // ESC
        inputValue = 0;
        sign = 1;
        break;    
      case ']':
        speedSetpointR++;
        setpointR = (double)speedSetpointR;
        break;
      case '[':
        speedSetpointR--;
        setpointR = (double)speedSetpointR;
        break;
      case 'q':
        speedSetpointL--;
        setpointL = (double)speedSetpointL;
        break;
      case 'w':
        speedSetpointL++;
        setpointL = (double)speedSetpointL;
        break;
      case 's':
        speedSetpointL = 0;
        speedSetpointR = 0;
        setpointL = 0.0;
        setpointR = 0.0;
        outputL = 0.0;
        outputR = 0.0;
        motorOutputL = 0;
        motorOutputR = 0;
        break;
    } 
  } 
 }
}



