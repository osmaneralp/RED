// Version 4

#define printt 1

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// PS2 Controller Setup
#include <PS2X_lib.h>
#define PS2_DAT        53  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        49  //16
#define PS2_CLK        47  //17
#define pressures   false
#define rumble      false

PS2X ps2x; // create PS2 Controller Class
int error = 0;
byte type = 0;
byte vibrate = 0;
double ps2offset=0, ps2offsetL=0, ps2offsetR=0, offset=1.2;
//#define offset 1.2;

MPU6050 mpu;

#define HEARTBEAT_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool heartbeatState = false;
uint8_t heartbeatCount=0;

// Some generic defines.
#define TRUE 1
#define FALSE 0

// Input pins to motor driver. IN1 and 2 are direction pins. D2 is disable.
// See Freescale datasheet for more info.
#define IN2_R 5
#define IN1_R 6
#define D2n_R 7
#define IN2_L 8
#define IN1_L 9
#define D2n_L 10

// Defines for motor direction.
#define FORWARD 0
#define REVERSE 1

// Input pins from motor encoder.
#define ENCA_L 42
#define ENCB_L 43
#define ENCA_R 40
#define ENCB_R 41


#define LED_DMP_ERROR 22  // Anode
#define LED_DMP_ERROR_C 24  // Cathode

#define LED_CALIBRATING 27  // Anode
#define LED_CALIBRATING_C 29  // Cathode

#define BUTTON_RESET_PID_GND 23  // Anode
#define BUTTON_RESET_PID 25  // Cathode

// Variables for PID
double setpoint, input, wheelInput, output, lastOutput, filteredOutput;
int32_t pulses_L, prevPulses_L, velocity_L;
int32_t pulses_R, prevPulses_R, velocity_R;
int32_t position, velocity;

//Specify the links and initial tuning parameters
PID myPID(&input, &wheelInput, &output, &setpoint, DIRECT);


#define DEBUG_PIN 12  // An output I can look at on a scope for debugging
bool debugOutput = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {

  pinMode(HEARTBEAT_PIN, OUTPUT); // Set LED as output.

  pinMode(LED_DMP_ERROR, OUTPUT);
  pinMode(LED_DMP_ERROR_C, OUTPUT);
  digitalWrite(LED_DMP_ERROR_C, LOW);  // Set cathode to ground.
  digitalWrite(LED_DMP_ERROR, LOW);

  pinMode(LED_CALIBRATING, OUTPUT);
  pinMode(LED_CALIBRATING_C, OUTPUT);
  digitalWrite(LED_CALIBRATING_C, LOW);  // Set cathode to ground.
  digitalWrite(LED_CALIBRATING, HIGH);

  pinMode(BUTTON_RESET_PID, INPUT_PULLUP);
  pinMode(BUTTON_RESET_PID_GND, OUTPUT);
  digitalWrite(BUTTON_RESET_PID_GND, LOW);  // Set cathode to ground.

  /*
  while (1) {
    if (digitalRead(BUTTON_RESET_PID) == 0) {
      digitalWrite(LED_CALIBRATING, HIGH);
    } else {
      digitalWrite(LED_CALIBRATING, LOW);
    }
  }
  */

  //delay(1000);
  //digitalWrite(LED_DMP_ERROR, HIGH);
  //digitalWrite(LED_CALIBRATING, LOW);
  //while (1) {}

  // Setup for PS2 controller.
  error = ps2x.config_gamepad(pressures, rumble);
  type = ps2x.readType(); 
  



  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

#if 0
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
#endif

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(2, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    digitalWrite(LED_DMP_ERROR, HIGH);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(15000);
  digitalWrite(LED_CALIBRATING, LOW);

  // Set encoder pins as inputs.
  pinMode(ENCA_L, INPUT);
  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_L, INPUT);
  pinMode(ENCB_R, INPUT);

  // Set motor control pins as outputs.
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(D2n_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(D2n_R, OUTPUT);

  // This puts motors in brake mode. We will PWM the IN pins.
  digitalWrite(D2n_L, HIGH);
  digitalWrite(D2n_R, HIGH);

  // Start with motors off
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);

  // Initialize motor encoder interrupts
  attachInterrupt(ENCA_L, updateWheelCountA_L, CHANGE);
  attachInterrupt(ENCB_L, updateWheelCountB_L, CHANGE);
  attachInterrupt(ENCA_R, updateWheelCountA_R, CHANGE);
  attachInterrupt(ENCB_R, updateWheelCountB_R, CHANGE);

  // Initialize PID variables
  input = 0;
  setpoint = 0;

  // Turn on PID control
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20);
  myPID.SetControllerDirection(REVERSE);
  //myPID.SetTunings(15,160.0,2.5);
  //myPID.SetTunings(25,100.0,0.5);
  //myPID.SetTunings(12,80.0,0.0);
  //myPID.SetTunings(8.0,40.0,1.0);
  //myPID.SetTunings(18.0,90.0,0.5);
  //myPID.SetTunings(18.0,90.0,0.5,0,0.1);
  //myPID.SetTunings(20.0, 40.0, 0.0, 0.10, 0.0);
  //myPID.SetTunings(15.0, 40.0, 1.0, 0.0, 1.1); // good
  //myPID.SetTunings(16.0, 40.0, 1.0, 0.0, 1.2); // very good
  //myPID.SetTunings(17.0, 45.0, 1.0, 0.0, 1.2); // even better
  myPID.SetTunings(17.0, 45.0, 1.0, 0.0, 1.2);
}




//------------------------------------------------------------------------------
void loop() {

  float pitch, filteredPitch, targetPitch, timer;
  int pwm;

  timer = millis();
  printt && Serial.print(timer);
  //Serial.println("Main loop.");

  /*
  while (1) {
    setMotor_L(40);
    setMotor_R(40);
    delay(3000);
    setMotor_L(-40);
    setMotor_R(-40);
    delay(3000);

  }

  while (1) {
    analogWrite(IN1_L, 0);
    analogWrite(IN2_L, 0);
    analogWrite(IN1_R, 0);
    analogWrite(IN2_R, 40);
  }

  while (1) {
    Serial.println("pwm high");
    analogWrite(PWM_R,255);
    digitalWrite(InA_R, LOW);                        
    digitalWrite(InB_R, LOW);
    delay(2000);

    Serial.println("normal");
    analogWrite(PWM_R,0);
    digitalWrite(InA_R, LOW);                        
    digitalWrite(InB_R, LOW);
    delay(2000);
  }
  */



  /*
  //analogWrite(IN2_L, 0);                     
  //analogWrite(IN2_R, 0);                        
  analogWrite(IN1_L, 0);
  analogWrite(IN1_R, 0);

  for (pwm=15; pwm<60; pwm+=1) {
    Serial.print("pwm=");
    Serial.print(pwm);
    Serial.println("");
    //analogWrite(IN1_L, pwm);
    //analogWrite(IN1_R, pwm);
    //analogWrite(IN2_L, pwm);                     
    analogWrite(IN2_R, pwm);                        

    delay(2000);
  }
  while (1) {}
  */


  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) { }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitch = ((ypr[2] * 180) / M_PI);

    printt && Serial.print("  a=");
    printt && Serial.print(pitch);

    // blink LED to indicate activity
    if (++heartbeatCount == 50) {
      heartbeatCount = 0;
      heartbeatState = !heartbeatState;
      digitalWrite(HEARTBEAT_PIN, heartbeatState);
    }

    // *********************** PID and motor drive *****************

    velocity_L = pulses_L - prevPulses_L;
    prevPulses_L = pulses_L;
    //Serial.print("  vL=");
    //Serial.print(velocity_L);

    velocity_R = pulses_R - prevPulses_R;
    prevPulses_R = pulses_R;
    //Serial.print("  vR=");
    //Serial.print(velocity_R);

    velocity = (velocity_L + velocity_R)/2;
    position = (pulses_L + pulses_R)/2;

    input = pitch;
    wheelInput = velocity;

    //setpoint = -offset - position*0.010;
    setpoint = -offset + ps2offset;
    Serial.print("  sp=");
    Serial.print(setpoint);

    myPID.Compute();

    filteredOutput = 0.4*output + 0.6*filteredOutput;
    if ( abs(pitch) < 3 ) {
      //pwm = filteredOutput;
      pwm = output;
    } else {
      //pwm = filteredOutput;
      pwm = output;
    }

    setMotor_L(pwm+ps2offsetL);
    setMotor_R(pwm+ps2offsetR);

    printt && Serial.print("  pwm=");
    printt && Serial.print(pwm);

    if ( (pitch > 15) || (pitch < -15) ) {
      setMotor_L(0);
      setMotor_R(0);
    } else {
      setMotor_L(pwm+ps2offsetL);
      setMotor_R(pwm+ps2offsetR);
    }
    /*
    */

  }

  if (digitalRead(BUTTON_RESET_PID) == 0) {
    myPID.PIDReset();
  }


  // Get data from PS2 controller.
  ps2x.read_gamepad(false, vibrate);  //Read controller and set vibrate.

  /*
  if(ps2x.Button(PSB_START))         //TRUE while button is pressed.
    Serial.println("Start is being held");
  if(ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held");      
   */

  ps2offset = (128 - ps2x.Analog(PSS_LY)) / 128.0;
  ps2offsetL = -(128 - ps2x.Analog(PSS_RX)) / 6.0;
  ps2offsetR = -ps2offsetL;

  /*
  if ( ps2x.Analog(PSS_LY) < 100 ) {
    ps2offset = 2;
  } else if ( ps2x.Analog(PSS_LY) > 156 ) {
    ps2offset = -2;
  } else {
    ps2offset = 0;
  }
  */

  Serial.print("  off=");
  Serial.print(ps2offset);

  Serial.print("  ");
  Serial.print(ps2x.Analog(PSS_LY), DEC);
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LX), DEC); 
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RY), DEC); 
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RX), DEC); 
  /*
  */

  Serial.println("");


}
