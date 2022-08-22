/* Efncoder library combined with l298n
 */

#include <Encoder.h>
#include <PID_v2.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
int counter = 0;
// Motor B connections
int enB = 5;
int in4 = 7;
int in3 = 8;

int enA = 6;
int in1 = 11;
int in2 = 12;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//Encoder myEnc(2,10);
Encoder myEnc(3,9);
//   avoid using pins with LEDs aokmttached

double pos_Kp = 975.0; // Proportional Gain (TBD) 1500
double pos_Kd = 600.0; // Derivative Gain (TBD)     500
double pos_Ki = 200.0; // Integral Gain (TBD)
double pos_pidOUT = 0; 
double current_pos;
double desired_pos = -0.02;
double position_delta;
double alpha = 0.9; // Low pass filter scaling

// MPU Control/Status (Don't know much about this just copying from chillibasket
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
bool dmpReady = false;           // Set true if DMP init was successful
uint8_t devStatus;              // Return status after device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer
// This is copied as well but I will leave uncommented until I figure out how it works
// Orientation/Motion
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Quaternion q;                   // [w, x, y, z]       Quaternion Container
VectorFloat gravity;             // [x, y, z]            Gravity Vector
int16_t gyro[3];                // [x, y, z]            Gyro Vector
float ypr[3];                   // [yaw, pitch, roll]   Yaw/Pitch/Roll & gravity vector
float averagepitch[50];         // Used for averaging pitch value

MPU6050 mpu;

bool pinstate = false;

// Creat PID object 
PID pos_PID(&current_pos, &pos_pidOUT, &desired_pos, pos_Kp, pos_Ki, pos_Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // Set timer1 OC to trigger at 100 Hz
  cli();
  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 7812.50 Hz
  
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;
  TCCR1B = 0b00000100;        // set the divider to 256
  TIMSK1 |= B00000010;
  OCR1A = 250;               // OC after 625 counts (100 Hz)
  sei();
  
  //Serial.begin(57600);
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  //Serial.println("Basic Encoder Test:");
  // Set all the motor control pins to outputs
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  
  // Turn off motors - Initial state
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  

  // setup pid
  pos_PID.SetMode(MANUAL); // Turn on PID controller
  pos_PID.SetOutputLimits(-255,255);
  pos_PID.SetSampleTime(0);
  /* * * * * * * * * * * * * * * * * * * *
     * IMPORTANT!
     * Supply your own MPU6050 offsets here
     * Otherwise robot will not balance properly.
     * * * * * * * * * * * * * * * * * * * */
    mpu.setXGyroOffset(-11);
    mpu.setYGyroOffset(15);
    mpu.setZGyroOffset(33);
    mpu.setXAccelOffset(-3352);
    mpu.setYAccelOffset(185);
    mpu.setZAccelOffset(1340);

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
       //ΩΩ vsg634 Serial.println("Enabling DMP");
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    pos_PID.SetMode(AUTOMATIC); // Turn on PID controller
    digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH);
   digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
    
}

void accelgyroData(){
    
    // Reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println("Warning - FIFO Overflowing!");

    // otherwise, check for DMP data ready interrupt (this should happen exactly once per loop: 100Hz)
    } else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length, should be less than 1-2ms, if any!
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get sensor data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.resetFIFO();

        //Serial.print(ypr[1]*180/M_PI);
        //Serial.print(" - ");
        //Serial.println(ypr[0]);
    }
}

long oldPosition  = -999;

void loop() {
  /*
  const double input = analogRead(PIN_INPUT);
  const double output = myPID.Run(input);
  analogWrite(PIN_OUTPUT, output);
  */
  accelgyroData();
  //current_pos = ypr[2];// * 180/M_PI;
  //pos_PID.Compute();
  //Serial.println(current_pos);
  //motor_control();
  
}

// This function lets you control speed of the motors
void speedControl() {
  // Turn on motors
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  // Accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++) {
    analogWrite(enB, i);
    analogWrite(enA, i);
    printEncoder();
    delay(20);
  }

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Decelerate from maximum speed to zero
  for (int i = 255; i >= 0; --i) {
    analogWrite(enB, i);
    analogWrite(enA, i);
    printEncoder();
    delay(20);
  }
  
  // Now turn off motors
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// This function lets you control speed of the motors
void motor_control() {
  // Turn on motors
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, HIGH);
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, HIGH);

  if(pos_pidOUT < 0){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }else{
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

   if(pos_pidOUT != 0){
    analogWrite(enB, min(abs(int(pos_pidOUT)) + 20.0, 255));
    analogWrite(enA, min(abs(int(pos_pidOUT)) + 20.0, 255));
   }
   if(abs(pos_pidOUT)      >= 255){
      digitalWrite(LED_BUILTIN,HIGH);
    }else{
      digitalWrite(LED_BUILTIN,LOW);  
    }
}


void printEncoder() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    position_delta = newPosition - oldPosition;
    oldPosition = newPosition;
    //Serial.println(newPosition);
    //Serial.println(position_delta);
  }
}
double low_pass_filter(double alpha_val, double prev_val, double raw_val){
  return alpha_val*raw_val + (1 - alpha_val)*prev_val;
}

//With the settings above, this IRS will trigger each 500ms.
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  current_pos = low_pass_filter(alpha, current_pos, ypr[2]);// * 180/M_PI;
  pos_PID.Compute();
  motor_control();
}
