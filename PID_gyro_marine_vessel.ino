#include "I2Cdev.h"
#include "PID_v1.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Declare PID
double setpoint = 0;
double Kp = 30; //Set this first
double Kd = 0; //Set this second
double Ki = 0; //Finally set this
double angel;
double input, output;
double yaw_angle;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(19200);
  pinMode(2, INPUT);
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
  }

  //setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-30, 30); //output

  //Initialise the Motor output pins
  pinMode (7, OUTPUT); //A
  pinMode (8, OUTPUT); //B
  pinMode (9, OUTPUT); //EN pin
  pinMode (5, OUTPUT); //PWM pin
  //pinMode (cspin, OUTPUT); //CS pin

  //By default turn off both the motors
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(5,HIGH);
  //analogWrite(5,HIGH);
}

void loop() {
  // current = analogRead(cspin);
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
   //no mpu data - performing PID calculations and output to motors
   pid.Compute();
   //Print the value of Input and Output on serial monitor to check how it is working.

    if (input <1.5 && input >-1.5){ //-9 ~ 9 angle
      Stop();
    }
    else if (input >=1.5 ) { //If the Bot is falling //input is angle?
        Forward(); //Rotate the wheels forward
    }
    else if (input <=-1.5){ //Falling towards back
        Backward(); //Rotate the wheels backward
    }
    else{ //If Bot not falling
      Stop(); //Hold the wheels still
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 16384)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

    //Compute PID
    input = (ypr[1]/M_PI)*180;
    yaw_angle= ((ypr[2]/M_PI)*180);
     
     //Experiment 1&2
     //Serial.print(input); Serial.print(" Roll_angle ");  Serial.print(output); Serial.println(" PWM "); 
  
     //Experiment 3
     Serial.print(input); Serial.print(" Roll_angle "); Serial.print(yaw_angle); Serial.println(" Yaw_angle ");
  }
}

void Forward() //Rotate the wheel forward
{
  digitalWrite (9, 1); //EN pin
  digitalWrite (7, 1); //A
  digitalWrite (8, 0); //B
  analogWrite (5, -1*output); //PWM pin
  delay(10);
}

void Backward() //Rotate the wheel Backward
{
  digitalWrite (9, 1); //EN pin
  digitalWrite (7, 0); //A
  digitalWrite (8, 1); //B
  analogWrite (5, output); //PWM pin
  delay(10);
}

void Stop() //Stop both the wheels
{
  digitalWrite (9, 0); //EN pin
  analogWrite(7, 0);
  analogWrite(8, 0);
}
