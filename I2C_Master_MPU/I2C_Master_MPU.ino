union int_bytes {
  uint8_t b[4];
  struct yp {
    uint16_t y;
    uint16_t p;
  } yp;
} int_bytes;

#define MPU6050_ACCEL_OFFSET_X -1660
#define MPU6050_ACCEL_OFFSET_Y -414
#define MPU6050_ACCEL_OFFSET_Z 1663
#define MPU6050_GYRO_OFFSET_X  109
#define MPU6050_GYRO_OFFSET_Y  124
#define MPU6050_GYRO_OFFSET_Z  86

#define DEBUG_OUT

// Calibration variables
#define CALIBRATION_PIN 14
float yaw_offset = 0;
float pitch_offset = 0;

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Teensy's digital pin 2.

   HARDWARE SETUP:
     MPU6050 Breakout ----- Teensy 3.2
     3.3V ----------------- 3.3V
     GND ------------------ GND
     SDA ------------------ A4/pin 18
     SCL ------------------ A5/pin 19
     INT ------------------ Digital Pin 2 (Teensy)
   ========================================================================= */
/* =========================================================================
   NOTE: For calibration, connect a push-to-make-contact push button to ground and
   $(CALIBRATION_PIN) (pin 14). Push button to reset the camera to initial position.
   Issue: when yaw drifts over or is pushed to the limit (180) by calibration,
   servo range is strongly limited in one direction and flips around.

   ========================================================================= */

#define LED_PIN 13
bool blinkState = false;

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
  // Set the calibration button to react when LOW is applied to the pin
  pinMode(CALIBRATION_PIN, INPUT);
  digitalWrite(CALIBRATION_PIN, HIGH);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
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
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));

    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .

  }
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

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (digitalRead(CALIBRATION_PIN) == LOW) {
      yaw_offset = ypr[0];
      pitch_offset = ypr[1];
    }

    // yaw degrees
    // Add M_PI to get positive values (ypr[0] element of (-M_PI, M_PI)).
    // Angle in degree is ratio of reading to max reading * 180 where max reading: 2 * M_PI
    int yaw_value = (int) 180 - (ypr[0] + M_PI - yaw_offset) * 180 / (M_PI * 2);
    /*yaw_value = yaw_value > 180.0 ? 180.0 : yaw_value;
      yaw_value = yaw_value < 0.0 ? 0.0 : yaw_value;*/
    // pitch degrees
    // Add 90 to start at horizontal, flat position
    // Angle in degree is ratio of reading to max reading * 180 where max reading: 2 * M_PI
    int pitch_value = (int) (90 + ypr[1] * 180 / M_PI - pitch_offset);

    // Send the yaw/pitch/roll euler angles (in degrees) over serial connection
    // calculated from the quaternions in the FIFO.
    // Note this also requires gravity vector calculations.
    // Also note that yaw/pitch/roll angles suffer from gimbal lock
    // (for more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

#ifdef DEBUG_OUT
    Serial.print("ypr\t");
    Serial.print(yaw_value);
    Serial.print("\t");
    Serial.println(pitch_value);
#endif

    int_bytes.yp.p = yaw_value;
    int_bytes.yp.y = pitch_value;

    Wire.beginTransmission(12);
    for (int i = 0; i < 4; i++) {
      Wire.write(int_bytes.b[i]);
    }

    Wire.endTransmission(12);

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

