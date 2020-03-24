// MPU-6050 LED Rainbow run
// By Igor Kosulin

// License Goodness
/* ============================================
  Adafruit NeoPixel library.
  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  ===============================================
*/

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg
  Updates available at: https://github.com/jrowberg/i2cdevlib
  ===============================================
*/

// Watchdog timer to reset the device if it freezes.
#include <avr/wdt.h>

#include "led_runner.h"
#include <ArduinoLog.h>

#define PIXELPIN       12        // Arduino pin connected to strip
#define NUMPIXELS      60       // Total number of RGB LEDs on strip
#define NUMSTRIPS      1       // Total number of RGB strips

#ifdef __AVR__
#include <avr/power.h>        // AVR Specific power library
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>

// MotionApps utilizes the "Digital Motion Processor" (DMP) on the MPU-6050
// to filter and fuse raw sensor data into useful quantities like quaternions,
// Euler angles, or Yaw/Pitch/Roll inertial angles
#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

// Class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5.0v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

// yaw/pitch/roll angles (in degrees) calculated from the quaternions
// coming from the FIFO. Note this also requires gravity vector
// calculations. Also note that yaw/pitch/roll angles suffer from gimbal
// lock (for more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quaternion;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

LedRunner* leds;

void setup() {
  // One watchdog timer will reset the device if it is unresponsive
  // for a second or more
  wdt_enable(WDTO_8S);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  // initialize device
  Log.trace(F(CR "Initializing I2C devices..." CR));
  mpu.initialize();

  // verify connection
  Log.trace(mpu.testConnection() ? F("MPU6050 connection successful" CR) : F("MPU6050 connection failed" CR));

  // load and configure the DMP
  Log.trace(F("Initializing DMP..." CR));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Log.trace(F("Enabling DMP..." CR));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Log.trace(F("Enabling interrupt detection (Arduino external interrupt 0)..." CR));

    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Log.notice(F("DMP ready! Waiting for first interrupt..." CR CR));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Log.error(F(CR));
    Log.error(F("DMP Initialization failed (code %d)" CR CR), devStatus);
  }
  leds = new LedRunner(NUMPIXELS, PIXELPIN);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // the program is alive...for now.
  wdt_reset();

  // if programming failed, don't try to do anything
  if (!dmpReady) {
    leds->update_leds(0);
    return;
  }

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
    Log.error(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &quaternion);
    mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
    float yaw_deg = ypr[0] * 180 / M_PI;
    float pitch_deg = ypr[1] * 180 / M_PI;
    float roll_deg = ypr[2] * 180 / M_PI;

    float angle = roll_deg;
    if( leds->update_leds(angle) ) Log.plotter(F(CR));
  } 
}
