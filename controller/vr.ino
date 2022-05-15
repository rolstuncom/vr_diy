#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;
bool dmpReady = false; 
uint8_t mpuIntStatus;  
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
String mass;

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#define GRIP_BTN  0x0001
#define THUMB_BTN 0x0002
#define A_BTN   0x0004
#define B_BTN   0x0008
#define MENU_BTN  0x0010
#define SYS_BTN   0x0020
const byte TriggerBtnPin = 3;
const byte GripBtnPin = 4;
const byte MenuBtnPin = 6;
const byte SystemBtnPin = 7;
const byte joy_x = 1;
const byte joy_y = 2;
const byte joy_button = 5;
const byte VibroMotorPin = 9;
int motor = 0;
float ctrl[12];


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
  
    pinMode(TriggerBtnPin, INPUT_PULLUP);
    pinMode(GripBtnPin, INPUT_PULLUP);
    pinMode(SystemBtnPin, INPUT_PULLUP);
    pinMode(MenuBtnPin, INPUT_PULLUP);
    pinMode(joy_x, INPUT);   
    pinMode(joy_y, INPUT);
    pinMode(joy_button, INPUT);
    pinMode(VibroMotorPin, INPUT);
    while (!Serial); 
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(16);
    mpu.setYGyroOffset(78);
    mpu.setZGyroOffset(9);
    mpu.setXAccelOffset(-3518);
    mpu.setYAccelOffset(1334); 
    mpu.setZAccelOffset(3066); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
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
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
  
        fifoCount -= packetSize;
mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            ctrl[3] = ypr[0] * 180/M_PI;
            ctrl[4] = ypr[2] * 180/M_PI * -1;
            ctrl[5] = ypr[1] * 180/M_PI;
            ctrl[8] = 0; //Buttons
            ctrl[9] = 0; //Trigger
            ctrl[10] = 0; //ThumbX
            ctrl[11] = 0; //ThumbY
            int CtrlButtons = 0;
            if (digitalRead(GripBtnPin) == LOW)
              CtrlButtons |= GRIP_BTN;

            if (digitalRead(joy_button) == LOW)
              CtrlButtons |= THUMB_BTN; 

            if (digitalRead(MenuBtnPin) == LOW)
              CtrlButtons |= MENU_BTN;

            if (digitalRead(SystemBtnPin) == LOW)
              CtrlButtons |= SYS_BTN;

            //Buttons
             ctrl[8] = CtrlButtons;

            //Trigger
            if (digitalRead(TriggerBtnPin) == LOW)
              ctrl[9] = 1;
ctrl[11] = map(analogRead(1),740,1023,0,1023);
ctrl[10] = map(analogRead(2),740,1023,0,1023); 

Serial.print(ctrl[3]);
Serial.print(',');
Serial.print(ctrl[4]);
Serial.print(',');
Serial.print(ctrl[5]);
Serial.print(',');
Serial.print(ctrl[8]-2);
Serial.print(',');
Serial.println(ctrl[9]);

}
}
