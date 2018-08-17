#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define M_PI 3.14
#include "mbed.h"
#include "SensorQueue.hpp"
#include "ppo-test.hpp"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Serial pc(USBTX, USBRX, 115200);
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float yprt[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
struct ypr {
    float pitch;
    float roll;
};
#define MOTOR_EN_PIN        PF_12   // D8
#define MOTOR_SPD_R_PIN     PD_14   // D10
#define MOTOR_SPD_L_PIN     PD_15   // D9
#define MOROR_DIR_R_PIN     PF_13   // D7
#define MOROR_DIR_L_PIN     PE_9    // D6
//#define FREQ_CHECK_PIN      PE_11   // D5

#define MOTOR_CW            false
#define MOTOR_CCW           true
bool prev_dir = false;
bool curr_dir = false;

DigitalOut MOTOR_En(MOTOR_EN_PIN);
DigitalOut MOTOR_DIR_R(MOROR_DIR_R_PIN);
DigitalOut MOTOR_DIR_L(MOROR_DIR_L_PIN);
PwmOut MOTOR_SPD_R(MOTOR_SPD_R_PIN);
PwmOut MOTOR_SPD_L(MOTOR_SPD_L_PIN);
SensorQueue<ypr> buff(160, 32, 2);
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize device
    pc.printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    pc.printf("Testing device connections...\n");
    pc.printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    pc.printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        pc.printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        pc.printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        pc.printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            pc.printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            pc.printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(yprt, &q, &gravity);
            ypr x;
            x.pitch = yprt[1];
            x.roll = yprt[2];
            buff.append(x);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            pc.printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            pc.printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
        //pc.printf("\n");
    }
}

void setMotors(float LSpd)
{
    // Need set period first then set pwm, it's a trap !!!
    LSpd = LSpd * 4500;
    //RSpd = RSpd * 3200.0;
    unsigned int TmpL = abs((int)(1.0 / LSpd));   
    //int TmpR = abs((int)(1.0 / RSpd));    
    // SpdL_print = TmpL;
/*    if (TmpL > -400 && TmpL < 0) {
        TmpL = -400;
    } else if (TmpL < 400 && TmpL >= 0) {
        TmpL = 400;
    } else if (TmpL >= 3125) {
        TmpL = 3125;
    } else if (TmpL <= -3125) {
        TmpL = -3125;
    }*/
    TmpL = 1000000 * TmpL;
     if (TmpL < 223 && TmpL >= 0) {
        TmpL = 223;
    } else if (TmpL >= 37650) {
        TmpL = 37650;
    } 
    /*if (TmpR > -400 && TmpR < 0) {
        TmpR = -400;
    } else if (TmpR < 400 && TmpR >= 0) {
        TmpR = 400;
    } else if (TmpR >= 3125) {
        TmpR = 3125;
    } else if (TmpR <= -3125) {
        TmpR = -3125;
    }*/
    //FREQ_CHECK.period_us(TmpL);
    MOTOR_SPD_L.period_us(TmpL);
    MOTOR_SPD_R.period_us(TmpL);

    //FREQ_CHECK = 0.5;               // Set PWM Duty Cycle 50%
    MOTOR_SPD_R = 0.5;          
    MOTOR_SPD_L = 0.5;

    if (LSpd >= 0.0) {
        curr_dir = true;
    } else {
        curr_dir = false;
    }

    if (curr_dir != prev_dir) {
    if(LSpd >= 0.0)  {    MOTOR_DIR_L = MOTOR_CW;  
                          MOTOR_DIR_R = MOTOR_CW;
                          prev_dir = true;
    } else {
                   MOTOR_DIR_L = MOTOR_CCW;  
                   MOTOR_DIR_R = MOTOR_CCW;  
                   prev_dir = false;
    }
    }
}

void dosomething() {
    pc.printf("start....\r\n");
    ypr* tmp = (ypr*) malloc(sizeof(ypr) * 160);
    pc.printf("before copy....\r\n");
    buff.copyTo(tmp);
    pc.printf("pr  %7.2f %7.2f \t\n", tmp[0].pitch * 180/M_PI, tmp[0].roll * 180/M_PI);
    float sensorRaw[4] = {0};
    sensorRaw[0] = tmp[0].pitch;
    sensorRaw[1] = tmp[0].roll;
    sensorRaw[2] = 0.5;
    sensorRaw[3] = 0.3; 
    float nnCmd = nn(sensorRaw);
    setMotors(nnCmd);
    free(tmp);

    
}
int main() {
    setup();
    wait_ms(1000);
    buff.setCallBack(dosomething);
    for (;;) {
        loop();
    }

    return 0;
}

