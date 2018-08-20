#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define M_PI 3.14
#include "mbed.h"
#include "ppo-test.hpp"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

bool stop = false;
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
const float maxspeed = 5.0;     // kHz

InterruptIn sw(PE_15);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;
Serial pc(USBTX, USBRX, 115200);
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float yprt[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t data[3];
struct ypr {
    float roll;
    float pitch;
    float gyrox;
    float gyroy;
};
ypr x_d;
#define MOTOR_EN_PIN        PF_12   // D8
#define MOTOR_SPD_R_PIN     PD_14   // D10
#define MOTOR_SPD_L_PIN     PD_15   // D9
#define MOROR_DIR_R_PIN     PF_13   // D7
#define MOROR_DIR_L_PIN     PE_9    // D6

#define MOTOR_CW            false
#define MOTOR_CCW           true
bool prev_dir = false;
bool curr_dir = false;

DigitalOut MOTOR_En(MOTOR_EN_PIN);
DigitalOut MOTOR_DIR_R(MOROR_DIR_R_PIN);
DigitalOut MOTOR_DIR_L(MOROR_DIR_L_PIN);
PwmOut MOTOR_SPD_R(MOTOR_SPD_R_PIN);
PwmOut MOTOR_SPD_L(MOTOR_SPD_L_PIN);

Timer timer;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setMotors(unsigned int TmpL)
{
    // Need set period first then set pwm, it's a trap !!!
    /*LSpd = LSpd * 4500;
    unsigned int TmpL = abs((int)(1.0 / LSpd));   
    TmpL = 1000000 * TmpL;

     if (TmpL < 223 && TmpL >= 0) {
        TmpL = 223;
    } else if (TmpL >= 37650) {
        TmpL = 37650;
    } 
    //FREQ_CHECK.period_us(TmpL);*/
    MOTOR_SPD_L.period_us(TmpL);
    MOTOR_SPD_R.period_us(TmpL);

    MOTOR_SPD_R = 0.5;          
    MOTOR_SPD_L = 0.5;

    /*if(LSpd >= 0.0)  {    MOTOR_DIR_L = MOTOR_CW;  
                          MOTOR_DIR_R = MOTOR_CW;
                          prev_dir = true;
    } else {
                   MOTOR_DIR_L = MOTOR_CCW;  
                   MOTOR_DIR_R = MOTOR_CCW;  
                   prev_dir = false;
    }*/
}
void motorInit()
{
    //FREQ_CHECK.period_us(1.0/maxspeed*1000.0);  // Set Frequency
    MOTOR_SPD_R.period_us(1.0/maxspeed*1000.0);
    MOTOR_SPD_L.period_us(1.0/maxspeed*1000.0);

    //FREQ_CHECK = 0.5;           // Set PWM Duty Cycle 50%
    MOTOR_SPD_R = 0.5;          
    MOTOR_SPD_L = 0.5;

    MOTOR_DIR_R = MOTOR_CW;     // CW: true, CCW: false
    MOTOR_DIR_L = MOTOR_CW;
}

void init() {
    motorInit();
    //MOTOR_En = true;
}
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

void dosomething() {
    timer.start();
    float sensorRaw[4] = {0};
    sensorRaw[0] = x_d.roll;
    sensorRaw[1] = x_d.pitch;
    sensorRaw[2] = x_d.gyrox * 250;
    sensorRaw[3] = x_d.gyroy * 250; 
    float nnCmd = nn(sensorRaw);
    if (nnCmd > 0.3) {
        nnCmd = 0.3f;
    } else if (nnCmd < -0.3){
        nnCmd = -0.3f;
    }
            timer.stop();
            int res = timer.read_us();
            pc.printf("%d ", res);
    setMotors(nnCmd);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
//        pc.printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(yprt, &q, &gravity);
            mpu.dmpGetGyro(data, fifoBuffer);
            int16_t acc[3];
            mpu.dmpGetAccel(acc, fifoBuffer);
            float a_datax = abs((float)acc[0] / 16384.0f);
            float a_datay = abs((float)acc[1] / 16384.0f);
            float a_dataz = abs((float)acc[2] / 16384.0f);
            x_d.pitch = yprt[1];
            x_d.roll = yprt[2];
            x_d.gyrox = (float)data[0] / 16384.0f;
            x_d.gyroy = (float)data[1] / 16384.0f;
//            pc.printf("d:%7.2f  b:%d \r\n", a_datax, timer.read_us());
   //         dosomething();
            //buff.append(x);
            float res = a_datax * a_datax + a_datay * a_datay + a_dataz * a_dataz;
 //           if (abs(a_datay) > 0.4) {
 //           timer.stop();
            pc.printf("d:%7.5f \r\n", res);
 //           stop = true;
 //           }
    }
}

void rise_handler() {
    dmpReady = true;
}

int main() {
    stop = false;
    setup();
    init();



            
    printf("run......\r\n");
    loop();
//    setMotors(200);
//    wait(10);
    printf("run 0.1 for 10 s....\r\n");
    //t.start(callback(&queue, &EventQueue::dispatch_forever));
    //sw.rise(rise_handler);
    //sw.fall(queue.event(loop));
    printf("start....\r\n");
    MOTOR_En = false;
 //   timer.start();
//    setMotors(200);
    while (!stop) {
        loop();
    //pc.printf("goo....\r\n");
    }
    wait(5);
    MOTOR_En = false;

    return 0;
}

