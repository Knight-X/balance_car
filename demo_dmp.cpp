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

bool start;
bool terminal;
int steps;
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
unsigned int motorg = 0;

InterruptIn sw(PE_15);
InterruptIn interrupt_button(PC_13);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue pqueue(8 * EVENTS_EVENT_SIZE);
Thread t;
Thread p;
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
};
struct nn_input {
    float roll;
    float pitch;
    float motor;
    float acc;
    float roll2;
    float pitch2;
    float motor2;
    float acc2;
    float roll3;
    float pitch3;
    float motor3;
};
ypr x_d;
nn_input nn_buf;
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


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setMotors(float LSpd)
{
    bool cw = true;
    // Need set period first then set pwm, it's a trap !!!
    if (LSpd >= 0.0) {
        cw = true;
    } else {
        cw = false;
    }
    LSpd = LSpd * 4500;
    LSpd = abs(1.0 / LSpd);   
    LSpd = 1000000 * LSpd;
    unsigned int TmpL = (unsigned int)LSpd;   
    motorg = TmpL;

     if (TmpL < 223 && TmpL >= 0) {
        TmpL = 223;
    } else if (TmpL >= 37650) {
        TmpL = 37650;
    } 
    //FREQ_CHECK.period_us(TmpL);
    MOTOR_SPD_L.period_us(TmpL);
    MOTOR_SPD_R.period_us(TmpL);

    MOTOR_SPD_R = 0.5;          
    MOTOR_SPD_L = 0.5;

    if(cw)  {    MOTOR_DIR_L = MOTOR_CW;  
                          MOTOR_DIR_R = MOTOR_CW;
                          prev_dir = true;
    } else {
                   MOTOR_DIR_L = MOTOR_CCW;  
                   MOTOR_DIR_R = MOTOR_CCW;  
                   prev_dir = false;
    }
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
    start = false;
    terminal = false;
    steps = 0;
    motorInit();
    MOTOR_En = true;
    nn_buf.roll = 0.0;
    nn_buf.pitch = 0.0;
    nn_buf.motor = 0.0;
    nn_buf.roll2 = 0.0;
    nn_buf.pitch2 = 0.0;
    nn_buf.motor2 = 0.0;
    nn_buf.acc = 0.0;
    nn_buf.acc2 = 0.0;
    nn_buf.motor3 = 0.0;
    nn_buf.roll3 = 0.0;
    nn_buf.pitch3 = 0.0;
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

void status(nn_input);
void dosomething() {
    float sensorRaw[8] = {0.0};
    x_d.roll = x_d.roll - 0.08;
    x_d.roll = x_d.roll / 1.5707963;
    x_d.pitch = x_d.pitch / 1.5707963;
    sensorRaw[0] = x_d.roll;
    sensorRaw[1] = x_d.pitch;
    sensorRaw[2] = nn_buf.roll2;
    sensorRaw[3] = nn_buf.pitch2; 
    sensorRaw[4] = nn_buf.motor;
    sensorRaw[5] = nn_buf.motor; 
    sensorRaw[6] = nn_buf.motor2;
    sensorRaw[7] = nn_buf.motor2; 

    float nnCmd = nn(sensorRaw);
/*    if (nnCmd > 0.3) {
        nnCmd = 0.3f;
    } else if (nnCmd < -0.3){
        nnCmd = -0.3f;
    }*/
    setMotors(nnCmd);
    nn_buf.motor3 = nn_buf.motor2;
    nn_buf.roll3 = nn_buf.roll2;
    nn_buf.pitch3 = nn_buf.pitch2;
    nn_buf.acc2 = nn_buf.acc;
    nn_buf.motor2 = nn_buf.motor;
    nn_buf.roll2 = nn_buf.roll;
    nn_buf.pitch2 = nn_buf.pitch;
    nn_buf.motor = nnCmd;
    nn_buf.roll = x_d.roll;
    nn_buf.pitch = x_d.pitch;
    pqueue.call(status, nn_buf);
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
//        int start_time = ss.read_ms();
//        ss.start();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        

            // display Euler angles in degrees
            if (!terminal) {
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(yprt, &q, &gravity);
              mpu.dmpGetGyro(data, fifoBuffer);
              x_d.pitch = yprt[1];
              x_d.roll = yprt[2];
              dosomething();
              if (start) {
                steps = steps + 1;
              }
            //buff.append(x);
//            ss.stop();
//            printf(" %d \r\n", ss.read_ms() - start_time);
              if (start && (abs(x_d.roll) > 0.3925 || abs(x_d.pitch) > 0.3925)) {
                terminal = true;
                start = false;
              }
            }
    }
}

void rise_handler() {
    dmpReady = true;
}
void status(nn_input b) {
    if (start) {
     pc.printf("roll: %7.2f, pitch: %7.2f , nn: %7.2f, roll2: %7.2f, pitch2: %7.2f, nn2: %7.2f, roll3: %7.2f, pitch3: %7.2f, nn3: %7.2f \r\n", b.roll, b.pitch, b.motor, b.roll2, b.pitch2, b.motor2, b.roll3, b.pitch3, b.motor3);
   } else if (terminal) {
     pc.printf("steps: %d\r\n", steps);
  } else {
       pc.printf("not started yet....\r\n");
   }
}

void start_timing() {
      start = true;
}
int main() {
    setup();
    init();
    wait_ms(1000);
    interrupt_button.fall(&start_timing);
    t.start(callback(&queue, &EventQueue::dispatch_forever));
    p.start(callback(&pqueue, &EventQueue::dispatch_forever));
    p.set_priority(osPriorityLow);



    sw.rise(rise_handler);
    sw.fall(queue.event(loop));
            
    for (;;) {

//            pc.printf("motor: %d \r\n", motorg);
//            pc.printf("roll: %7.2f, pitch: %7.2f \r\n", x_d.roll, x_d.pitch);
        wait_ms(2);
        if (terminal) {
            break;
        }
    }

    return 0;
}

