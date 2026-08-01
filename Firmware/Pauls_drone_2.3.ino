#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "DroneController";
const char* password = "123456789";

WebServer server(80);

// Control parameters (1000-2000 range)
int leftX = 1500, leftY = 1500;
int rightX = 1500, rightY = 1500;
bool emergency = false;

// Direction configuration (true = normal, false = reversed)
bool leftXDir = true;
bool leftYDir = true;
bool rightXDir = true;
bool rightYDir = true;

//Uncomment only one receiver type
#define USE_WIFI
//#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G


//REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <ESP32Servo.h>  //Commanding any extra actuators
#include <VL53L1X.h> //Reading values from the lidar

VL53L1X lidar; //Creating the lidar object


#if defined USE_MPU6050_I2C
  #include "src/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#else
  #error No MPU defined... 
#endif





//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif




//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1500; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1000; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
// unsigned long channel_6_fs = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.17;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_altitude = 0.3;
float B_altitude_des = 0.3;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.09;
float AccErrorY = 0.03;
float AccErrorZ = 0.03;
float GyroErrorX = -3.38;
float GyroErrorY = 1.82;
float GyroErrorZ = -0.38;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float i_limit_alt = 25.0;  
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec
float max_alt = 3000;


float Kp_roll_angle = 0.5;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.04;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.11;   //Roll D-gain - angle mode 
float Kp_pitch_angle = 0.0;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.00;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.00;  //Pitch D-gain - angle mode 

float Kp_yaw = 0.2;           //Yaw P-gain
float Ki_yaw = 0.0;          //Yaw I-gain
float Kd_yaw = 0.0;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)


float Kp_alt = 0.02; 
float Ki_alt = 0.02;
float Kd_alt = -0.01;


const int ch1Pin = 4; //throttle
const int ch2Pin = 4; //ail
const int ch3Pin = 4; //ele
const int ch4Pin = 4; //rudd
const int ch5Pin = 4; //gear (throttle cut)
// const int ch6Pin = 1; //aux1 (free aux channel)
//PWM servo or ESC outputs:
const int servo1Pin = 13;
const int servo2Pin = 12;
const int servo3Pin = 14;
const int servo4Pin = 27;
Servo servo1;  //Create servo objects to control a servo or ESC with PWM
Servo servo2;
Servo servo3;
Servo servo4;



//Lidar:
float altitude;
float altitude_prev;
float altitude_hold_height;
bool timeout_state = false;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 50; // 50ms timing budget

//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;


//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des, alt_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_prev, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_prev, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
float error_alt, error_alt_prev, alt_des_prev, integral_alt, integral_alt_prev, derivative_alt, alt_PID = 0;

//Mixer
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//Flight status
bool armedFly = false;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  Wire.begin();
  delay(500);
  
  //Initialize all pins
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  servo1.attach(servo1Pin, 1000, 2000); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 1000, 2000);
  servo3.attach(servo3Pin, 1000, 2000);
  servo4.attach(servo4Pin, 1000, 2000);

  delay(5);

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;

  //Initialize Lidar communication
  //lidar_init();

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);

  delay(5);

  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

}



                                             
void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printAltData();
  printRollPitchYawAlt();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
  printPropIntegralDerivPitch();

  // Get arming status
  armedStatus(); //Check if the throttle cut is off and throttle is low.

  //Get vehicle state
  //getAltdata(); //Pulls raw lidar values
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits
  
  //PID Controller - SELECT WHAT APPLIES:
  //controlHeight(); //Stabilize on altitude setpoint
  controlANGLE(); //Stabilize on angle setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  servo1.write(s1_command_PWM); //Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);

    
  //Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

//                  ALTITUDE CONTROL FUNCTIONS
void lidar_init(){
  lidar.setTimeout(500);
  if (!lidar.init()) {
    Serial.println("VL53L1X not detected");
    while (1);
  }

  lidar.setDistanceMode(VL53L1X::Long);
  lidar.setMeasurementTimingBudget(50000); // 50 ms
  lidar.startContinuous(50); // continuous readings every 50ms
}

void getAltdata(){
  unsigned long now = millis();
  if (now - lastReadTime >= readInterval) {
    lastReadTime = now;

    if (lidar.dataReady()) {
      lidar.read();
      altitude = lidar.ranging_data.range_mm;
      processAltdata();
    }
  }
}

void processAltdata(){
  float pitchRad = abs(pitch_IMU - 90) * DEG_TO_RAD;
  float rollRad  = abs(roll_IMU)  * DEG_TO_RAD;
  altitude = altitude * cos(pitchRad) * cos(rollRad);
  altitude = abs(altitude);
  altitude = (1 - B_altitude)*altitude_prev + B_altitude*altitude;
  altitude_prev = altitude;
}


void controlMixer() {
  s1_command_scaled = thro_des - roll_PID - yaw_PID - pitch_PID;//front left
  s2_command_scaled = thro_des + roll_PID + yaw_PID - pitch_PID;//front right
  s3_command_scaled = thro_des + roll_PID - yaw_PID + pitch_PID;//back right
  s4_command_scaled = thro_des - roll_PID + yaw_PID + pitch_PID;//back left
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_3_pwm < 1050)) {
    armedFly = true;
  }
}

void IMUinit() {
  #if defined USE_MPU6050_I2C
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
    mpu6050.initialize();
    
  //  if (mpu6050.testConnection() == false) {
  //    Serial.println("MPU6050 initialization unsuccessful");
  //    Serial.println("Check MPU6050 wiring or try cycling power");
  //    while(1) {}
  //  }

    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
  #endif
}

void getIMUdata() {
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

}

void calculate_IMU_error() {
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}


void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  pitch_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  roll_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void getDesState() {
  thro_des = (channel_3_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_1_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5


  alt_des = thro_des * max_alt;
  alt_des = (1 - B_altitude_des)*alt_des_prev + B_altitude_des*alt_des;
  alt_des_prev = alt_des;
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  alt_des = constrain(alt_des, 0.0,max_alt);
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = (constrain(pitch_des, -1.0, 1.0)* -maxPitch); //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)* -maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlHeight(){
    //Height
  error_alt = alt_des - altitude;
  integral_alt = integral_alt_prev + error_alt*dt;
  if (channel_3_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_alt = 0;
  }
  integral_alt = constrain(integral_alt, -i_limit_alt, i_limit_alt); //Saturate integrator to prevent unsafe buildup
  derivative_alt = (error_alt - error_alt_prev)/dt;
  alt_PID = 0.01*(Kp_alt*error_alt + Ki_alt*integral_alt - Kd_alt*derivative_alt); //Scaled by .01 to bring within -1 to 1 range
  //update altitude variables
  error_alt_prev = error_alt;
  integral_alt_prev = integral_alt;
}

void controlANGLE() {

  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_3_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroY;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_3_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = -GyroX;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_3_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}


void scaleCommands() {
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);

}

void getCommands() {
  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
  #elif defined USE_WIFI
    server.handleClient();
    channel_1_pwm = rightX;
    channel_2_pwm = rightY;
    channel_3_pwm = leftY;
    channel_4_pwm = leftX;
    channel_5_pwm = map(emergency,0,1,1000,2000);
    
  #elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;  
      float bias  = 895.0; 
      channel_1_pwm = sbusChannels[0] * scale + bias;
      channel_2_pwm = sbusChannels[1] * scale + bias;
      channel_3_pwm = sbusChannels[2] * scale + bias;
      channel_4_pwm = sbusChannels[3] * scale + bias;
    }
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
  }
}


void calibrateESCs() {
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(10, HIGH); //LED on to indicate we are not in main loop

      getCommands(); //Pulls current available radio commands
      failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      
      s1_command_scaled = thro_des;
      s2_command_scaled = thro_des;
      s3_command_scaled = thro_des;
      s4_command_scaled = thro_des;
      scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
    
      //throttleCut(); //Directly sets motor commands to low based on state of ch5
      
      servo1.write(s1_command_PWM); 
      servo2.write(s2_command_PWM);
      servo3.write(s3_command_PWM);
      servo4.write(s4_command_PWM);

      //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}



void throttleCut() {
  if ((channel_5_pwm > 1500) || (armedFly == false)) {
    armedFly = false;

    //Uncomment if using servo PWM variables to control motor ESCs
    s1_command_PWM = 0;
    s2_command_PWM = 0;
    s3_command_PWM = 0;
    s4_command_PWM = 0;
  }
}


void loopRate(int freq) {
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(10, blinkAlternate); 
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(10, LOW);
    delay(downTime);
    digitalWrite(10, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("  thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.print(yaw_des);
    Serial.print(F(" alt_des:"));
    Serial.print(alt_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}
void printAltData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Altitude:"));
    Serial.println(altitude);
  }
}


void printRollPitchYawAlt() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("  roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.print(yaw_IMU);
    Serial.print(F(" alt:"));
    Serial.print(altitude);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.print(yaw_PID);
    Serial.print(F(" alt_PID:"));
    Serial.print(alt_PID);
  }
}



void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("  s1_command:"));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command:"));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command:"));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command:"));
    Serial.print(s4_command_PWM);
    Serial.print(F(" s5_command:"));
    Serial.print(s5_command_PWM);
    Serial.print(F(" s6_command:"));
    Serial.print(s6_command_PWM);
  }
}

void printPropIntegralDerivPitch() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("  prop:"));
    Serial.print(error_yaw);
    Serial.print(F(" integral:"));
    Serial.print(integral_yaw);
    Serial.print(F(" derivative:"));
    Serial.println(derivative_yaw);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("  dt:"));
    Serial.print(dt*1000000.0);
  }
}

//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x);
  }

















  void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Drone Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <style>
    :root {
      --primary: #4361ee;      /* Vibrant blue */
      --primary-dark: #3a56d4; /* Darker blue */
      --secondary: #7209b7;    /* Purple */
      --accent: #f72585;       /* Pink */
      --danger: #e63946;       /* Red */
      --dark: #1a1a2e;         /* Dark blue */
      --light: #f8f9fa;        /* Light gray */
      --grey: #6c757d;         /* Medium gray */
      --success: #2a9d8f;      /* Teal */
    }
    
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      -webkit-tap-highlight-color: transparent;
      touch-action: manipulation;
    }
    
    body {
      font-family: 'Inter', 'Segoe UI', system-ui, sans-serif;
      background: linear-gradient(135deg, var(--dark) 0%, #16213e 100%);
      color: var(--light);
      min-height: 100vh;
      padding: 20px;
      overflow-x: hidden;
      display: flex;
      justify-content: center;
      align-items: center;
    }
    
    .container {
      max-width: 100%;
      width: 100%;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    
    .header {
      text-align: center;
      margin-bottom: 25px;
      padding: 15px;
      border-radius: 16px;
      background: rgba(255, 255, 255, 0.05);
      backdrop-filter: blur(10px);
      width: 100%;
      max-width: 600px;
    }
    
    h1 {
      font-size: 28px;
      color: var(--primary);
      margin-bottom: 8px;
      font-weight: 700;
    }
    
    .subtitle {
      font-size: 14px;
      color: var(--grey);
    }
    
    .control-row {
      display: flex;
      justify-content: center;
      align-items: center;
      margin-bottom: 25px;
      flex-wrap: wrap;
      gap: 25px;
      width: 100%;
      max-width: 600px;
    }
    
    .joystick-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      width: 140px;
    }
    
    .joystick-wrapper {
      position: relative;
      width: 110px;
      height: 110px;
      margin-bottom: 15px;
      display: flex;
      justify-content: center;
      align-items: center;
    }
    
    .joystick {
      width: 100%;
      height: 100%;
      background: rgba(255, 255, 255, 0.08);
      border-radius: 50%;
      position: relative;
      box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
      border: 2px solid var(--primary);
      display: flex;
      justify-content: center;
      align-items: center;
    }
    
    .joystick-handle {
      width: 40px;
      height: 40px;
      background: var(--primary);
      border-radius: 50%;
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      transition: transform 0.1s ease;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
      z-index: 10;
    }
    
    .joystick-center {
      width: 8px;
      height: 8px;
      background: rgba(255, 255, 255, 0.5);
      border-radius: 50%;
      position: absolute;
      z-index: 5;
    }
    
    .joystick-label {
      font-size: 16px;
      font-weight: 600;
      color: var(--light);
      margin-bottom: 12px;
      text-align: center;
    }
    
    .direction-buttons {
      display: flex;
      flex-direction: column;
      gap: 8px;
      width: 100%;
      align-items: center;
    }
    
    .dir-btn {
      padding: 10px 12px;
      background: rgba(255, 255, 255, 0.1);
      border: 2px solid var(--primary);
      border-radius: 10px;
      color: var(--light);
      font-size: 12px;
      font-weight: 500;
      cursor: pointer;
      transition: all 0.2s;
      width: 100%;
      max-width: 120px;
      text-align: center;
    }
    
    .dir-btn.active {
      background: var(--primary);
    }
    
    .emergency-section {
      display: flex;
      flex-direction: column;
      align-items: center;
      width: 140px;
    }
    
    .emergency-btn {
      width: 100px;
      height: 100px;
      background: var(--grey);
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 16px;
      font-weight: bold;
      box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
      transition: all 0.2s;
      border: 2px solid var(--danger);
      color: white;
      cursor: pointer;
      text-align: center;
      line-height: 1.3;
      margin-bottom: 15px;
    }
    
    .emergency-btn.active {
      background: var(--danger);
      transform: scale(0.95);
      box-shadow: 0 4px 10px rgba(0, 0, 0, 0.2);
    }
    
    .config-section {
      background: rgba(255, 255, 255, 0.05);
      backdrop-filter: blur(10px);
      border-radius: 16px;
      padding: 20px;
      margin-bottom: 25px;
      width: 100%;
      max-width: 600px;
    }
    
    .section-title {
      font-size: 18px;
      color: var(--primary);
      margin-bottom: 15px;
      text-align: center;
      font-weight: 600;
    }
    
    .config-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
    }
    
    .config-item {
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    
    .config-label {
      font-size: 14px;
      color: var(--grey);
      margin-bottom: 8px;
    }
    
    .status-section {
      background: rgba(255, 255, 255, 0.05);
      backdrop-filter: blur(10px);
      border-radius: 16px;
      padding: 20px;
      margin-bottom: 25px;
      width: 100%;
      max-width: 600px;
    }
    
    .status-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
    }
    
    .status-item {
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    
    .status-label {
      font-size: 14px;
      color: var(--grey);
      margin-bottom: 8px;
    }
    
    .status-value {
      font-size: 18px;
      font-weight: bold;
      color: var(--primary);
      font-family: 'JetBrains Mono', 'Courier New', monospace;
    }
    
    .instructions {
      background: rgba(255, 255, 255, 0.05);
      backdrop-filter: blur(10px);
      border-radius: 16px;
      padding: 20px;
      font-size: 14px;
      color: var(--grey);
      width: 100%;
      max-width: 600px;
    }
    
    .instructions h3 {
      color: var(--primary);
      margin-bottom: 12px;
      font-size: 18px;
      text-align: center;
      font-weight: 600;
    }
    
    .instructions ul {
      padding-left: 20px;
      margin-bottom: 12px;
    }
    
    .instructions li {
      margin-bottom: 8px;
    }
    
    /* Responsive adjustments */
    @media (max-width: 768px) {
      .control-row {
        gap: 20px;
      }
      
      .joystick-wrapper {
        width: 100px;
        height: 100px;
      }
      
      .joystick-handle {
        width: 35px;
        height: 35px;
      }
      
      .emergency-btn {
        width: 90px;
        height: 90px;
        font-size: 14px;
      }
    }
    
    @media (max-width: 480px) {
      body {
        padding: 15px;
      }
      
      .joystick-wrapper {
        width: 90px;
        height: 90px;
      }
      
      .joystick-handle {
        width: 30px;
        height: 30px;
      }
      
      .emergency-btn {
        width: 80px;
        height: 80px;
        font-size: 13px;
      }
      
      .dir-btn {
        padding: 8px 10px;
        font-size: 11px;
        max-width: 100px;
      }
      
      .control-row {
        gap: 15px;
      }
    }
    
    /* Landscape mode */
    @media (orientation: landscape) and (max-height: 500px) {
      .container {
        display: flex;
        flex-wrap: wrap;
        flex-direction: row;
        justify-content: center;
      }
      
      .header {
        width: 100%;
      }
      
      .control-row {
        width: 100%;
        margin-bottom: 15px;
      }
      
      .config-section, .status-section {
        width: 48%;
        margin: 0 1% 15px;
      }
      
      .instructions {
        width: 100%;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>Drone Controller</h1>
      <div class="subtitle">Connect to: DroneController | Password: 123456789</div>
    </div>
    
    <div class="control-row">
      <!-- Left Joystick -->
      <div class="joystick-container">
        <div class="joystick-label">Left Stick</div>
        <div class="joystick-wrapper">
          <div class="joystick" id="leftJoystick">
            <div class="joystick-center"></div>
            <div class="joystick-handle" id="leftHandle"></div>
          </div>
        </div>
        <div class="direction-buttons">
          <button class="dir-btn" id="leftXDirBtn">X: Normal</button>
          <button class="dir-btn" id="leftYDirBtn">Y: Normal</button>
        </div>
      </div>
      
      <!-- Emergency Button -->
      <div class="emergency-section">
        <div class="joystick-label">Emergency</div>
        <button class="emergency-btn" id="emergencyBtn">STOP</button>
      </div>
      
      <!-- Right Joystick -->
      <div class="joystick-container">
        <div class="joystick-label">Right Stick</div>
        <div class="joystick-wrapper">
          <div class="joystick" id="rightJoystick">
            <div class="joystick-center"></div>
            <div class="joystick-handle" id="rightHandle"></div>
          </div>
        </div>
        <div class="direction-buttons">
          <button class="dir-btn" id="rightXDirBtn">X: Normal</button>
          <button class="dir-btn" id="rightYDirBtn">Y: Normal</button>
        </div>
      </div>
    </div>
    
    <div class="config-section">
      <div class="section-title">Direction Configuration</div>
      <div class="config-grid">
        <div class="config-item">
          <span class="config-label">Left X</span>
          <button class="dir-btn" id="leftXDirConfigBtn">Normal</button>
        </div>
        <div class="config-item">
          <span class="config-label">Left Y</span>
          <button class="dir-btn" id="leftYDirConfigBtn">Normal</button>
        </div>
        <div class="config-item">
          <span class="config-label">Right X</span>
          <button class="dir-btn" id="rightXDirConfigBtn">Normal</button>
        </div>
        <div class="config-item">
          <span class="config-label">Right Y</span>
          <button class="dir-btn" id="rightYDirConfigBtn">Normal</button>
        </div>
      </div>
    </div>
    
    <div class="status-section">
      <div class="section-title">Current Values</div>
      <div class="status-grid">
        <div class="status-item">
          <span class="status-label">Left X</span>
          <span class="status-value" id="lxVal">1500</span>
        </div>
        <div class="status-item">
          <span class="status-label">Left Y</span>
          <span class="status-value" id="lyVal">1500</span>
        </div>
        <div class="status-item">
          <span class="status-label">Right X</span>
          <span class="status-value" id="rxVal">1500</span>
        </div>
        <div class="status-item">
          <span class="status-label">Right Y</span>
          <span class="status-value" id="ryVal">1500</span>
        </div>
        <div class="status-item">
          <span class="status-label">Emergency</span>
          <span class="status-value" id="emVal">0</span>
        </div>
      </div>
    </div>
    
    <div class="instructions">
      <h3>Instructions:</h3>
      <ul>
        <li>Connect to the DroneController WiFi network</li>
        <li>Touch and drag on joysticks to control the drone</li>
        <li>Left Y-axis stays in position when released</li>
        <li>Multi-touch supported - use both joysticks at once</li>
        <li>Click direction buttons to reverse axis controls</li>
        <li>Emergency button stops all motors immediately</li>
      </ul>
    </div>
  </div>

  <script>
    // DOM Elements
    const leftStick = document.getElementById('leftJoystick');
    const rightStick = document.getElementById('rightJoystick');
    const emergencyBtn = document.getElementById('emergencyBtn');
    const leftHandle = document.getElementById('leftHandle');
    const rightHandle = document.getElementById('rightHandle');
    
    // Value displays
    const lxVal = document.getElementById('lxVal');
    const lyVal = document.getElementById('lyVal');
    const rxVal = document.getElementById('rxVal');
    const ryVal = document.getElementById('ryVal');
    const emVal = document.getElementById('emVal');
    
    // Direction buttons
    const leftXDirBtn = document.getElementById('leftXDirBtn');
    const leftYDirBtn = document.getElementById('leftYDirBtn');
    const rightXDirBtn = document.getElementById('rightXDirBtn');
    const rightYDirBtn = document.getElementById('rightYDirBtn');
    const leftXDirConfigBtn = document.getElementById('leftXDirConfigBtn');
    const leftYDirConfigBtn = document.getElementById('leftYDirConfigBtn');
    const rightXDirConfigBtn = document.getElementById('rightXDirConfigBtn');
    const rightYDirConfigBtn = document.getElementById('rightYDirConfigBtn');
    
    // Control values
    let leftX = 1500, leftY = 1500;
    let rightX = 1500, rightY = 1500;
    let emergency = false;
    
    // Direction configuration
    let leftXDir = true;
    let leftYDir = true;
    let rightXDir = true;
    let rightYDir = true;
    
    // Touch tracking for multi-touch
    let activeTouches = {};
    const LEFT_STICK = 'left';
    const RIGHT_STICK = 'right';

    // Initialize the controller
    function initController() {
      setupEventListeners();
      updateValues();
    }

    // Set up all event listeners
    function setupEventListeners() {
      // Add touch event listeners to joysticks
      addTouchListeners(leftStick, LEFT_STICK);
      addTouchListeners(rightStick, RIGHT_STICK);
      
      // Emergency button handler
      emergencyBtn.addEventListener('click', toggleEmergency);
      emergencyBtn.addEventListener('touchstart', function(e) {
        e.preventDefault();
        toggleEmergency();
      }, {passive: false});
      
      // Direction button handlers
      const directionButtons = [
        {element: leftXDirBtn, axis: 'leftX'},
        {element: leftYDirBtn, axis: 'leftY'},
        {element: rightXDirBtn, axis: 'rightX'},
        {element: rightYDirBtn, axis: 'rightY'},
        {element: leftXDirConfigBtn, axis: 'leftX'},
        {element: leftYDirConfigBtn, axis: 'leftY'},
        {element: rightXDirConfigBtn, axis: 'rightX'},
        {element: rightYDirConfigBtn, axis: 'rightY'}
      ];
      
      directionButtons.forEach(btn => {
        btn.element.addEventListener('click', () => toggleDirection(btn.axis));
        btn.element.addEventListener('touchstart', (e) => {
          e.preventDefault();
          toggleDirection(btn.axis);
        }, {passive: false});
      });
    }

    // Add touch listeners to a joystick
    function addTouchListeners(element, stickType) {
      element.addEventListener('touchstart', function(e) {
        handleTouchStart(e, stickType);
      }, {passive: false});
      
      element.addEventListener('touchmove', function(e) {
        handleTouchMove(e, stickType);
      }, {passive: false});
      
      element.addEventListener('touchend', function(e) {
        handleTouchEnd(e, stickType);
      }, {passive: false});
      
      element.addEventListener('touchcancel', function(e) {
        handleTouchEnd(e, stickType);
      }, {passive: false});
    }

    // Handle touch start events
    function handleTouchStart(e, stickType) {
      const touches = e.changedTouches;
      
      for (let i = 0; i < touches.length; i++) {
        const touch = touches[i];
        if (!activeTouches[touch.identifier]) {
          activeTouches[touch.identifier] = stickType;
          updateJoystick(stickType, touch);
        }
      }
      
      e.preventDefault();
    }

    // Handle touch move events
    function handleTouchMove(e, stickType) {
      const touches = e.changedTouches;
      
      for (let i = 0; i < touches.length; i++) {
        const touch = touches[i];
        const stick = activeTouches[touch.identifier];
        
        if (stick) {
          updateJoystick(stick, touch);
        }
      }
      
      e.preventDefault();
    }

    // Handle touch end events
    function handleTouchEnd(e, stickType) {
      const touches = e.changedTouches;
      
      for (let i = 0; i < touches.length; i++) {
        const touch = touches[i];
        const stick = activeTouches[touch.identifier];
        
        if (stick) {
          resetJoystick(stick);
          delete activeTouches[touch.identifier];
        }
      }
      
      e.preventDefault();
    }

    // Update joystick position based on touch
    function updateJoystick(stick, touch) {
      const stickElement = stick === LEFT_STICK ? leftStick : rightStick;
      const handle = stick === LEFT_STICK ? leftHandle : rightHandle;
      
      const rect = stickElement.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      
      const x = touch.clientX - rect.left;
      const y = touch.clientY - rect.top;
      
      // Calculate position relative to center (-1 to 1)
      let dx = (x - centerX) / centerX;
      let dy = (y - centerY) / centerY;
      
      // Limit to circle
      const distance = Math.sqrt(dx*dx + dy*dy);
      if (distance > 1) {
        dx /= distance;
        dy /= distance;
      }
      
      // Update handle position - ensure it stays centered
      const maxMovement = 35; // Max movement in pixels from center
      handle.style.transform = `translate(calc(-50% + ${dx * maxMovement}px), calc(-50% + ${dy * maxMovement}px)`;
      
      // Convert to 1000-2000 range
      const xVal = Math.round(1500 + dx * 500);
      const yVal = Math.round(1500 + dy * 500);
      
      if (stick === LEFT_STICK) {
        leftX = xVal;
        leftY = yVal;
      } else {
        rightX = xVal;
        rightY = yVal;
      }
      
      updateValues();
    }

    // Reset joystick to center position
    function resetJoystick(stick) {
      if (stick === LEFT_STICK) {
        // Only reset X axis for left stick, keep Y axis position
        leftX = 1500;
        const maxMovement = 35;
        leftHandle.style.transform = `translate(-50%, calc(-50% + ${(leftY - 1500) / 500 * maxMovement}px))`;
      } else {
        // Reset both axes for right stick
        rightX = 1500;
        rightY = 1500;
        rightHandle.style.transform = 'translate(-50%, -50%)';
      }
      updateValues();
    }

    // Toggle emergency state
    function toggleEmergency() {
      emergency = !emergency;
      if (emergency) {
        emergencyBtn.classList.add('active');
        emergencyBtn.textContent = "ACTIVE";
      } else {
        emergencyBtn.classList.remove('active');
        emergencyBtn.textContent = "STOP";
      }
      updateValues();
    }

    // Toggle direction for an axis
    function toggleDirection(axis) {
      switch(axis) {
        case 'leftX':
          leftXDir = !leftXDir;
          updateDirectionButton(leftXDirBtn, leftXDir, 'X');
          updateDirectionButton(leftXDirConfigBtn, leftXDir, 'X');
          break;
        case 'leftY':
          leftYDir = !leftYDir;
          updateDirectionButton(leftYDirBtn, leftYDir, 'Y');
          updateDirectionButton(leftYDirConfigBtn, leftYDir, 'Y');
          break;
        case 'rightX':
          rightXDir = !rightXDir;
          updateDirectionButton(rightXDirBtn, rightXDir, 'X');
          updateDirectionButton(rightXDirConfigBtn, rightXDir, 'X');
          break;
        case 'rightY':
          rightYDir = !rightYDir;
          updateDirectionButton(rightYDirBtn, rightYDir, 'Y');
          updateDirectionButton(rightYDirConfigBtn, rightYDir, 'Y');
          break;
      }
      updateValues();
    }

    // Update direction button text
    function updateDirectionButton(button, isNormal, axis) {
      button.textContent = `${axis}: ${isNormal ? 'Normal' : 'Reversed'}`;
      button.classList.toggle('active', !isNormal);
    }

    // Update values and send to ESP32
    function updateValues() {
      // Apply direction configuration
      const sendLX = leftXDir ? leftX : 3000 - leftX;
      const sendLY = leftYDir ? leftY : 3000 - leftY;
      const sendRX = rightXDir ? rightX : 3000 - rightX;
      const sendRY = rightYDir ? rightY : 3000 - rightY;
      
      // Update display
      lxVal.textContent = sendLX;
      lyVal.textContent = sendLY;
      rxVal.textContent = sendRX;
      ryVal.textContent = sendRY;
      emVal.textContent = emergency ? 1 : 0;
      
      // Send to ESP32
      sendControlData(sendLX, sendLY, sendRX, sendRY, emergency);
    }

    // Send control data to ESP32
    function sendControlData(lx, ly, rx, ry, em) {
      const data = {
        lx: lx,
        ly: ly,
        rx: rx,
        ry: ry,
        em: em ? 1 : 0,
        lxDir: leftXDir ? 1 : 0,
        lyDir: leftYDir ? 1 : 0,
        rxDir: rightXDir ? 1 : 0,
        ryDir: rightYDir ? 1 : 0
      };
      
      fetch('/control?' + new URLSearchParams(data))
        .catch(err => console.log('Error sending data:', err));
    }

    // Initialize the controller when page loads
    window.addEventListener('load', initController);
    
    // Handle orientation changes
    window.addEventListener('orientationchange', function() {
      setTimeout(() => {
        window.dispatchEvent(new Event('resize'));
      }, 300);
    });
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleControl() {
  leftX = server.arg("lx").toInt();
  leftY = server.arg("ly").toInt();
  rightX = server.arg("rx").toInt();
  rightY = server.arg("ry").toInt();
  emergency = server.arg("em").toInt() == 1;
  
  // Update direction configuration if provided
  if (server.hasArg("lxDir")) leftXDir = server.arg("lxDir").toInt() == 1;
  if (server.hasArg("lyDir")) leftYDir = server.arg("lyDir").toInt() == 1;
  if (server.hasArg("rxDir")) rightXDir = server.arg("rxDir").toInt() == 1;
  if (server.hasArg("ryDir")) rightYDir = server.arg("ryDir").toInt() == 1;

  //sendControlData();
}

void sendControlData() {
  Serial.printf("LX: %d (Dir: %s), LY: %d (Dir: %s), RX: %d (Dir: %s), RY: %d (Dir: %s), Emergency: %d\n",
                leftX, leftXDir ? "Normal" : "Reversed",
                leftY, leftYDir ? "Normal" : "Reversed",
                rightX, rightXDir ? "Normal" : "Reversed",
                rightY, rightYDir ? "Normal" : "Reversed",
                emergency);
}
