//  WATERROCKET -  Implements parachute deployment from IMU data
//  
//  
//  Author:   Thomas Lew
//  email:    thomas.lew@epfl.ch
//  Website:  https://github.com/thomasjlew/
//  August 2017; Last revision: 27-August-2017
//
//  ------------- BEGIN CODE --------------
#include <Wire.h>
#include <Servo.h>


#define SERVO_ANGLE_PARACHUTE_OUT 130
#define SERVO_ANGLE_PARACHUTE_IN 0
#define SERVO_PARACHUTE_PIN_NB    9

//#define SERIAL_BAUD_RATE 9600
//#define SERIAL_BAUD_RATE 19200
#define SERIAL_BAUD_RATE 38400
//#define SERIAL_BAUD_RATE 57600
//#define SERIAL_BAUD_RATE 115200   //tries to self-destroy :(
#define SERIAL_TIMEOUT_TIME 5
#define SERIAL_IN_OUT_DELAY 7

//  If enabled, sends debug info with serial com
//#define B_DEBUG true
#define B_DEBUG true
#define B_FAILURE false
#define B_SUCCESS true

#define B_APPLY_LOW_PASS true
#define ALPHA_LOW_PASS 0.1

Servo parachute_servo;
int servo_angle = SERVO_ANGLE_PARACHUTE_IN;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
const int NB_DATA_INIT_GYRO_OFFSET = 10;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t offset_gx, offset_gy, offset_gz;
double ax, ay, az;
double gx, gy, gz;
double new_ax, new_ay, new_az;
double new_gx, new_gy, new_gz;

int loop_counter = 0;
unsigned long time;


//  ------------------------------------------------------
//  INIT_SERVO()
//  Initializes a servomotor
//  Requires to include the "Servo" library
int init_servo( Servo servo,
                int init_angle,
                int pin_nb) {
  servo.attach(pin_nb);
  servo.write(init_angle);

  return B_SUCCESS;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  INIT_IMU
//  Initializes an Inertial Measurement Unit (IMU-MPU6050)
//  Requires to include the "Wire" library
int init_imu() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //  Note: code taken from:
  // MPU-6050 Short Example Sketch
  // By Arduino User JohnChi
  // August 17, 2014

  return B_SUCCESS;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  INIT_SERIAL_DEBUG
//  Initializes Serial communication with computer
//    The output can be displayed in "Serial Monitor"
int init_serial_debug(int baud_rate = SERIAL_BAUD_RATE,
                int timeout_time = SERIAL_TIMEOUT_TIME) {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_TIME);

  return B_SUCCESS;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  INIT_GYROSCOPES()
//  Initializes IMU: initializes gyroscope offsets
int init_gyroscopes(){
  offset_gx = 0; offset_gy = 0; offset_gz = 0;
  
  for (int i = 0; i < NB_DATA_INIT_GYRO_OFFSET; i++)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    offset_gx += GyX;
    offset_gy += GyY;
    offset_gz += GyZ;
  }
  
  offset_gx = (int16_t)(double(offset_gx) / NB_DATA_INIT_GYRO_OFFSET);
  offset_gy = (int16_t)(double(offset_gy) / NB_DATA_INIT_GYRO_OFFSET);
  offset_gz = (int16_t)(double(offset_gz) / NB_DATA_INIT_GYRO_OFFSET);

  return B_SUCCESS;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  SETUP()
//  Main Initialization
//  - Setups the microcontroller ports, communication
//  - Initialize gyroscope offsets
void setup() {
  //  Initialise and try again later if not working
  if(!init_servo(parachute_servo,
                 SERVO_ANGLE_PARACHUTE_IN,
                 SERVO_PARACHUTE_PIN_NB)){
    delay(500); //  Wait 500 milliseconds
    setup();    //  Try to restart initialization
  }
  
  if(!init_imu()){
    delay(500); //  Wait 500 milliseconds
    setup();    //  Try to restart initialization
  }
  
  if(B_DEBUG)
    if(!init_serial_debug()){
    delay(500); //  Wait 500 milliseconds
    setup();    //  Try to restart initialization
  }

  if(!init_gyroscopes()){
    offset_gx = 0;  //  Ignore offset initialization
    offset_gy = 0;
    offset_gz = 0;
  }
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  IMU_READ_DATA()
//  Reads IMU data and stores it in corresponding variables
int imu_read_data(bool b_apply_lowpass_filter){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //  Convert raw data into appropriate values
  new_ax = double(AcX) / 16384;
  new_ay = double(AcY) / 16384;
  new_az = double(AcZ) / 16384;
  new_gx = double(GyX - offset_gx) / 131;
  new_gy = double(GyY - offset_gy) / 131;
  new_gz = double(GyZ - offset_gz) / 131;

  if(b_apply_lowpass_filter){
    ax = apply_low_pass_filter(ax, new_ax);
    ay = apply_low_pass_filter(ay, new_ay);
    az = apply_low_pass_filter(az, new_az);
    gx = apply_low_pass_filter(gx, new_gx);
    gy = apply_low_pass_filter(gy, new_gy);
    gz = apply_low_pass_filter(gz, new_gz);
  }
  else{
    ax = new_ax;
    ay = new_ay;
    az = new_az;
    gx = new_gx;
    gy = new_gy;
    gz = new_gz;
  }
  
  return B_SUCCESS;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  IMU_FILTER_LOW_PASS()
//  Applies a simplified filter to avoid spikes
double apply_low_pass_filter(double old_value,
                             double new_value){
  //   Apply low-pass filter to avoid spikes
  return ALPHA_LOW_PASS * new_value +
         (1-ALPHA_LOW_PASS) * old_value;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  SET_SERVO_ANGLE()
//  Sets the angle of a servomotor (which was initialized)
//int set_servo_angle( Servo servo, 
//                      int init_angle) {
//  servo.write(init_angle);
//  
//  return B_SUCCESS;
//}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  DETECT_ROCKET_APOGEE()
//  Detects when the water rocket attained its apogee
//  Returns true if apogee is reached (release parachute)
int detect_rocket_apogee() {
  //  Perform filtering
  //  ......

  //  Detect apogee
  //if(ax < - 0.1)
  if(az >  0.1)
    return true;  //  Rocket nose pointing down
  else
    return false;
}
//  ------------------------------------------------------


//  ------------------------------------------------------
//  DEBUG_SERIAL_IMU()
//  Sends imu data via serial port to debug
int debug_serial_imu(){
  //  For raw data output, use this:
  /*Serial.print("Iter n."); 
  Serial.print(loop_counter); Serial.print(" ");
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.print(" ");
  time = millis();
  Serial.print(time); Serial.print(" ");
  Serial.print("RockNRoll!");
  Serial.print("\n"); //*/
  //  ---------------

  //  For a nice plot, use this section of the code
  Serial.print(ax); Serial.print(" ");
  Serial.print(new_ax); Serial.print(" "); 
  Serial.print(az); Serial.print(" ");
  Serial.print(new_az); 
  Serial.print("\n"); //*/
  //  ---------------

  return B_SUCCESS;
}
//  -------------------------------------


//  -------------------------------------
//  DETECT_TAKEOFF()
//  Detects the rocket takeoff from IMU
//  -------------------------------------
int detect_takeoff(){
  if(az < -1.3){
    return true;
  }
  else
    return false;
}
//  -------------------------------------


//  -------------------------------------
//  LOOP()
//  Main arduino loop
//  - Collects IMU data
//  - Sends IMU data serial port
//  -------------------------------------
void loop() {
  loop_counter++;

  imu_read_data(B_APPLY_LOW_PASS);

  //  Timer-based deployment
  if(detect_takeoff()){
    delay(5000);
    //  release_parachute();
    parachute_servo.write(SERVO_ANGLE_PARACHUTE_OUT);
    delay(1000);
  }

  //  Apogee detection parachute deployment
  //  Disabled: IMU is not enough for this
  if(detect_rocket_apogee()){
    //  release_parachute();
    //parachute_servo.write(SERVO_ANGLE_PARACHUTE_OUT);
  }
  else{
    parachute_servo.write(SERVO_ANGLE_PARACHUTE_IN);
  }

  if(B_DEBUG){
    /*  Send Data to serial port. Format:
          counter ax ay az gx gy gz time RockNRoll!\n
        example:
          6162 -0.84 0.18 -0.53 0.27 -0.02 -0.35 105554 RockNRoll!
    */
    debug_serial_imu();
  }

}
//  -------------------------------------
