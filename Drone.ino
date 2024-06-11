#include<Wire.h>
#include <ESP8266WiFi.h>

// Define Input Connections
#define CH1 D0
#define CH2 D1
#define CH3 D2
#define CH4 D3

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch3Value;
int ch4Value;
// Boolean to represent switch value-----------------------//
bool ch6Value;
//----------------------------------------------------------//   
int ESCout_1 ,ESCout_2 ,ESCout_3 ,ESCout_4;
int input_PITCH ;
int input_ROLL;
int input_YAW;
int input_THROTTLE;
int state1,state2,state3,state4;
//-----------------------------------------------------------------------// 
int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float elapsedTime;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
//-----------------------------------------------------------------------// 
float pitch_PID,roll_PID,yaw_PID;
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i;
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle; 
double twoX_kp=5;      
double twoX_ki=0.003;
double twoX_kd=2;     
double yaw_kp=3;    
double yaw_ki=0.002;

float Q_angle = 0.001; // Process noise variance for the angle and we have to make optimum changes in it as well
float Q_bias = 0.003; // Process noise variance for the bias
float R_measure = 0.03; // Measurement noise variance

// Kalman filter variables for pitch
float K_angle_pitch = 0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float bias_pitch = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate_pitch; // Unbiased rate calculated from the rate and the estimated bias - you need to compute the unbiased rate
float P_pitch[2][2] = {0, 0, 0, 0}; // Error covariance matrix - This is a 2x2 matrix
float K_pitch_bias=0;

// Kalman filter variables for roll
float K_angle_roll = 0;
float bias_roll = 0;
float rate_roll;
float K_roll_bias=0;
float P_roll[2][2] = {0, 0, 0, 0};

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(int channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void KalmanPitch(float newAngle, float newRate, float dt) {
    static float S;  // Estimate error
    static float P[2][2] = { { 1, 0 }, { 0, 1 } };  // Error covariance matrix - This is a 2x2 matrix

    // Discrete Kalman filter time update equations - Prediction step
    K_angle_pitch += dt * (newRate - K_pitch_bias);
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Measurement update - Correction step
    S = P[0][0] + R_measure;
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - K_angle_pitch;  // Angle difference
    K_angle_pitch += K[0] * y;
    K_pitch_bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}

void KalmanRoll(float newAngle, float newRate, float dt) {
    static float S;  // Estimate error
    static float P[2][2] = { { 1, 0 }, { 0, 1 } };  // Error covariance matrix

    // Prediction step
    K_angle_roll += dt * (newRate - K_roll_bias);
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correction step
    S = P[0][0] + R_measure;
    float K[2]; // Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - K_angle_roll;  // Angle difference
    K_angle_roll += K[0] * y;
    K_roll_bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}
//----------------------------------------------------------------VOID SETUP--------------------------------// 
void setup() {
Serial.begin(115200);
    // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  pinMode(19, OUTPUT);
pinMode(D5,OUTPUT);pinMode(D6,OUTPUT);pinMode(D7,OUTPUT);pinMode(D8,OUTPUT);
delay(1300);
WiFi.mode(WIFI_STA);
WiFi.begin("qwerty", "12345678");
while (WiFi.status() != WL_CONNECTED){
  GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15); 
  delayMicroseconds(1000);
  GPOC = (1 << 14);GPOC = (1 << 12);GPOC = (1 << 13);GPOC = (1 << 15);   
  delayMicroseconds(1000);
  yield(); 
}
Serial.println(WiFi.localIP()); 
//-----------------------------------------------------------------------//    
  Wire.begin();    
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                  
  Wire.endTransmission();         
  Wire.beginTransmission(0x68);                                      
  Wire.write(0x1C);                                                   
  Wire.write(0x10);                                                   
  Wire.endTransmission();             
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                 
  Wire.write(0x08);                                                   
  Wire.endTransmission();  
  delay(1000);
for (int cal_int = 0; cal_int < 2000 ; cal_int ++){  
  if(cal_int % 125 == 0)Serial.print(".");                                           
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                               
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                              
  gyro_x_cal += gyro_x;                                              
  gyro_y_cal += gyro_y;                                             
  gyro_z_cal += gyro_z;                                             
  GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15); 
  delayMicroseconds(1000);
  GPOC = (1 << 14);GPOC = (1 << 12);GPOC = (1 << 13);GPOC = (1 << 15);   
  delayMicroseconds(1000);   
  yield();                                                    
}
gyro_x_cal /= 2000;                                                 
gyro_y_cal /= 2000;                                                 
gyro_z_cal /= 2000;
//-----------------------------------------------------------------------//    
Time = micros();                                                                            
}

void loop(){ 
GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15); 
  timePrev = Time;                   
  Time = micros();  
  elapsedTime = (float)(Time - timePrev) / (float)1000000;
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                               
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read(); 
gyro_x -= gyro_x_cal;                                              
gyro_y -= gyro_y_cal;                                               
gyro_z -= gyro_z_cal;                                                       
  angle_pitch += gyro_x * elapsedTime * 0.01526717557;                                 
  angle_roll += gyro_y * elapsedTime * 0.01526717557;
  angle_yaw += gyro_z * elapsedTime * 0.01526717557;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066); 
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  angle_pitch_acc += 0;                                              
  angle_roll_acc += 0;                                         
  if(set_gyro_angles){                                                
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{                                                               
    angle_pitch = angle_pitch_acc;                                     
    angle_roll = angle_roll_acc;                                       
    set_gyro_angles = true;                                            
  }
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

//-----------------------------------------------------------------------//
KalmanPitch(angle_pitch_acc, gyro_x, elapsedTime);
KalmanRoll(angle_roll_acc, gyro_y, elapsedTime);
roll_desired_angle = 3*((float)input_ROLL/(float)10 - (float)5);
pitch_desired_angle =3*((float)input_PITCH/(float)10 - (float)5);
//yaw_desired_angle =0;

roll_error =  K_angle_roll - roll_desired_angle;
pitch_error = K_angle_pitch - pitch_desired_angle; 
yaw_error = angle_yaw - yaw_desired_angle;  
  
roll_pid_p = twoX_kp*roll_error;
pitch_pid_p = twoX_kp*pitch_error;
yaw_pid_p = yaw_kp*yaw_error;

if(-3 < roll_error <3){roll_pid_i = roll_pid_i+(twoX_ki*roll_error);}
if(-3 < pitch_error <3){pitch_pid_i = pitch_pid_i+(twoX_ki*pitch_error);}
if(-3 < yaw_error <3){yaw_pid_i = yaw_pid_i+(yaw_ki*yaw_error);}

roll_pid_d = twoX_kd*((roll_error - roll_previous_error)/elapsedTime);
pitch_pid_d = twoX_kd*((pitch_error - pitch_previous_error)/elapsedTime);
roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
yaw_PID = yaw_pid_p + yaw_pid_i;

if(roll_PID < -400){roll_PID=-400;}
else if(roll_PID > 400) {roll_PID=400;}
if(pitch_PID < -400){pitch_PID=-400;}
else if(pitch_PID > 400) {pitch_PID=400;}
if(yaw_PID < -400){yaw_PID=-400;}
else if(yaw_PID > 400) {yaw_PID=400;}

ESCout_1 = input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
ESCout_2 = input_THROTTLE + roll_PID - pitch_PID + yaw_PID;
ESCout_3 = input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
ESCout_4 = input_THROTTLE - roll_PID + pitch_PID + yaw_PID;

if(ESCout_1>2000) ESCout_1=2000;
else if(ESCout_1<1100) ESCout_1=1100;
if(ESCout_2>2000) ESCout_2=2000;
else if(ESCout_2<1100) ESCout_2=1100;
if(ESCout_3>2000) ESCout_3=2000;
else if(ESCout_3<1100) ESCout_3=1100;
if(ESCout_4>2000) ESCout_4=2000;
else if(ESCout_4<1100) ESCout_4=1100;

roll_previous_error = roll_error;
pitch_previous_error = pitch_error;
//-----------------------------------------------------------------------//
while((micros() - Time) < 1000);
state1 = 1; state2 = 1; state3 = 1; state4 = 1;
while(state1 == 1 || state2 == 1 || state3 == 1 || state4 == 1){
  time2 = micros();
  if((time2 - Time) >= ESCout_1 && state1 == 1){ GPOC = (1 << 14); state1=0;}
  if((time2 - Time) >= ESCout_2 && state2 == 1){ GPOC = (1 << 12);state2=0;}
  if((time2 - Time) >= ESCout_3 && state3 == 1){ GPOC = (1 << 13);state3=0;}
  if((time2 - Time) >= ESCout_4 && state4 == 1){ GPOC = (1 << 15);state4=0;}
}
  // Get values for each channel
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, -100);
  ch4Value = readChannel(CH4, -100, 100, 0);
  ch5Value = readChannel(CH5, -100, 100, 0);
  ch6Value = readSwitch(CH6, false);
  
  // Print to Serial Monitor
  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" | Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" | Ch3: ");
  Serial.print(ch3Value);
  Serial.print(" | Ch4: ");
  Serial.print(ch4Value);
  Serial.print(" | Ch5: ");
  Serial.print(ch5Value);
  Serial.print(" | Ch6: ");
  Serial.println(ch6Value);

  delay(500);
}
