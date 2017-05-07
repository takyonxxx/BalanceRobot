#include <Wire.h>
#include <Kalman.h> 
#include "ComPacket.h"
#include <DueFlashStorage.h>
#include "NewPing.h"
#include "PID_v1.h"

#define RESTRICT_PITCH 
#define GUARD_GAIN 5

//motor  define L298P
#define PWM_L 2  //M1 ENA
#define PWM_R 3  //M2 ENB

#define pingPin 9 // Trigger Pin of Ultrasonic Sensor
#define echoPin 8 // Echo Pin of Ultrasonic Sensor

#define DIR_L1 5 //IN2
#define DIR_L2 4 //IN1
#define DIR_R1 7 //IN4
#define DIR_R2 6 //IN3

//encoder define
#define SPD_INT_R 13   //interrupt R
#define SPD_PUL_R 12  
#define SPD_INT_L 11   //interrupt L
#define SPD_PUL_L 10   

// Rutine for repeat part of the code every X miliseconds
#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(pingPin, echoPin, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// kalman variables for sonar
float varSonar = 1.12184278324081E-05;  // variance determined using excel and reading samples of raw sensor data
float varProcess = 1e-8;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

float distance = MAX_DISTANCE;

Kalman kalmanX; 
Kalman kalmanY;

ComPacket SerialPacket;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double temperature;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


int Speed_L,Speed_R;
int mSpeed,pwm_l,pwm_r;

int Speed_Need,Turn_Need;
int Speed_Diff,Speed_Diff_ALL;

double Angle_Car;
double Gyro_Car;
double Correction;

char buf[100];

uint8_t sensorPin = A0; 


//speed control values
long lastSpeedError = 0;
long speedAdjust = 0;
long dSpeedError,adjustLMotor,adjustRMotor;
long SKp ,SKi ,SKd;

int Position_AVG = 0;

float ftmp = 0;

DueFlashStorage dueFlashStorage;

struct storedData {
  float kp;
  float ki;
  float kd;
  int kc;
};

storedData m_storedData;

/////////////

#define SAMPLE_TIME 10

double Setpoint, Input, Output;

double aggKp, aggKi, aggKd;

//Specify the links and initial tuning parameters
PID balancePID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  aggKp=190;
  aggKi=425;
  aggKd=2.55;
   
  Serial.print("\nBalance Robot Initializing...\n"); 

  uint8_t codeRunningForTheFirstTime = dueFlashStorage.read(0); // flash bytes will be 255 at first run
 
  if (codeRunningForTheFirstTime) {    
    /* OK first time running, set defaults */
    m_storedData.kp = aggKp;
    m_storedData.ki = aggKi;
    m_storedData.kd = aggKd; 
    m_storedData.kc = Correction; 
    
    // write configuration struct to flash at adress 4
    byte b2[sizeof(storedData)]; // create byte array to store the struct
    memcpy(b2, &m_storedData, sizeof(storedData)); // copy the struct to the byte array
    dueFlashStorage.write(4, b2, sizeof(storedData)); // write byte array to flash

    // write 0 to address 0 to indicate that it is not the first time running anymore
    dueFlashStorage.write(0, 0); 
  }  
 
 /* read configuration struct from flash */
  byte* b = dueFlashStorage.readAddress(4); // byte array which is read from flash at adress 4
  storedData configurationFromFlash; // create a temporary struct
  memcpy(&configurationFromFlash, b, sizeof(storedData)); // copy byte array to temporary struct

  aggKp = configurationFromFlash.kp;
  aggKi = configurationFromFlash.ki;
  aggKd = configurationFromFlash.kd;
  Correction = configurationFromFlash.kc;

  Serial.print("Kp:");
  Serial.print(aggKp);

  Serial.print(" Ki:");
  Serial.print(aggKi);

  Serial.print(" Kd:");
  Serial.print(aggKd);

  Serial.print(" Correction:");
  Serial.print(Correction);

  Init();

  initGyro();  

  Setpoint = 0;  

    //setup PID    
  balancePID.SetMode(AUTOMATIC);
  balancePID.SetSampleTime(SAMPLE_TIME);
  balancePID.SetOutputLimits(-255, 255);  
  
  timer = micros();   
}

void Init()
{  
  
  pinMode(SPD_PUL_L, INPUT);
  pinMode(SPD_PUL_R, INPUT);
  pinMode(SPD_INT_L, INPUT_PULLUP);
  pinMode(SPD_INT_R, INPUT_PULLUP);
  
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT); 

  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //init variables
  Speed_L = 0;
  Speed_R = 0;
 
  mSpeed = 0; pwm_l = 0; pwm_r = 0;
  Speed_Diff = 0;Speed_Diff_ALL = 0;   
  
  Speed_Need = 0;
  Turn_Need = 0;
  Correction = 0;

  //motor speed difference (Left-Right) correction
  lastSpeedError = 0;
  speedAdjust = 0;
  dSpeedError = 0;
  adjustLMotor = 0;
  adjustRMotor = 0;
  SKp = 1L;
  SKi = 0.5L;
  SKd = 0.3L;  

  attachInterrupt(SPD_INT_L, Encoder_L,FALLING);
  attachInterrupt(SPD_INT_R, Encoder_R,FALLING);   
 
  //lastTime = millis() - SampleTime;
  
}

double getVoltage(uint8_t sensorPin,uint16_t refVoltage,float dividerRatio) {    
  analogReadResolution(12);  //A0 
  double reading = analogRead(sensorPin) * dividerRatio * refVoltage / 4096 / 1000;   
  delay(2); // allow the ADC to stabilize
  return reading;
}

float getFilteredSonar()
{  
  long sonar_value = sonar.ping_cm();
  
   // kalman process
  Pc = P + varProcess;
  G = Pc/(Pc + varSonar);    // kalman gain
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(sonar_value-Zp)+Xp;   // the kalman estimate of the sensor voltage

  return Xe;
}

void MySerialEvent()
{
  uchar c = '0';
  uchar tmp = 0;
  
  if(Serial1.available()) {
    
    c = (uchar)Serial1.read(); 
   
    for(int i = 5; i > 0; i--)
    {
       SerialPacket.m_Buffer[i] = SerialPacket.m_Buffer[i-1];
    }
    
    SerialPacket.m_Buffer[0] = c;
    
    if(SerialPacket.m_Buffer[5] == 0xAA)
    {
       tmp = SerialPacket.m_Buffer[1]^SerialPacket.m_Buffer[2]^SerialPacket.m_Buffer[3];
       if(tmp == SerialPacket.m_Buffer[0])
       {
         SerialPacket.m_PackageOK = true;         
       }
    } 
  } 
}

void UserComunication()
{
  MySerialEvent();  
  
  if(SerialPacket.m_PackageOK == true)
  {    
    SerialPacket.m_PackageOK = false;     
   
    switch(SerialPacket.m_Buffer[4])
    {      
      case 0x01:  break;
      case 0x02: UpdatePID();break;
      case 0x03: CarDirection();break;
      case 0x04:  break;
      case 0x05:  break;                  
      case 0x06:  break;
      case 0x07:  break;
      default:    break;             
    }
  }  
}

void CarDirection()
{    
  unsigned char Speed = SerialPacket.m_Buffer[1];
  switch(SerialPacket.m_Buffer[3])
  {
    case 0x00: Speed_Need = 0;Turn_Need = 0;break;
    case 0x01: Speed_Need = -Speed; break;
    case 0x02: Speed_Need = Speed; break;
    case 0x03: Turn_Need = Speed; break;
    case 0x04: Turn_Need = -Speed; break;
    case 0x05: Correction = Correction + 0.1;UpdateCorrection(); break;
    case 0x06: Correction = Correction - 0.1;UpdateCorrection(); break;
    default:break;
  }     
}

void UpdateCorrection()
{
  m_storedData.kc = Correction; 

  // write configuration struct to flash at adress 4
  byte b2[sizeof(storedData)]; // create byte array to store the struct
  memcpy(b2, &m_storedData, sizeof(storedData)); // copy the struct to the byte array
  dueFlashStorage.write(4, b2, sizeof(storedData)); // write byte array to flash

  // write 0 to address 0 to indicate that it is not the first time running anymore
  dueFlashStorage.write(0, 0); 
}

void loop() {      
unsigned long timeSerial = millis();  

    while(1)
    { 
      if(calculateGyro())
      {  
          PWM_Calculate();
          Car_Control();    
          //distance = getFilteredSonar();   
           
          sprintf(buf, "Data:%d:%d:%0.1f:%d:%d:%d:%d:%0.2f:%0.2f:%0.2f:%0.1f:%d:%0.1f>", 
          pwm_l, pwm_r,Angle_Car,Speed_Need,Turn_Need,Speed_L,Speed_R,aggKp,aggKi,aggKd,temperature,Position_AVG,Correction);           
          Serial1.println(buf);    
          
          /*sprintf(buf, "pwm_l:%d  pwm_r:%d  Position_Add:%d  Angle_Car:%0.1f  Gyro_Car:%0.1f Speed_Need:%d",pwm_l,pwm_r,Position_AVG,Angle_Car,Gyro_Car,Speed_Need);     
          Serial.println(buf);*/
      }  

      UserComunication();         
      
    }    
}

void initGyro() { 
  
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;  

}

int calculateGyro()
{ 
  if((micros() - timer) >= 10000) 
  {    //10ms 
     /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
    
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    // It is then converted from radians to degrees
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif
    
    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s
    
    #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    
    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    
    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    #endif 
    
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
    
    Angle_Car = kalAngleX;   //negative backward  positive forward
    Gyro_Car = gyroXrate;    
    
    temperature = (double)tempRaw / 340.0 + 36.53;
   
    return 1;
   }
  return 0;
}

void Encoder_L()   //car up is positive car down  is negative
{
  if (digitalRead(SPD_PUL_L))
    Speed_L += 1;
  else
    Speed_L -= 1;

  /*sprintf(buf, "Speed_L:%d  Speed_R:%d  Speed_Diff_ALL:%d",Speed_L,Speed_R,Speed_Diff_ALL);     
  Serial.println(buf);  */
                        
}

void Encoder_R()    //car up is positive car down  is negative
{
  if (!digitalRead(SPD_PUL_R))
    Speed_R += 1;
  else
    Speed_R -= 1;   

  /*sprintf(buf, "Speed_L:%d  Speed_R:%d  Speed_Diff_ALL:%d",Speed_L,Speed_R,Speed_Diff_ALL);   
  Serial.println(buf);*/
}

void correctSpeedDiff() 
{
  dSpeedError = Speed_Diff - lastSpeedError;
  
  speedAdjust = (SKp*Speed_Diff)
              + (SKi*Speed_Diff_ALL)
              + (SKd*dSpeedError);

  adjustLMotor -= speedAdjust/2 ;
  adjustRMotor += speedAdjust/2 ;  
  
  lastSpeedError = Speed_Diff;

}

bool StopFlag = true;

float integrated_error = 0;
int count = 0,last_count;
float pTerm,iTerm,dTerm,last_error,pTerm_Wheel,dTerm_Wheel;
float Kp_Wheel,Kd_Wheel,K;

int getPid(float targetPosition, float currentPosition)   {
  Kp_Wheel = 1;
  Kd_Wheel = 1;
  K=1;
  
  float error = targetPosition - currentPosition; 
  pTerm = aggKp * error;
  integrated_error += error;                                       
  iTerm = aggKi * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = aggKd * (error - last_error);                            
  last_error = error;
  
  count = -Speed_R;
  pTerm_Wheel = Kp_Wheel * count;
  dTerm_Wheel = Kd_Wheel * (count - last_count);                            
  last_count = count;
  return -K*(pTerm + iTerm + dTerm + pTerm_Wheel + dTerm_Wheel);
}


void PWM_Calculate()
{     
  Speed_Diff = Speed_L - Speed_R;
  Speed_Diff_ALL += Speed_Diff;    
 
  //correctSpeedDiff();

  ftmp = (Speed_L + Speed_R) * 0.5;
  
  if( ftmp > 0)
    Position_AVG = ftmp +0.5;  
  else
    Position_AVG = ftmp -0.5;    

  Input = Angle_Car;

  Setpoint = Correction;

  balancePID.Setpoint(&Setpoint);
  
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    balancePID.SetTunings(aggKp/3, aggKi/3, aggKd/3);
  }
  else
  {     //we're far from setpoint, use aggressive tuning parameters
     balancePID.SetTunings(aggKp, aggKi, aggKd);
  }

  balancePID.Compute(2.5 * Speed_Need);
  
  mSpeed = -constrain(Output,-255,255);    
    
  if((Speed_Need != 0) && (Turn_Need == 0))
  {
    if(StopFlag == true)
    {
      Speed_Diff_ALL = 0;
      StopFlag = false;
    }
    pwm_r = constrain(int(mSpeed - Turn_Need - adjustRMotor - Speed_Diff_ALL),-255,255);
    pwm_l = constrain(int(mSpeed + Turn_Need + adjustLMotor + Speed_Diff_ALL ),-255,255);
  
  }
  else
  {
    StopFlag = true;
    pwm_r = constrain(int(mSpeed - Turn_Need - adjustRMotor),-255,255);
    pwm_l = constrain(int(mSpeed + Turn_Need + adjustLMotor),-255,255);
  }
  
  Speed_L = 0;
  Speed_R = 0;
}

void Car_Control()
{  
 if (pwm_l>0)
  {
    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, HIGH);
  }
  if (pwm_l<0)
  {
    digitalWrite(DIR_L1, HIGH);
    digitalWrite(DIR_L2, LOW);
    pwm_l =- pwm_l;  //cchange to positive
  }
  
  if (pwm_r>0)
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
  }
  if (pwm_r<0)
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
    pwm_r = -pwm_r;
  }  
  
  if( Angle_Car > 40 || Angle_Car < -40 )
  {
    pwm_l = 0;
    pwm_r = 0;
  }

  analogWrite(PWM_L, pwm_l);
  analogWrite(PWM_R, pwm_r);    
}
void UpdatePID()
{  
  unsigned int Upper,Lower;
  double NewPara;
  Upper = SerialPacket.m_Buffer[2];
  Lower = SerialPacket.m_Buffer[1];
  NewPara = (float)(Upper<<8 | Lower)/100.0;  
  
  switch(SerialPacket.m_Buffer[3])
  {
    case 0x01:aggKp = NewPara;break; 
    case 0x02:aggKi = NewPara;break;
    case 0x03:aggKd = NewPara;break;
    default:break;
  }

  sprintf(buf,"UpdatePID: %0.2X VAL: %0.2f Kp: %0.2f Ki: %0.2f Kd: %0.2f",SerialPacket.m_Buffer[3],NewPara,aggKp,aggKi,aggKd);
  Serial.println(buf); 
 
  m_storedData.kp = aggKp;
  m_storedData.ki = aggKi;
  m_storedData.kd = aggKd; 

  // write configuration struct to flash at adress 4
  byte b2[sizeof(storedData)]; // create byte array to store the struct
  memcpy(b2, &m_storedData, sizeof(storedData)); // copy the struct to the byte array
  dueFlashStorage.write(4, b2, sizeof(storedData)); // write byte array to flash

  // write 0 to address 0 to indicate that it is not the first time running anymore
  dueFlashStorage.write(0, 0); 
}

/*// PID-control values
float pTerm = 0;
float iTerm = 0; // Used to accumalate error (intergral)
float dTerm = 0;

float Kp;// (P)roportional Tuning Parameter
float Ki;// (I)ntegral Tuning Parameter        
float Kd;// (D)erivative Tuning Parameter 

float lastError = 0;                // Keeps track of error over time
float targetAngle = 0;          // Can be adjusted according to centre of gravity            

double lastTime = 0;
double SampleTime = 10;


int setPidPwm(float pitch)   {  
 
  float error = targetAngle - pitch;
  
  pTerm = Kp * error + Correction;                                              
  iTerm = Ki * abs(Gyro_Car) - 2 * Speed_Need;
  dTerm = Kd * (error - lastError) * constrain(Position_Add/25, - GUARD_GAIN, GUARD_GAIN);                       
  lastError = error;  
 
  float PIDValue = -constrain((pTerm + iTerm + dTerm), -255, 255);   
  return int(PIDValue);
}*/

/*
Proportional – present error
The present error is the amount of tilt that the robot has. 
It is supposed to have a tilt of 0°. If it is tiled by 10°, the error is -10.

Integral – sum of accumulated errors
The sum of errors accumulated. For the self balancing robot, this value is zero.

Derivative – expected future error
The error we expect to have on the next iteration of this algorithm.
The current tilt may be 10°, and we may be expecting it to become 11° on the next iteration.
The algorithm takes this into account when it decides how much force to apply on the wheels.

Calibrating your PID Controller

Create some way in which you can change the PID constant of your robot while it is running.
One option is to use a potentiometer or some other analogue input to be able to increase or decrease the PID constant. 
I personally used the USB connection and the serial monitor to send new PID values.
This is important as you can then see straightaway how well the new PID values are working, and you won’t have to re-upload the code hundreds of times!
Set all PID constants to zero. This is as good a place to start as any…
Slowly increase the P-constant value.
While you are doing this, hold the robot to make sure it doesn’t fall over and smash into a million pieces!
You should increase the P-constant until the robot responds quickly to any tilting, and then just makes the robot overshoot in the other direction.
Now increase the I-constant.
This component is a bit tricky to get right.
You should keep this relatively low, as it can accumulate errors very quickly.
In theory, the robot should be able to stabilise with only the P and I constants set, but will oscillate a lot and ultimately fall over.
Raise the D-constant.
A lot. The derivative components works against any motion, so it helps to dampen any oscillations and reduce overshooting.
I found that this constant has to be set significantly higher than the other two (x10 to x100 more) in order to have any effect.
All the same, don’t set it too high, as it will reduce the robot’s ability to react to external forces (aka. being pushed around).
Spend many fruitless hours slightly modifying the PID values.
This is probably the longest part of the procedure, as there isn’t much of a method to it. 
You just have to increase and decrease the values until you reach that perfect sweet-spot for your robot!

What P, I & D values means practically?
In case of Self-Balancing Robot-

P-P determines the force with which the robot will correct itself. A lower P shows robot’s inability to balance itself and a higher P will shows the violent behavior.
I-I determines the response time of robot for correcting itself. Higher the P, Faster it will response.
D- D determines the sensitivity of robot to the error in its state. It is used to smoothen/depress the robot oscillations. A lower D is unable to remove oscillations and a higher D will cause violent vibrations.*/





