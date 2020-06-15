#include <Wire.h>  //wire.h library is included for i2c communication


double elapsedTime, time, timePrev;         
float gyro_error=0;                    
float Gyr_rawY;     
float Gyro_angle_y;         
float Gyro_raw_error_y; 

float acc_error=0;                         
float rad_to_deg = 180/3.141592654;      
float Acc_rawX, Acc_rawY, Acc_rawZ;    
float Acc_angle_y;          
float Acc_angle_error_y; 

float Total_angle_y;      //Total angle minus desired angle will give error

float PID,error,previous_error,PWM=0;
float pid_p=0;
float pid_i=0;           //PID=pid_p+pid_i+pid_d
float pid_d=0;

double kp=29.8;
double ki=100;            //PID CONSTANTS//
double kd=0.75;     

float desired_angle=0;   //This is the desired angle at which bot must stay

void connect_to_mpu6050()
 {Wire.begin();                           
  Wire.beginTransmission(0x68); //0x68 is i2c address of mpu6050,now transmission has begun with mpu6050                      
  Wire.write(0x6B);             //power management system of mpu6050 is accessed                     
  Wire.write(0x00);             //upon power up mpu6050 comes in sleep mode(bit 6),so all bits are cleared
  Wire.endTransmission(true);}  //transmission has ended
  
void Gyro_config()
 {Wire.beginTransmission(0x68); //transmission has begun with mpu6050          
  Wire.write(0x1B);             //accessing the gyroscope register to configure it          
  Wire.write(0x10);             //setting angular velocity full scale range to +/-1000 degree/sec           
  Wire.endTransmission(true);}  //transmission has ended
  
  void Acc_config()
 {Wire.beginTransmission(0x68); //transmission has begun with mpu6050          
  Wire.write(0x1C);             //accessing the acceleration register to configure it          
  Wire.write(0x10);             //setting acceleration full scale range to +/-8g          
  Wire.endTransmission(true);}  //transmission has ended

void Acc_angle_error() //to calculate error value in acceleration
  {if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68); //transmission has begun with mpu6050
      Wire.write(0x3B);             //accessing accel_xout          
      Wire.endTransmission(false); //false is used to restart the transmission,multiple transmissions are performed
      Wire.requestFrom(0x68,6,true);  //requesting register 3B to 40 
      
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ;   // 1g = 4096 LSB/g ,the values are in LSB/g so to convert into g value divide by 4096
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;

         
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg));//Euler's equation
      
      if(a==199)
      {
        Acc_angle_error_y = Acc_angle_error_y/200; //getting mean of all acceleration to get error
        acc_error=1;
      }
    }
  }
  }


void Gyro_angle_error() //to calculate error in gyroscope value
  {if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);//transmission has begun with mpu6050             
      Wire.write(0x45);  //accessing gyro_yout                         
      Wire.endTransmission(false);  //false is used to restart the transmission,multiple transmissions are performed
      Wire.requestFrom(0x68,2,true); //requesting registers 45 and 46         
         
      
      Gyr_rawY=Wire.read()<<8|Wire.read();
   
     
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);//1degree/s=32.8 LSB/degree/sec,the values are in LSB/degree/sec so to
                                                              //convert into degree/sec divide by 32.8  
      if(i==199)                                                 
      {
        Gyro_raw_error_y = Gyro_raw_error_y/200;
        gyro_error=1;
      }
    }
  }
  } 


void Gyro_angle_read() //to calculate gyro value 
   {Wire.beginTransmission(0x68);             
    Wire.write(0x45);                        
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,2,true);          
        
    Gyr_rawY=Wire.read()<<8|Wire.read();
    
    Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;
    
    Gyro_angle_y = Gyr_rawY*elapsedTime;}

void Acc_angle_read() //to calculate acceleration value
 {Wire.beginTransmission(0x68);     
  Wire.write(0x3B);                
  Wire.endTransmission(false);      
  Wire.requestFrom(0x68,6,true);      
  
  Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; 
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 

 Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;} //calculation of angle using euler's equation

void Complementary_filter()
  {Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;}   //complementary filter,y-axis angle

void Pid()
 {error=Total_angle_y-desired_angle;
  pid_p=kp*error;
  pid_i=pid_i+(ki*error)*elapsedTime;                //PID//
  pid_d=kd*((error-previous_error)/elapsedTime);
  PID=pid_p+pid_i+pid_d;}
  
void Pwm()
 {if(PID<0)                   //if value of pid is negative then we make it positive to use it to set pwm
 {PID=-PID;}
 if(PID>255)                  //if value of pid is greater than 255 then pwm is set to 255 as it is the largest value that can be used
 {PWM=255;}
 else                        //if value of pid is positive then pwm becomes pid
 {PWM=PID;}}                  

void balance()
 {if(0<error)
 {digitalWrite(2,HIGH);
 digitalWrite(8,HIGH);
 digitalWrite(4,LOW);           //if error is positive then bot moves forward,so wheels are set to move forward
 digitalWrite(12,LOW);
 analogWrite(3,PWM);
 analogWrite(6,PWM);}

 if(error<0)
 {digitalWrite(2,LOW);
 digitalWrite(8,LOW);
 digitalWrite(4,HIGH);           //if error is negative then bot moves backward,so wheels are set to move backward
 digitalWrite(12,HIGH);
 analogWrite(3,PWM);
 analogWrite(6,PWM);}}






void setup() 
{ connect_to_mpu6050();         //configures mpu6050
  Gyro_config();                //configures gyroscope
  Acc_config();                 //configures accelerometer

    
  Serial.begin(9600);                      
  time = millis();                 

  Acc_angle_error();               //gives error in angle given by accelrometer
  Gyro_angle_error();              //gives error in angle given by gyroscope      
}

void loop() 
{  timePrev = time;                        // the previous time is stored before the actual time read
  time = millis();                        // actual time read
  elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds
  Gyro_angle_read();             //gives angle obtained by gyroscope
  Acc_angle_read();             //gives angle obtained with the help of accelerometer
  Complementary_filter();      //complementary filter is applied using angles obtained from gyroscope and accelerometer
  Pid();                       //calculates the value of pid
  Pwm();                      //calculates the value of pwm using value of pid
  balance();                  //this applies pwm to dc motors to help the bot to balance
  previous_error=error;     //error value is stored in previous error 
  delay(5); //delay of 5ms is done so that motors can react to change in angle accuratly
}
