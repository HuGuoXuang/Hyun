#include "EEPROM.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "FUTABA_SBUS.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define RXD1 16    //串口IO
#define TXD1 17

#define ANALOG_PIN_ch6   34  //电压检测的IO
#define LPF_a 0.7// 滤波系数a(0-1)

#define B1_PWM 2   //舵机IO
#define B2_PWM 0
#define A2_PWM 33
#define A1_PWM 32

#define k1pin 25   //按键IO
#define k2pin 26
#define k3pin 27

#define INTERRUPT_PIN 23  // 陀螺仪外部中断io
MPU6050 mpu;     

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0,/* reset=*/ U8X8_PIN_NONE , 12 ,14); 

FUTABA_SBUS sBus;
float sbuschx[8] = {0};

int ff = 0;
int ff1 = 0;
int ff2 = 0;

float car_ch1 = 0;     // 旋转
float car_ch2 = 0;     // 油门
float car_ch3 = 800;    // 高度
float car_ch4 = 0;     // 左右
int ch4 = 0;
int car_ch5 = 0;     // 
int car_ch6 = 0;     // 
float car_ch7 = 0;     // 
float car_ch8 = 0;     // 

float Power_Voltage=0;

float abh = 0.1;               
float AbaroAlt = 800;
float BbaroAlt = 800;

float AH = 140;
float BH = 140;

float T_a = 0.1;               
float F_Kionix = 0;

float LPF_value;  //滤波后的值

int show_f = 2;   //显示标志位
int start_f = 0;
int jdbz = 0;
char jdbz_press_time = 0;
char jdbz_press_time1 = 0;
float Ts = 0;     //周期时间
unsigned long Tt; 
unsigned long now_us = 0;
unsigned long velocity_calc_timestamp = 0;

struct{
 
    float Q_angle; 
    float Q_bias; 
    float R_measure; 
    float Angle; 
    float Angle1; 
    float Bias; 
    float Rate; 
    float Rate1;
    float Rate2;

    float p[2][2]; 

}KalmanX,KalmanY,KalmanZ;

/* IMU Data */
bool imuReady = false;  // 
float AngleZ=0,AngleX=0, AngleY=0;              
float AngleX_bias=0, AngleY_bias=0;    
bool dmpReady = false;  
uint8_t mpuIntStatus;  
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t fifoBuffer[64]; 

float Hcm = 0;

int Apwm1 = 0;
int Bpwm1 = 0;
// orientation/motion vars
Quaternion q;           
VectorInt16 aa;         
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];        
float ypr[3];         

volatile bool mpuInterrupt = false;    

boolean  key1,key2,key3;
boolean  s1,s2,s3;
boolean  s1_long_press,s2_long_press,s3_long_press;
char s1_press_time,s2_press_time,s3_press_time,ch5_press_time;
char menu_f = 0;

int freq = 50; // 频率
int resolution0 = 20;   // 分辨率，取值0~20，duty最大取值为2^resolution-1
int channel0 = 0;    // 通道0，共16个通道，0~15
int channel1 = 1;    // 
int channel2 = 2;    // 
int channel3 = 3;    // 


String comdata = "";    //串口接收数据
char terminator = '=';

unsigned char recstatu = 0;//表示是否处于一个正在接收数据包的状态
unsigned char ccnt = 0;//计数
unsigned char packerflag = 0;//是否接收到一个完整的数据包标志
unsigned char rxbuf[7] = {0,0,0,0,0,0,0};//接收数据的缓冲区
unsigned char txbuf[7] = {0,0,0,0,0,0,0};
unsigned char dat = 0;

float B_Angle = 0;//设置机器高度
float A_Angle = 0;

int A1angle_max = 79999;  //设置舵机角度的最大值
int A2angle_max = 77000;
int B1angle_max = 74444;
int B2angle_max = 79999;

int A1angle_mini = 129999; //设置舵机角度的最小值
int A2angle_mini = 28888;
int B1angle_mini = 26000;
int B2angle_mini = 127777;

float Amotor_speed = 0;   //A电机当前速度
float Bmotor_speed = 0;
float Amotor_angle_prev = 0;  //电机上一次的角度
float Bmotor_angle_prev = 0;

uint8_t n_sample = 8;           //滑动加权滤波算法采样个数
float sample_array[8] = {0};    //采样队列

uint8_t n_sample1 = 8;           
float sample_array1[8] = {0};    

uint8_t n_sample2 = 8;           
float sample_array2[8] = {0};    

uint8_t n_sample3 = 8;           
float sample_array3[8] = {0};    

float dead_band = 0;

float newerr = 0;

/////////////////////////////////
 int ix,i1,i2,i3,tx,t0,t1,sta;
 float V_min,V_max,V_mid; 
 long Freq;
 float Vpp;
 float Y[96]; //声明信号值储存数组
 float Buffer[192]; 
 const float L[] PROGMEM = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x03,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x03,0x70,0x3F,0x87,0xF0,0xC1,0x99,0x83,
0x07,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x07,0x30,0x3F,0xC7,0xF8,0xC1,0x99,0xC3,
0x1F,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,
0x06,0x38,0x30,0xC6,0x1C,0xC1,0x99,0xE3,
0x18,0x60,0x00,0x00,0x00,0x00,0x00,0x00,
0x0E,0x18,0x30,0xC6,0x0C,0xC1,0x99,0xF3,
0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,
0x0F,0xFC,0x3F,0xC6,0x0C,0xC1,0x99,0xBB,
0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,
0x1F,0xFC,0x3F,0x06,0x0C,0xC1,0x99,0x9F,
0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,
0x18,0x0E,0x31,0x86,0x1C,0xE3,0x99,0x8F,
0x38,0x70,0x00,0x00,0x00,0x00,0x00,0x00,
0x38,0x07,0x30,0xC7,0xF8,0x7F,0x19,0x87,
0x1F,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,
0x78,0x07,0xB0,0xE7,0xE0,0x3E,0x19,0x87,
0x07,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x82,0x00,0x10,
0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x04,0x92,0x00,0x08,
0x00,0x00,0x20,0x40,0x80,0x3E,0x78,0x00,
0x00,0x00,0x00,0x00,0x02,0xA4,0x01,0xFF,
0xE0,0x7F,0xC0,0x40,0x80,0x24,0x88,0x00,
0x00,0x00,0x00,0x00,0x02,0xC4,0x02,0x00,
0x40,0x00,0x00,0x08,0x88,0x24,0x88,0x00,
0x00,0x00,0x00,0x00,0x0F,0xF7,0xE2,0x00,
0x80,0x00,0x00,0x1F,0xF8,0x3C,0x78,0x00,
0x00,0x00,0x00,0x00,0x01,0xCC,0x81,0xFF,
0x80,0x00,0x19,0xA8,0x90,0x26,0xA0,0x00,
0x00,0x00,0x00,0x00,0x02,0xB4,0x80,0x02,
0x03,0xFF,0xE0,0xA8,0x80,0x03,0x14,0x00,
0x00,0x00,0x00,0x00,0x04,0x94,0x80,0x04,
0x00,0x24,0x00,0x27,0xF8,0x7C,0xE8,0x00,
0x00,0x00,0x00,0x00,0x08,0x84,0x80,0x0C,
0x00,0x24,0x80,0x4A,0x10,0x0C,0x40,0x00,
0x00,0x00,0x00,0x00,0x01,0x34,0x87,0xFF,
0xF0,0x44,0x40,0x49,0x20,0x10,0x30,0x00,
0x00,0x00,0x00,0x00,0x07,0xE3,0x00,0x08,
0x00,0x44,0x20,0x49,0x20,0x7E,0xFE,0x00,
0x00,0x1E,0x00,0x00,0x02,0x43,0x00,0x08,
0x00,0x84,0x10,0x88,0xC0,0x22,0x88,0x00,
0x00,0x33,0x00,0x00,0x01,0xC3,0x00,0x08,
0x01,0x04,0x10,0x90,0xC0,0x22,0x88,0x00,
0x00,0x61,0x80,0x00,0x01,0xB4,0x80,0x08,
0x02,0x04,0x00,0x91,0x30,0x22,0x88,0x00,
0x00,0xC0,0x80,0x00,0x02,0x08,0x60,0x38,
0x00,0x1C,0x00,0xA6,0x1C,0x3E,0xF8,0x00,
0x00,0x80,0xC0,0x00,0x04,0x30,0x00,0x10,
0x00,0x08,0x00,0x58,0x00,0x00,0x00,0x00,
0x01,0x80,0x40,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x80,0x60,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x00,0x60,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x03,0x00,0x20,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x02,0x00,0x30,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x02,0x00,0x30,0x00,0x00,0x00,0x00,0x03,
0x00,0x60,0x00,0x20,0x00,0x18,0x00,0x00,
0x06,0x00,0x10,0x00,0x00,0x00,0x00,0x06,
0x00,0x30,0x00,0x60,0x00,0x18,0x00,0x00,
0x06,0x00,0x10,0x00,0x00,0x00,0x00,0x06,
0x00,0x30,0x00,0x60,0x00,0x18,0x00,0x00,
0x04,0x00,0x10,0x00,0x00,0x00,0x00,0x06,
0x1E,0x31,0xBC,0xFE,0x38,0xDB,0xC1,0xD8,
0x04,0x00,0x18,0x00,0x00,0x00,0x00,0x0C,
0x3F,0x19,0xFE,0xFE,0x38,0xDF,0xE3,0xF8,
0x04,0x00,0x18,0x00,0x10,0x00,0x00,0x0C,
0x73,0x19,0xC6,0x63,0x39,0x9C,0x67,0x38,
0x0C,0x00,0x18,0x00,0x18,0x00,0x00,0x0C,
0x60,0x19,0x86,0x63,0x6D,0x98,0x66,0x18,
0x7F,0xFF,0xFF,0xFF,0xFE,0x00,0x00,0x0C,
0x60,0x19,0x86,0x63,0x6D,0x98,0x66,0x18,
0x0C,0x00,0x08,0x00,0x10,0x00,0x00,0x0C,
0x60,0x19,0x86,0x63,0x6D,0x98,0x66,0x18,
0x00,0x00,0x08,0x00,0x30,0x00,0x00,0x0C,
0x73,0x19,0x86,0x61,0xC7,0x18,0x67,0x38,
0x00,0x00,0x0C,0x00,0x30,0x00,0x00,0x0C,
0x3F,0x19,0x86,0x79,0xC7,0x18,0x63,0xF8,
0x00,0x00,0x0C,0x00,0x30,0x00,0x00,0x06,
0x1E,0x31,0x86,0x39,0xC7,0x18,0x61,0xD8,
0x00,0x00,0x04,0x00,0x20,0x00,0x00,0x06,
0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x18,
0x00,0x00,0x04,0x00,0x20,0x00,0x00,0x06,
0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x18,
0x00,0x00,0x06,0x00,0x60,0x00,0x00,0x03,
0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x18,
0x00,0x00,0x06,0x00,0x60,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x02,0x00,0x40,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x03,0x00,0xC0,0x00,0x00,0x00,
0x00,0x00,0x01,0xC7,0x04,0x70,0x38,0xE0,
0x00,0x00,0x03,0x00,0x80,0x00,0x00,0x00,
0x00,0x00,0x02,0x28,0x8C,0x88,0x45,0x10,
0x00,0x00,0x01,0x01,0x80,0x00,0x00,0x00,
0x00,0x00,0x00,0x28,0x94,0x08,0x45,0x10,
0x00,0x00,0x01,0x81,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x48,0x84,0x30,0x44,0xE0,
0x00,0x00,0x00,0xC3,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x88,0x84,0x08,0x45,0x10,
0x00,0x00,0x00,0x66,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x08,0x84,0x88,0x45,0x10,
0x00,0x00,0x00,0x3C,0x00,0x00,0x00,0x00,
0x00,0x00,0x03,0xE7,0x04,0x70,0x38,0xE0,
0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
}; 
/////////////////////////////////

struct{
    float ActualOut;        //定义实际输出
    float err;              //定义偏差值
    float err_last;         //定义上一个偏差值
    float err_last1;         //定义上一个偏差值
    float Kp,Ki,Kd;         //定义比例、积分、微分系数
    float pidout;           //定义控制执行器的变量
    float integral;         //定义积分值
    float derivative;       //定义微分
    float newerr;
    float newerr1;
    
}Avelocity,Abalance,Bvelocity,Bbalance,Turn,Height;

struct{
  
    float Q_Xk; 

    float R_measure;

    float Xk; 

    float p1; 

}A_speed,B_speed,Zturn;

void dmpDataReady() //陀螺仪外部中断
{
  mpuInterrupt = true;  //陀螺仪中断过标志位
}

float Tfilter(float x)
{
    float TbaroOffset  = x; 
    float Tbaro = T_a * TbaroOffset  + (1.0f - T_a) * F_Kionix;
    F_Kionix = Tbaro;
    return Tbaro;  
}

float Afilter(float x)
{
    float AbaroOffset  = x; 
    float Abaro = abh * AbaroOffset  + (1.0f - abh) * AbaroAlt;
    AbaroAlt = Abaro;
    return Abaro;  
}

float Bfilter(float x)
{
    float BbaroOffset  = x; 
    float Bbaro = abh * BbaroOffset  + (1.0f - abh) * BbaroAlt;
    BbaroAlt = Bbaro;
    return Bbaro;  
}

void sample()  
{   
   Buffer[i3] = Abalance.err;
   i3++;
   if(i3>=192)
   {
      i3 = 0; 
   }
}

void Measure()
{
  V_max=Buffer[0];
  V_min=Buffer[0];      
  for(ix=0;ix<192;ix++)
  { 
    if(Buffer[ix]>V_max) 
    V_max=Buffer[ix];
    if(Buffer[ix]<V_min) 
    V_min=Buffer[ix];
  }
  V_mid=(V_max+V_min)/2;  
  Vpp=V_max-V_mid;
  for(ix=0;ix<97;ix++)
  {
    if(Buffer[ix]<V_mid&&Buffer[ix+1]>=V_mid)  
    {
      i1=ix;
      break;
    }
  }
  for(ix=i1+1;ix<98+i1;ix++)
  {
    if(Buffer[ix]<V_mid&&Buffer[ix+1]>=V_mid)  
    {
      i2=ix;
      break;
    }
  }
  tx=i2-i1;
  if(tx>0)
  Freq=1/(tx*0.01);
  else
  Freq=0;
}

void Transform( )  
{ 
  for(sta=0;sta<96;sta++)
  {
    if(Buffer[sta]<0&&Buffer[sta+2]>0)  
    break;
  } 
  for(ix = 0;ix < 96;ix++)  
  Y[ix] =  Buffer[ix+sta];     
}


void observe()
{
  sample();
  if(i3==0)
  {
    Measure(); 
    Transform();
    for(ix = 0;ix < 96;ix++)  
    {
      Serial.print(Y[ix],3);
      Serial.println("\t"); 
    }     
    Serial.print(Freq,3);
    Serial.print("\t");  
    Serial.print(Vpp,3);
    Serial.println("\t");    
  }
}


void AB_Velocity_filter_init()
{
    A_speed.Q_Xk = 0.01f;
    A_speed.R_measure = 0.001f;
    A_speed.Xk = 0.0f; 
    A_speed.p1 = 0.01f; 

    B_speed.Q_Xk = 0.01f;
    B_speed.R_measure = 0.001f;
    B_speed.Xk = 0.0f; 
    B_speed.p1 = 0.01f;    

    Zturn.Q_Xk = 0.01f;
    Zturn.R_measure = 0.001f;
    Zturn.Xk = 0.0f; 
    Zturn.p1 = 0.01f;     

}


void Kalman_init() { 
    KalmanY.Q_angle = 0.001f;
    KalmanY.Q_bias = 0.003f;
    KalmanY.R_measure = 0.035f;
    KalmanY.Angle = 0.0f; 
    KalmanY.Bias = 0.0f;  
    KalmanY.p[0][0] = 0.0f; 
    KalmanY.p[0][1] = 0.0f;
    KalmanY.p[1][0] = 0.0f;
    KalmanY.p[1][1] = 0.0f;

    KalmanX.Q_angle = 0.001f;
    KalmanX.Q_bias = 0.003f;
    KalmanX.R_measure = 0.035f;
    KalmanX.Angle = 0.0f; 
    KalmanX.Bias = 0.0f;  
    KalmanX.p[0][0] = 0.0f; 
    KalmanX.p[0][1] = 0.0f;
    KalmanX.p[1][0] = 0.0f;
    KalmanX.p[1][1] = 0.0f;

    KalmanZ.Q_angle = 0.001f;
    KalmanZ.Q_bias = 0.003f;
    KalmanZ.R_measure = 0.035f;
    KalmanZ.Angle = 0.0f; 
    KalmanZ.Bias = 0.0f;  
    KalmanZ.p[0][0] = 0.0f; 
    KalmanZ.p[0][1] = 0.0f;
    KalmanZ.p[1][0] = 0.0f;
    KalmanZ.p[1][1] = 0.0f;    
}

void A_Velocity_filter(float NewVelocity )
{
  A_speed.p1 = A_speed.p1+A_speed.Q_Xk;
  float S = A_speed.p1+A_speed.R_measure; 
  float k1 = A_speed.p1/S;
  A_speed.Xk = A_speed.Xk+k1*(NewVelocity-A_speed.Xk);
  A_speed.p1 = (1-k1)*A_speed.p1;
}

void B_Velocity_filter(float NewVelocity )
{

  B_speed.p1 = B_speed.p1+B_speed.Q_Xk;
  
  float S = B_speed.p1+B_speed.R_measure; 
  float k1 = B_speed.p1/S;
  B_speed.Xk = B_speed.Xk+k1*(NewVelocity-B_speed.Xk);
  B_speed.p1 = (1-k1)*B_speed.p1;
}

float Z_turn_filter(float NewVelocity )
{

  Zturn.p1 = Zturn.p1+Zturn.Q_Xk;
  
  float S = Zturn.p1+Zturn.R_measure; 
  float k1 = Zturn.p1/S;
  Zturn.Xk = Zturn.Xk+k1*(NewVelocity-Zturn.Xk);
  Zturn.p1 = (1-k1)*Zturn.p1;

  return Zturn.Xk;
}


void YKalmangGetAngle(float newAngle, float newRate, float dt) {

    KalmanY.Rate = newRate - KalmanY.Bias;
    KalmanY.Angle += dt * KalmanY.Rate;

    KalmanY.p[0][0] += dt * (dt*KalmanY.p[1][1] - KalmanY.p[0][1] - KalmanY.p[1][0] + KalmanY.Q_angle);
    KalmanY.p[0][1] -= dt * KalmanY.p[1][1];
    KalmanY.p[1][0] -= dt * KalmanY.p[1][1];
    KalmanY.p[1][1] += KalmanY.Q_bias * dt;

    float S = KalmanY.p[0][0] + KalmanY.R_measure; // Estimate error

    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = KalmanY.p[0][0] / S;
    K[1] = KalmanY.p[1][0] / S;

    float y = newAngle - KalmanY.Angle; // Angle difference

    KalmanY.Angle += K[0] * y;
    KalmanY.Bias += K[1] * y;


    float P00_temp = KalmanY.p[0][0];
    float P01_temp = KalmanY.p[0][1];

    KalmanY.p[0][0] -= K[0] * P00_temp;
    KalmanY.p[0][1] -= K[0] * P01_temp;
    KalmanY.p[1][0] -= K[1] * P00_temp;
    KalmanY.p[1][1] -= K[1] * P01_temp;
}


void XKalmangGetAngle(float newAngle, float newRate, float dt) {

    KalmanX.Rate = newRate - KalmanX.Bias;
    KalmanX.Angle += dt * KalmanX.Rate;

    KalmanX.p[0][0] += dt * (dt*KalmanX.p[1][1] - KalmanX.p[0][1] - KalmanX.p[1][0] + KalmanX.Q_angle);
    KalmanX.p[0][1] -= dt * KalmanX.p[1][1];
    KalmanX.p[1][0] -= dt * KalmanX.p[1][1];
    KalmanX.p[1][1] += KalmanX.Q_bias * dt;

    float S = KalmanX.p[0][0] + KalmanX.R_measure; // Estimate error

    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = KalmanX.p[0][0] / S;
    K[1] = KalmanX.p[1][0] / S;

    float y = newAngle - KalmanX.Angle; // Angle difference

    KalmanX.Angle += K[0] * y;
    KalmanX.Bias += K[1] * y;


    float P00_temp = KalmanX.p[0][0];
    float P01_temp = KalmanX.p[0][1];

    KalmanX.p[0][0] -= K[0] * P00_temp;
    KalmanX.p[0][1] -= K[0] * P01_temp;
    KalmanX.p[1][0] -= K[1] * P00_temp;
    KalmanX.p[1][1] -= K[1] * P01_temp;
}

void ZKalmangGetAngle(float newAngle, float newRate, float dt) {

    KalmanZ.Rate = newRate - KalmanZ.Bias;
    KalmanZ.Angle += dt * KalmanZ.Rate;

    KalmanZ.p[0][0] += dt * (dt*KalmanZ.p[1][1] - KalmanZ.p[0][1] - KalmanZ.p[1][0] + KalmanZ.Q_angle);
    KalmanZ.p[0][1] -= dt * KalmanZ.p[1][1];
    KalmanZ.p[1][0] -= dt * KalmanZ.p[1][1];
    KalmanZ.p[1][1] += KalmanZ.Q_bias * dt;

    float S = KalmanZ.p[0][0] + KalmanZ.R_measure; // Estimate error

    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = KalmanZ.p[0][0] / S;
    K[1] = KalmanZ.p[1][0] / S;

    float y = newAngle - KalmanZ.Angle; // Angle difference

    KalmanZ.Angle += K[0] * y;
    KalmanZ.Bias += K[1] * y;


    float P00_temp = KalmanZ.p[0][0];
    float P01_temp = KalmanZ.p[0][1];

    KalmanZ.p[0][0] -= K[0] * P00_temp;
    KalmanZ.p[0][1] -= K[0] * P01_temp;
    KalmanZ.p[1][0] -= K[1] * P00_temp;
    KalmanZ.p[1][1] -= K[1] * P01_temp;
}

void pidcs()
{
  Abalance.Kp = 1.3;     
  Abalance.Ki = 5;
  Abalance.Kd = 0; 
  
  Avelocity.Kp = 0;
  Avelocity.Ki = 0;
  Avelocity.Kd = 0; 

  Bbalance.Kp = 1.3;     
  Bbalance.Ki = 5;
  Bbalance.Kd = 0;   

  Bvelocity.Kp = 0;
  Bvelocity.Ki = 0;
  Bvelocity.Kd = 0;     
}

void pid_init()  //pid初始化变量
{
  dead_band=0;   //死去范围

  Height.Kp = 0.2;
  Height.Ki = 0.1;
  Height.Kd = 0;
  Height.err = 0; 
  Height.integral = 0;
  Height.pidout = 0;
  Height.ActualOut = 0; 

  Turn.Kp = 0.5;
  Turn.Ki = 0;
  Turn.err = 0; 
  Turn.integral = 0;
  Turn.pidout = 0;
  Turn.ActualOut = 0; 

  //平衡环PID
  Abalance.ActualOut = 0; 
  Abalance.err = 0;  
  Abalance.err_last = 0;
  Abalance.pidout = 0;
  Abalance.integral = 0;
  Abalance.Kp = 1.3;     
  Abalance.Ki = 5;
  Abalance.Kd = 0; 

  //速度环PID
  Avelocity.ActualOut = 0;
  Avelocity.err = 0;
  Avelocity.err_last = 0;
  Avelocity.pidout = 0;
  Avelocity.integral = 0;
  Avelocity.Kp = 0.3;
  Avelocity.Ki = 0.4;
  Avelocity.Kd = 0; 
  Avelocity.newerr = 0;
  Avelocity.newerr1 = 0;

  //平衡环PID
  Bbalance.ActualOut = 0; 
  Bbalance.err = 0;  
  Bbalance.err_last = 0;
  Bbalance.pidout = 0;
  Bbalance.integral = 0;
  Bbalance.Kp = 1.3;     
  Bbalance.Ki = 5;
  Bbalance.Kd = 0; 

  //速度环PID
  Bvelocity.ActualOut = 0;
  Bvelocity.err = 0;
  Bvelocity.err_last = 0;
  Bvelocity.pidout = 0;
  Bvelocity.integral = 0;
  Bvelocity.Kp = 0.3;
  Bvelocity.Ki = 0.4;
  Bvelocity.Kd = 0; 
  Bvelocity.newerr = 0;
  Bvelocity.newerr1 = 0;  
}


void PID_parameter1(float ah,float bh)
{

  Abalance.Kp = -0.00002*sq(ah)+0.0357*ah+1.9519;     
  Abalance.Ki = 0.0005*sq(ah)+0.1415*ah+11.045;     
  Abalance.Kd = 0; 

  Avelocity.Kp = 0.00001*sq(ah)-0.0025*ah+0.2509;    
  Avelocity.Ki = 0.00003*sq(ah)-0.0053*ah+0.303;     
  Avelocity.Kd = 0;     

  Bbalance.Kp = -0.00002*sq(bh)+0.0357*bh+1.9519;     
  Bbalance.Ki = 0.0005*sq(bh)+0.1415*bh+11.045;     
  Bbalance.Kd = 0; 

  Bvelocity.Kp = 0.00001*sq(bh)-0.0025*bh+0.2509;    
  Bvelocity.Ki = 0.00003*sq(bh)-0.0053*bh+0.303;     
  Bvelocity.Kd = 0;  
  
}


void PID_parameter(float ah,float bh)
{
  Abalance.Kp = 0.00001*sq(ah)+0.0068*ah+0.1028;     
  Abalance.Ki = 0.00002*sq(ah)+0.0396*ah+0.8171;     
  Abalance.Kd = 0; 

  Avelocity.Kp = 0.000002*sq(ah)-0.002*ah+0.539;    
  Avelocity.Ki = -0.002*ah+0.6857;  
  Avelocity.Kd = 0;   

  Bbalance.Kp = 0.00001*sq(ah)+0.0068*ah+0.1028;    
  Bbalance.Ki = 0.00002*sq(ah)+0.0396*ah+0.8171; 
  Bbalance.Kd = 0; 

  Bvelocity.Kp = 0.000002*sq(ah)-0.002*ah+0.539;     
  Bvelocity.Ki = -0.002*ah+0.6857;    
  Bvelocity.Kd = 0;   

  if(Abalance.Kp<1.3)
  Abalance.Kp = 1.3;

  if(Bbalance.Kp<1.3)
  Bbalance.Kp = 1.3;
 
/*
  Bbalance.Kp = Abalance.Kp;     
  Bbalance.Ki = Abalance.Ki;     
  Bbalance.Kd = 0; 

  Bvelocity.Kp = Avelocity.Kp;    
  Bvelocity.Ki = Avelocity.Ki;     
  Bvelocity.Kd = 0; 
 */

}



float Height_pid(float Target,float tilt)
{         
    Height.err =  tilt-Target;

    if(abs(Height.err)<=2)
    {
       Height.err = 0;  
    }

    float p = Height.Kp*(Height.err-Height.err_last);
    float i = Height.Ki*Height.err;
    float d = Height.Kd*(Height.err-2*Height.err_last+Height.err_last1);
      
    Height.pidout = Height.pidout+p+i+d;
   
    Height.err_last = Height.err;
    
    Height.err_last1 = Height.err_last;

    if(Height.pidout>80)
    Height.pidout = 80;  
    else if(Height.pidout<(-80))
    Height.pidout = -80;   
    
    Height.ActualOut = Height.pidout*10;
    
    return Height.ActualOut;
}

float Turn_PID (float Target,float z)
{      
   float zx = T_Sliding_weighted_filter(z);    
   Turn.err = Tfilter((zx-Target));

   //if(abs(Turn.err)<=80)
   //Turn.err=0;

   Turn.pidout=Turn.Kp*Turn.err;         //位置式PID控制器    
   Turn.ActualOut = Turn.pidout*1.0;
   
   //if(Turn.ActualOut>55)
   // Turn.ActualOut = 55;  
   // else if(Turn.ActualOut<(-55))
   //  Turn.ActualOut = -55;             
   return Turn.ActualOut;   
}


float Abalance_pin(float Target,float Xangle)  //平衡环PID
{     
    Abalance.err = Xangle-Target;
       
    Abalance.integral += Abalance.err*0.01;

    if(Abalance.integral>2.5) Abalance.integral = 2.5;   
    if(Abalance.integral<(-2.5)) Abalance.integral = -2.5;   

    Abalance.derivative = gy.y;//(Abalance.err-Abalance.err_last)/0.01;;
    
    Abalance.pidout = Abalance.Kp*Abalance.err+Abalance.Ki*Abalance.integral+Abalance.Kd*Abalance.derivative;
   
    Abalance.err_last = Abalance.err;
    
    Abalance.ActualOut = Abalance.pidout*1.0;
       
    if(Abalance.ActualOut>0) 
    Abalance.ActualOut = Abalance.ActualOut + dead_band;
    else if(Abalance.ActualOut<0) 
    Abalance.ActualOut = Abalance.ActualOut + (-dead_band);

    if(Abalance.ActualOut>150) 
    Abalance.ActualOut = 150;
    else if(Abalance.ActualOut<(-150)) 
    Abalance.ActualOut = -150;
       
    return Abalance.ActualOut;
}

float Bbalance_pin(float Target,float Xangle)  //平衡环PID
{
    Bbalance.err = Abalance.err;//Xangle-Target;
    
    Bbalance.integral += Bbalance.err*0.01;

    Bbalance.derivative = gy.y;
    
    Bbalance.pidout = Bbalance.Kp*Bbalance.err+Bbalance.Ki*Abalance.integral+Bbalance.Kd*Abalance.derivative;
   
    Bbalance.err_last = Bbalance.err;
    
    Bbalance.ActualOut = Bbalance.pidout*1.0;
       
    if(Bbalance.ActualOut>0) 
    Bbalance.ActualOut = Bbalance.ActualOut + dead_band;
    else if(Bbalance.ActualOut<0) 
    Bbalance.ActualOut = Bbalance.ActualOut + (-dead_band);

    if(Bbalance.ActualOut>150) 
    Bbalance.ActualOut = 150;
    else if(Bbalance.ActualOut<(-150)) 
    Bbalance.ActualOut = -150;
       
    return Bbalance.ActualOut;
}

float Avelocity_pin(float Target,float Aspeed,float Bspeed)  //速度环PID控制器
{   
    float newerr =Aspeed+Bspeed; 
    //if(abs(car_ch2)>5)
    //newerr = 0;
    
    newerr = newerr-Target; 
           
    Avelocity.newerr = newerr*0.3+Avelocity.newerr*0.7;
    
    Avelocity.newerr1 = A_Sliding_weighted_filter(Avelocity.newerr);

    A_Velocity_filter(Avelocity.newerr1);

    Avelocity.err = A_speed.Xk;

    //if(abs(car_ch2)<5)
    //{
      Avelocity.integral += Avelocity.err*0.01;
      if(Avelocity.integral>30) Avelocity.integral = 30;   
      if(Avelocity.integral<(-30)) Avelocity.integral = -30;    
    //}

    Avelocity.derivative = (Avelocity.err-Avelocity.err_last)/0.01;
    
    Avelocity.pidout = Avelocity.Kp*Avelocity.err + Avelocity.Ki*Avelocity.integral + Avelocity.Kd*Avelocity.derivative;
          
    Avelocity.ActualOut = Avelocity.pidout*1.0;

    if(Avelocity.ActualOut>22) Avelocity.ActualOut = 22;   
    if(Avelocity.ActualOut<(-22)) Avelocity.ActualOut = -22;
    
    return Avelocity.ActualOut;
}

float Bvelocity_pin(float Target,float Aspeed,float Bspeed)  //速度环PID控制器
{   
    float newerr =Aspeed+Bspeed; 
    //if(abs(car_ch2)>5)
    //newerr = 0;
    
    newerr = newerr-Target;      
    
    Bvelocity.newerr = newerr*0.3+Bvelocity.newerr*0.7;
    
    Bvelocity.newerr1 = B_Sliding_weighted_filter(Bvelocity.newerr);

    B_Velocity_filter(Bvelocity.newerr1);

    Bvelocity.err = B_speed.Xk;

    //if(abs(car_ch2)<5)
    //{
      Bvelocity.integral += Bvelocity.err*0.01;
      if(Bvelocity.integral>30) Bvelocity.integral = 30;   
      if(Bvelocity.integral<(-30)) Bvelocity.integral = -30;    
    //}

    Bvelocity.derivative = (Bvelocity.err-Bvelocity.err_last)/0.01;
    
    Bvelocity.pidout = Bvelocity.Kp*Bvelocity.err + Bvelocity.Ki*Bvelocity.integral + Bvelocity.Kd*Bvelocity.derivative;
          
    Bvelocity.ActualOut = Bvelocity.pidout*1.0;

    if(Bvelocity.ActualOut>22) Bvelocity.ActualOut = 22;   
    if(Bvelocity.ActualOut<(-22)) Bvelocity.ActualOut = -22;
    
    return Bvelocity.ActualOut;
}


int ch_limiter(int x)
{
   if(x>1800)
   x = 1800;
   if(x<170)
   x = 170;
   return x;  
}

void RXsbus()
{    
  sBus.FeedLine();
  if (sBus.toChannels == 1)
  {
    sBus.toChannels = 0;
    //sBus.UpdateServos();
    sBus.UpdateChannels();
    sBus.toChannels = 0;

    sBus.channels[0] = ch_limiter(sBus.channels[0]);
    sBus.channels[1] = ch_limiter(sBus.channels[1]);
    sBus.channels[2] = ch_limiter(sBus.channels[2]);
    sBus.channels[3] = ch_limiter(sBus.channels[3]);
    sBus.channels[4] = ch_limiter(sBus.channels[4]);
    sBus.channels[5] = ch_limiter(sBus.channels[5]);
    sBus.channels[6] = ch_limiter(sBus.channels[6]);
    sBus.channels[7] = ch_limiter(sBus.channels[7]);
    
    sbuschx[0] =  map(sBus.channels[0], 170, 1800, -4400, 4400);
    sbuschx[1] =  map(sBus.channels[1], 170, 1800, 33, -33);
    sbuschx[2] =  map(sBus.channels[2], 170, 1800, 0, 800);
    sbuschx[3] =  map(sBus.channels[3], 170, 1800, -40, 40);
    sbuschx[4] =  map(sBus.channels[4], 170, 1800, 0, 2);
    sbuschx[5] =  map(sBus.channels[5], 170, 1800, 0, 2);


    car_ch1 = sbuschx[0];
    car_ch2 = sbuschx[1];
    car_ch3 = sbuschx[2];
    car_ch4 = sbuschx[3]; 
    car_ch5 = sbuschx[4];
    car_ch6 = sbuschx[5];
    car_ch7 = sbuschx[6];
    car_ch8 = sbuschx[7];   

    if(abs(car_ch1)<111)
    {
      car_ch1 = 0;
    }
    if(abs(car_ch2)<2)
    {
      car_ch2 = 0;
    }
  }  
}


void EEPROM_init()
{
  if (!EEPROM.begin(100)) 
  {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }  
  AngleX_bias = EEPROM.readFloat(30);
  AngleY_bias = EEPROM.readFloat(40);
  Serial.print("AngleX_bias:");
  Serial.print(AngleX_bias);
  Serial.print("  ---  ");
  Serial.print("AngleY_bias:");
  Serial.println(AngleY_bias);  

}


float A_Sliding_weighted_filter(float xdat) //滑动加权滤波算法
{
  long array_sum = 0; //采样队列和                             
  for(int i=1;i<n_sample;i++)
  {
      sample_array[i-1] = sample_array[i];
      array_sum += sample_array[i] * i;
  }  
  sample_array[n_sample-1] = xdat;
  array_sum += xdat * n_sample;
  float filte_value= (array_sum / (11*n_sample/2.0)) * 9 / 7.0; //
  return filte_value;
}

float B_Sliding_weighted_filter(float xdat1) //滑动加权滤波算法
{
  long array_sum = 0; //采样队列和                             
  for(int i=1;i<n_sample1;i++)
  {
      sample_array1[i-1] = sample_array1[i];
      array_sum += sample_array1[i] * i;
  }  
  sample_array1[n_sample1-1] = xdat1;
  array_sum += xdat1 * n_sample1;
  float filte_value= (array_sum / (11*n_sample1/2.0)) * 9 / 7.0; //
  return filte_value;
}

float T_Sliding_weighted_filter(float xdat2) //滑动加权滤波算法
{
  long array_sum = 0; //采样队列和                             
  for(int i=1;i<n_sample2;i++)
  {
      sample_array2[i-1] = sample_array2[i];
      array_sum += sample_array2[i] * i;
  }  
  sample_array2[n_sample2-1] = xdat2;
  array_sum += xdat2 * n_sample2;
  float filte_value= (array_sum / (11*n_sample2/2.0)) * 9 / 7.0; //
  return filte_value;
}

float IMU_Sliding_weighted_filter(float xdat3) //滑动加权滤波算法
{
  long array_sum = 0; //采样队列和                             
  for(int i=1;i<n_sample3;i++)
  {
      sample_array3[i-1] = sample_array3[i];
      array_sum += sample_array3[i] * i;
  }  
  sample_array3[n_sample3-1] = xdat3;
  array_sum += xdat3 * n_sample3;
  float filte_value= (array_sum / (11*n_sample3/2.0)) * 9 / 7.0; //
  return filte_value;
}

float LPF_filter(float x)  //低通滤波
{ 
  LPF_value = LPF_a*LPF_value + (1-LPF_a)*x;
  return LPF_value;
}

boolean crc1(unsigned char buffer[])
{
  unsigned int crc_bit1=0;
  unsigned int sum1=0;
  
  for (int j = 2; j <=5; j++)
  {
    sum1 += buffer[j];
  }
  crc_bit1 = sum1 & 0xff;
  if ((unsigned char)crc_bit1 == buffer[6])
    return true;
  else
    return false;
}

unsigned char crc2(unsigned char buffer[])
{
  unsigned int crc_bit1=0;
  unsigned int sum1=0;
  
  for (int j = 2; j <=5; j++)
  {
    sum1 += buffer[j];
  }
  crc_bit1 = sum1 & 0xff;

  return (unsigned char)crc_bit1;
}


void Set_motor_speed(int MA, int MB)
{   
  int ta = MA+32767;
  int tb = MB+32767;
  txbuf[0]=111;
  txbuf[1]=66;
  txbuf[2]=ta>>8;
  txbuf[3]=ta&0xff;
  txbuf[4]=tb>>8;
  txbuf[5]=tb&0xff;
  txbuf[6]=crc2(txbuf);
  Serial1.write(txbuf,sizeof(txbuf));
}

void Read_motor_speed()
{         
  while (Serial1.available() > 0) 
  {
     dat = Serial1.read();   
     //Serial.println(dat); 
     if((ccnt==0)&&(dat == 111))
     {
       rxbuf[ccnt] = dat;
       ccnt = 1;
     }
     else if((ccnt==1)&&(dat == 66))
     {
       rxbuf[ccnt] = dat;
       recstatu = 1;
       ccnt = 2; 
     }
     else if(recstatu == 1) //表示是否处于一个正在接收数据包的状态
     {
          rxbuf[ccnt] = dat; 
          ccnt++;
          if(ccnt==7)
          {
              if(crc1(rxbuf))
              {
                recstatu = 0;
                packerflag = 1;//用于告知系统已经接收成功
                ccnt = 0;  
                int x1=rxbuf[2];
                x1<<=8;
                x1+=rxbuf[3];
                int x2=rxbuf[4];
                x2<<=8;
                x2+=rxbuf[5];

                //float Amotor_angle = (x1-32767)*0.01;
                //Amotor_speed = (Amotor_angle - Amotor_angle_prev)/Ts;
                //Amotor_angle_prev = Amotor_angle;  
                
                //float Bmotor_angle = -((x2-32767)*0.01);
                //Bmotor_speed = (Bmotor_angle - Bmotor_angle_prev)/Ts;
                //Bmotor_angle_prev = Bmotor_angle;
                
                Amotor_speed = (x1-32767)*0.01;
                Bmotor_speed = -((x2-32767)*0.01);
             
                //Serial.print(Amotor_speed);
                //Serial.print(" ");
                //Serial.print(Bmotor_speed);  
                //Serial.print(" ");
                //Serial.println(Ts,3); 
                //velocity_calc_timestamp = now_us;  
              }
              else
              {
                

                rxbuf[0] = 0;
                rxbuf[1] = 0;
                recstatu = 0;
                packerflag = 0;//用于告知系统已经接收失败
                ccnt = 0;  
                                         
              }                   
          }  
      }
      else
      {
          rxbuf[0] = 0;
          rxbuf[1] = 0;
          recstatu = 0;
          packerflag = 0;//用于告知系统已经接收失败
          ccnt = 0;
      }         
  }
}  


void Serialcommand()
{
  while(Serial.available()>0)
  {
    comdata =Serial.readStringUntil(terminator);
    if(comdata=="wkp")
    {
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Serial.parseFloat());
    }      
    if(comdata=="wki")
    {
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Serial.parseFloat());
    }  
    if(comdata=="wkd")
    {
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Serial.parseFloat());
    }
    
    if(comdata=="adj")
    {
      A_Angle = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(A_Angle);
    }  

    if(comdata=="bdj")
    {
      B_Angle = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(B_Angle);
    }  
    if(comdata=="abh")
    {
      abh = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(abh);
    }  
    ///////////////////////////////
    if(comdata=="a1max")
    {
      A1angle_max = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(A1angle_max);
    }   
    if(comdata=="a1min")
    {
      A1angle_mini = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(A1angle_mini);
    } 
    /////////////////////////////////
    if(comdata=="a2max")
    {
      A2angle_max = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(A2angle_max);
    }   
    if(comdata=="a2min")
    {
      A2angle_mini = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(A2angle_mini);
    } 
    //////////////////////////////////
    if(comdata=="b1max")
    {
      B1angle_max = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(B1angle_max);
    }   
    if(comdata=="b1min")
    {
      B1angle_mini = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(B1angle_mini);
    }  
    ////////////////////////////////////
    if(comdata=="b2max")
    {
      B2angle_max = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(B2angle_max);
    }   
    if(comdata=="b2min")
    {
      B2angle_mini = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(B2angle_mini);
    }  
    ////////////////////////////////////
    if(comdata=="apkp")
    {
      Abalance.Kp = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Abalance.Kp,2);
    }   
    if(comdata=="apki")
    {
      Abalance.Ki = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Abalance.Ki,4);
    } 
    if(comdata=="apkd")
    {
      Abalance.Kd = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Abalance.Kd,4);
    }  
    ////////////////////////////////////
        if(comdata=="askp")
    {
      Avelocity.Kp = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Avelocity.Kp,4);
    }   
    if(comdata=="aski")
    {
      Avelocity.Ki = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Avelocity.Ki,4);
    } 
    if(comdata=="askd")
    {
      Avelocity.Kd = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Avelocity.Kd,4);
    }  
    if(comdata=="tkp")
    {
      Turn.Kp = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Turn.Kp,4);
    } 
    if(comdata=="tki")
    {
      Turn.Ki = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Turn.Ki,4);
    } 
    if(comdata=="hkp")
    {
      Height.Kp = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Height.Kp,4);
    }    
    if(comdata=="hki")
    {
      Height.Ki = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Height.Ki,4);
    }  
    if(comdata=="hkd")
    {
      Height.Kd = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(Height.Kd,4);
    }  

    if(comdata=="sq")
    {
      dead_band = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(dead_band);
    } 
    
    if(comdata=="ta")
    {
      T_a = Serial.parseFloat();
      Serial.print(comdata);
      Serial.print('=');
      Serial.println(T_a,3);
    }  
         
  }
  comdata = "";    
}

void Set_servo_Angle(float aangle,float bangle)
{

  float a_angle = aangle;
  float b_angle = bangle;
  
  if(a_angle<0) a_angle = 0;
  if(a_angle>800) a_angle = 800;

  if(b_angle<0) b_angle = 0;
  if(b_angle>800) b_angle = 800;
  
  float a1 = a_angle*0.1;
  if(a1<1)
  a1 = 1;
  float b1 = b_angle*0.1;
  if(b1<1)
  b1 = 1;

  float ah1 = sin(a1*PI/180)*90;
  float al1 = cos(a1*PI/180)*90+18;
  float ah2 = sqrt(14400 - al1 * al1);
  AH = 140+ah1+ah2-53.9;

  float bh1 = sin(b1*PI/180)*90;
  float bl1 = cos(b1*PI/180)*90+18;
  float bh2 = sqrt(14400 - bl1 * bl1);
  BH = 140+bh1+bh2-53.9;
  /*
  ledcWrite(channel0,B1angle_mini);  //B1
  ledcWrite(channel1,B2angle_mini);  //B2
  ledcWrite(channel2,A2angle_mini);  //A2
  ledcWrite(channel3,A1angle_mini);  //A1
  */
  
  ledcWrite(channel0,map(b_angle, 0, 900, B1angle_mini, B1angle_max));  //B1
  ledcWrite(channel1,map(b_angle, 0, 900, B2angle_mini, B2angle_max));  //B2 
  ledcWrite(channel2,map(a_angle, 0, 900, A2angle_mini, A2angle_max));  //A2
  ledcWrite(channel3,map(a_angle, 0, 900, A1angle_mini, A1angle_max));  //A1
     
  
}

void Servo_Initialization() // ledc初始化程序
{
  ledcSetup(channel0, freq, resolution0); // 设置通道0
  ledcSetup(channel1, freq, resolution0); // 
  ledcSetup(channel2, freq, resolution0); // 
  ledcSetup(channel3, freq, resolution0); // 

  ledcAttachPin(B1_PWM, channel0);  // 将通道0与引脚13连接
  ledcAttachPin(B2_PWM, channel1);  // 
  ledcAttachPin(A2_PWM, channel2);  // 
  ledcAttachPin(A1_PWM, channel3);  // 
  
  Set_servo_Angle(A_Angle,B_Angle);
  delay(1111);
}



void Keypad_init()
{
  pinMode(k1pin, INPUT_PULLUP);  
  pinMode(k2pin, INPUT_PULLUP);  
  pinMode(k3pin, INPUT_PULLUP);    
}

void Keypad_detection()
{ 
  boolean K1 = digitalRead(k1pin);
  boolean K2 = digitalRead(k2pin);
  boolean K3 = digitalRead(k3pin);

  if((car_ch5==0)||(car_ch5==1))
  {
    ch5_press_time++;
    if(ch5_press_time>50)
    {
      ch5_press_time=0;
      start_f = 0;
    }    
  }
  if(car_ch5==2)
  {
    ch5_press_time++;
    if(ch5_press_time>50)
    {
      ch5_press_time=0;
      start_f = 1;
    }    
  }
  
  if(K1)          
  {
    s1_press_time=0;
    s1_long_press=0;
    if(s1)           
    {            
      s1=0;       
      key1=1;
    } 
  }
  else 
  {
    s1=1;
    s1_press_time++;
    if(s1_press_time>50)
    {
      s1_press_time=0;s1=0;
      s1_long_press=1;
    }
  }         
  
  if(K2)          
  {
    s2_press_time=0;
    s2_long_press=0;
    if(s2)           
    {            
      s2=0;       
      key2=1;
    } 
  }
  else 
  {
    s2=1;
    s2_press_time++;
    if(s2_press_time>50)
    {
      s2_press_time=0;s2=0;
      s2_long_press=1;
    }
  }
  
  if(K3)          
  {
    s3_press_time=0;
    s3_long_press=0;
    if(s3)           
    {            
      s3=0;       
      key3=1;
    } 
  }
  else 
  {
    s3=1;
    s3_press_time++;
    if(s3_press_time>50)
    {
      s3_press_time=0;s3=0;
      s3_long_press=1;
    }
  } 


  if(key1)
  {
    key1 = 0;
    show_f++;  
    if(show_f>7)show_f = 7;
  }
  if(key2)
  {
    key2 = 0;
    show_f--; 
    if(show_f<0)show_f = 0; 
  }
  

   switch (show_f)
   {
    case 0:

            Serial.print(" ch1:"); 
            Serial.print(car_ch1);   
            Serial.print(" ch2:"); 
            Serial.print(car_ch2);   
            Serial.print(" ch3:"); 
            Serial.print(car_ch3);   
            Serial.print(" ch4:"); 
            Serial.print(car_ch4);   
            Serial.print(" ch5:"); 
            Serial.print(car_ch5);   
            Serial.print(" ch6:"); 
            Serial.print(car_ch6);   
            Serial.print(" ch7:"); 
            Serial.print(car_ch7);   
            Serial.print(" ch8:");   
            Serial.print(car_ch8);  
            Serial.print("\t");
            Serial.print(Power_Voltage);    
            Serial.print(" Ts:");
            Serial.println(Ts,3);      
            break;
    case 1:   
            Serial.print((KalmanY.Angle-AngleY_bias),3);
            Serial.print("\t");
            Serial.print(AngleY,3);
            Serial.print("\t");
            Serial.print(AngleX,3);
            //float y1 = (float)(-gy.y/16.4);
            //float x1 = (float)(gy.x/16.4);
            //Serial.print("\t");
            //Serial.print(KalmanZ.Rate);
            //Serial.print("\t");
            //Serial.print(KalmanZ.Rate2);
            //Serial.print("\t");
            //Serial.print(Turn.ActualOut);
            //Serial.print("aworld\t");
            
            //Serial.print(Hcm);
            //Serial.print("\t");
            //Serial.print(aaWorld.z);
            Serial.println("\t");   
            break;
    case 2:
            Serial.print(KalmanY.Angle);
            Serial.print("\t");
            Serial.print(AngleY_bias);
            Serial.print("\t");
            Serial.print(KalmanY.Angle-AngleY_bias);
            Serial.print("\t");
            Serial.print(KalmanX.Angle);
            Serial.print("\t");
            Serial.print(AngleX_bias);
            Serial.print("\t");
            Serial.print(KalmanX.Angle-AngleX_bias);
            Serial.println("\t");    
            if(key3)
            {
              key3 = 0;
              EEPROM.writeFloat(30,AngleX);
              EEPROM.writeFloat(40,AngleY);
              EEPROM.commit();
              AngleX_bias = EEPROM.readFloat(30);
              AngleY_bias = EEPROM.readFloat(40);
            }
                    
            break;
    case 3:
            Serial.print(Bvelocity.err);
            Serial.print("\t"); 
            Serial.print(Avelocity.err);
            Serial.println("\t");  
            break; 

    case 4:
            //Serial.print(" Hpwm:"); 
            //Serial.print(Height.ActualOut); 

            Serial.print(" Herr:"); 
            Serial.print(Height.err); 

            //Serial.print(" te:"); 
            //Serial.print(Turn.err); 
            
            //Serial.print(" Tpwm:"); 
            //Serial.print(Turn.ActualOut); 

            Serial.print(" Apwm:"); 
            Serial.print(Abalance.ActualOut); 
            
            Serial.print(" Bpwm:"); 
            Serial.print(Bbalance.ActualOut); 

            Serial.print(" AI:"); 
            Serial.print(Abalance.integral); 
            
            Serial.print(" BI:"); 
            Serial.print(Bbalance.integral); 

            Serial.print(" SApwm:"); 
            Serial.print(Avelocity.ActualOut); 
            
            Serial.print(" SBpwm:"); 
            Serial.print(Bvelocity.ActualOut); 
            
            Serial.print(" Ts:");
            Serial.println(Ts,3);

            if(key3)
            {
               key3 = 0;
               if(start_f==1) start_f = 0;
               else if(start_f==0) start_f = 1;              
            }
    
            break;     
            
    case 5:
            Serial.print(" APp:");
            Serial.print(Abalance.Kp,3);
            Serial.print(" APi:");
            Serial.print(Abalance.Ki,3);    
            Serial.print(" ASp:");
            Serial.print(Avelocity.Kp,3);
            Serial.print(" ASi:");
            Serial.print(Avelocity.Ki,3);

            Serial.print(" BPp:");
            Serial.print(Bbalance.Kp,3);
            Serial.print(" BPi:");
            Serial.print(Bbalance.Ki,3);    
            Serial.print(" BSp:");
            Serial.print(Bvelocity.Kp,3);
            Serial.print(" BSi:");
            Serial.print(Bvelocity.Ki,3);
            Serial.print(" AH:");
            Serial.println(Bvelocity.Ki,3);
            
            break;        

    case 6:
    
            Serial.print(AngleY);
            Serial.print("\t");
            Serial.print(Abalance.ActualOut);
            Serial.print("\t");
            Serial.print(Avelocity.err);
            Serial.print("\t");
            Serial.print(Avelocity.ActualOut);
            Serial.println("\t");        
            
            break;   
            
    case 7:

            
            //Serial.print(Abalance.err,3);
            //Serial.println("\t");  
            break;              
                              
    default:  
              ;//printf("default!\n");
        break;
  }    
}

void ReadVoltage()
{
   Power_Voltage = analogRead(ANALOG_PIN_ch6);//LPF_filter(analogRead(ANALOG_PIN_ch6));//(范围从0到4096)
   Power_Voltage = (3.3/4096)*Power_Voltage*6.1*1.059;    
}


void OledInt()
{
  u8g2.begin();    //初始化函数
  u8g2.enableUTF8Print();        // 为Arduino print（）函数启用UTF8支持 

  u8g2.setFont(u8g2_font_wqy12_t_gb2312);  //对“你好世界”的所有字形使用chinese2

  u8g2.setFontDirection(0);  
}

void OledShow()
{
  if(key1)
  {
    key1 = 0;
    show_f++;  
    if(show_f>3)show_f = 3;
  }
  if(key2)
  {
    key2 = 0;
    show_f--; 
    if(show_f<0)show_f = 0; 
  }
  
  u8g2.firstPage();
  do {
       switch (show_f)
       {
        case 0:
                  u8g2.setCursor(38,15);
                  u8g2.print("国炫学长");
                  u8g2.setCursor(49, 32);
                  u8g2.print("Hyun");        // Chinese "Hello World"         
            break;
        case 1:
                  u8g2.setCursor(0,15);  
                  u8g2.print("X");
                  u8g2.setCursor(9,14);  
                  u8g2.print(":");
                  u8g2.setCursor(12,15);    
                  u8g2.print(AngleX);
                  
                  //u8g2.setCursor(52,15);   
                  //u8g2.print(kalmanX.getRate());
                  
                  u8g2.setCursor(0, 32);
                  u8g2.print("Y");
                  u8g2.setCursor(9,31);  
                  u8g2.print(":");
                  u8g2.setCursor(12,32);  
                  u8g2.print(AngleY);

                  //u8g2.setCursor(52,32);  
                  //u8g2.print(kalmanY.getRate());
            
                  u8g2.setCursor(91, 23);
                  u8g2.print(Power_Voltage);   
                  u8g2.print("V");     
                  if(key3)
                  {
                    key3 = 0;
                    EEPROM.writeFloat(30,AngleX);
                    EEPROM.writeFloat(40,AngleY);
                    EEPROM.commit();
                    AngleX_bias = EEPROM.readFloat(30);
                    AngleY_bias = EEPROM.readFloat(40);
                  }
                        
            break;
        case 2:
                  u8g2.setCursor(0,15);  
                  u8g2.print("A");
                  u8g2.setCursor(9,14);  
                  u8g2.print(":");
                  u8g2.setCursor(12,15);    
                  u8g2.print(Amotor_speed);

                  u8g2.setCursor(0,32);  
                  u8g2.print("B");
                  u8g2.setCursor(9,31);  
                  u8g2.print(":");
                  u8g2.setCursor(12,32);    
                  u8g2.print(Bmotor_speed);
        
            break; 
        case 3:
                  u8g2.setCursor(0,15);  
                  u8g2.print("Aout");
                  u8g2.print(":");
                  u8g2.print(Abalance.pidout,4);               
                  
                  u8g2.setCursor(0, 32);
                  u8g2.print("KP");
                  u8g2.print(":");
                  u8g2.print(Abalance.Kp,3);
        
            break;                  
        default:  
                  ;//printf("default!\n");
            break;
        }
  } while ( u8g2.nextPage() );
}


int Mpu6050Int()
{
  Wire.begin(19,22, 400000);//SDA1,SCL1
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setSleepEnabled(false);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-1);
  mpu.setYGyroOffset(80);
  mpu.setZGyroOffset(-58);
  mpu.setXAccelOffset(-404);
  mpu.setYAccelOffset(-2764);
  mpu.setZAccelOffset(1218);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    mpuIntStatus = mpu.getIntStatus();
    imuReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delay(2000);
  Serial.println(F("Adjusting DMP sensor fusion gain..."));
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0x20);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
  
  return imuReady;
}

int hasDataIMU(){
  return imuReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}


void Mpu6050Read()
{  
  
   // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float X = ypr[2] * 180 / M_PI;
    float Y = ypr[1] * 180 / M_PI;
    float Z = ypr[0] * 180 / M_PI;
     
    AngleX = X;
    AngleY = Y;  
    AngleZ = Z;  

    if(AngleX > 33)
    AngleX = 33;
    
    if(AngleX < (-33))
    AngleX = -33;

    if(AngleY > 33)
    AngleY = 33;
    
    if(AngleY < (-33))
    AngleY = -33;
    
    mpu.dmpGetGyro(&gy, fifoBuffer);
    float y1 = (float)(-gy.y/16.4);
    float x1 = (float)(gy.x/16.4);
    float z1 = (float)(-gy.z/16.4);
    
    YKalmangGetAngle(AngleY, y1, Ts);
    XKalmangGetAngle(AngleX, x1, Ts);
    ZKalmangGetAngle(AngleZ, z1, Ts);
/////////////////////////////////////////////////////////////////

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);   
}

void setup() 
{
  Serial.begin(500000);
  Serial1.begin(500000, SERIAL_8N1, RXD1, TXD1);
  sBus.begin();
  
  //Serial.print("core count:");
  //Serial.println(portNUM_PROCESSORS);//输出可用的核心数量，应是2  
  pid_init();
  Keypad_init(); 
  //OledInt();
  Set_motor_speed(0,0);

  if ( !Mpu6050Int() ) 
  {
    Serial.println(F("IMU connection problem... Disabling!"));
    return;
  }
  
  Kalman_init();
  AB_Velocity_filter_init();
  //创建任务
  //xTaskCreatePinnedToCore(Task1,"Task1",100000,NULL,3,NULL ,1);
  
  //xTaskCreatePinnedToCore(Task2,"Task2",70000,NULL,2,NULL ,1);
  
  //xTaskCreatePinnedToCore(Task3,"Task3",40000,NULL,1,NULL ,1);//

  //Serial.println(ESP.getFreeHeap());
  delay(111);
  Servo_Initialization(); // ledc初始化程序 
  EEPROM_init();
  now_us = micros();
  velocity_calc_timestamp = now_us; 
}


void loop() 
{
  now_us = micros();
  Ts = (now_us - velocity_calc_timestamp)*1e-6;
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 
  
  if (Ts>=0.01) //if (mpuInterrupt)
  {      
      velocity_calc_timestamp = now_us; 
      RXsbus();
      if(mpuInterrupt)
      {
        if (hasDataIMU()) 
        {
          Mpu6050Read();
        }     
        mpuInterrupt = false;      
      }

      Read_motor_speed(); 
      Hcm =((IMU_Sliding_weighted_filter(aaWorld.z)-8500)/16.4*Ts);
      if(Power_Voltage<=11)
      {
        //start_f = 0;  
        //Serial.print(Power_Voltage);
        //Serial.print(" ");
        //Serial.print("no electricity ");
      }
      
      ff++;
      if(ff>100)
      {
        ff=0;
        //ff1=random(-10000,10000);
        //ff2=random(-90,90);
      }
      else
      {
        ff1=0;
        ff2=0;
      }

      
      if(start_f==1)
      { 
        int Tpwm = (int)Turn_PID (car_ch1,KalmanZ.Rate);//-gy.z/16.4     

        float asd = Avelocity_pin(car_ch2,Amotor_speed,Bmotor_speed);
        float bsd = Bvelocity_pin(car_ch2,Amotor_speed,Bmotor_speed);
        
        int Apwm = (int)(Abalance_pin(asd,(KalmanY.Angle-AngleY_bias))*100)-Tpwm;
        int Bpwm = (int)(Bbalance_pin(bsd,(KalmanY.Angle-AngleY_bias))*100)+Tpwm;   

        if(Apwm>15000)
        Apwm = 15000;

        if(Apwm<(-15000))
        Apwm = -15000;

        if(Bpwm>15000)
        Bpwm = 15000;

        if(Bpwm<(-15000))
        Bpwm = -15000;
 
        int acz = abs(Apwm-Apwm1);
        int bcz = abs(Bpwm-Bpwm1);
        
        //Serial.print(acz);
        //Serial.println("\t");     
        //Serial.println(KalmanY.Angle);
        Set_motor_speed(-Apwm,-Bpwm);
        Apwm1 = Apwm;
        Bpwm1 = Bpwm;
        //observe();
      }
      else 
      {
        Set_motor_speed(0,0);

        Amotor_speed = 0;
        Bmotor_speed = 0;
        
        dead_band=0;   //死去范围
        //平衡环PID
        Abalance.ActualOut = 0; 
        Abalance.err = 0;  
        Abalance.err_last = 0;
        Abalance.pidout = 0;
        Abalance.integral = 0;
        //速度环PID
        Avelocity.ActualOut = 0;
        Avelocity.err = 0;
        Avelocity.err_last = 0;
        Avelocity.pidout = 0;
        Avelocity.integral = 0;
        Avelocity.newerr = 0;
        Avelocity.newerr1 = 0;

        //平衡环PID
        Bbalance.ActualOut = 0; 
        Bbalance.err = 0;  
        Bbalance.err_last = 0;
        Bbalance.pidout = 0;
        Bbalance.integral = 0;
        //速度环PID
        Bvelocity.ActualOut = 0;
        Bvelocity.err = 0;
        Bvelocity.err_last = 0;
        Bvelocity.pidout = 0;
        Bvelocity.integral = 0;
        Bvelocity.newerr = 0;
        Bvelocity.newerr1 = 0;

        float Tpwm = Turn_PID(car_ch1,KalmanZ.Rate1);//-gy.z/16.4     
        
        int Apwm =(int)(Abalance_pin(Avelocity_pin(0,Amotor_speed,Bmotor_speed),(KalmanY.Angle-AngleY_bias))*100);
        int Bpwm =(int)(Bbalance_pin(Bvelocity_pin(0,Amotor_speed,Bmotor_speed),(KalmanY.Angle-AngleY_bias))*100);
      }       
      //mpuInterrupt = false;   
  

    ReadVoltage();
    Keypad_detection();
    ch4 = Bfilter(car_ch4);
    float c3 = Afilter(car_ch3);
  
    if(car_ch5==1)
    {
      ch5_press_time++;
      if(ch5_press_time>50)
      {
        ch5_press_time=0;
      }    
      //A_Angle = 0;
      //B_Angle = 0;    
    }
    if(car_ch5==2)
    {
      ch5_press_time++;
      if(ch5_press_time>50)
      {
        ch5_press_time=0;
      } 
      //A_Angle = 800;
      //B_Angle = 800;        
    }
    
    if(car_ch6==1)
    {
      A_Angle = c3-ch4*10;
      B_Angle = c3+ch4*10;   
    }
    else if((car_ch6==2)&&(c3<=400))
    {
      float Hpwm = Height_pid(ch4,(KalmanX.Angle-AngleX_bias));
      A_Angle = c3+Hpwm;
      B_Angle = c3-Hpwm; 
    }
    
    if(A_Angle>800)A_Angle=800;
    if(A_Angle<0)A_Angle=0;
    
    if(B_Angle>800)B_Angle=800;
    if(B_Angle<0)B_Angle=0;
    
    Set_servo_Angle(A_Angle,B_Angle);
    if(abs(KalmanY.Angle-AngleY_bias)>30)
    {
      jdbz_press_time1++; 
      if(jdbz_press_time1>20)
       {
         jdbz_press_time1 = 0;
         jdbz = 1; 
       } 
    }
  
  
    if(abs(KalmanY.Angle-AngleY_bias)<1)
    {
       jdbz_press_time++;  
       if(jdbz_press_time>20)
       {
         jdbz_press_time = 0;
         jdbz_press_time1 = 0;
         jdbz = 0; 
       }
    }
  
    if(jdbz==1)
    {
      pidcs();
    }
    else
    {
      PID_parameter(AH,BH);    
    }
    //PID_parameter1(map(A_Angle,0,800,0, 80),map(B_Angle,0,800,0, 80));
    Serialcommand(); 
    //delay(8);
  }  
}
