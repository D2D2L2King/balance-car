//接线：
/*PWMA->D5*/
/*PWMB->D11*/
/*STBY->D9;高电平为工作状态，低电平为待机状态*/
/*AIN1->D8*/
/*AIN2->D7*/
/*BIN1->D10*/
/*BIN2->D4*/
/*scl(iic)->A5*/
/*sda(iic)->A4*/
/*E2A->D3(中断)->L*/
/*E1A->D2(中断)->R*/
/*E2B->D12*/
/*E1B->D13*/
#include"math.h"
#include"I2Cdev.h"
#include"MPU6050_6Axis_MotionApps20.h"
static float target_a=3.38;//目标角度
MPU6050 mpu;//实例化一个mpu6050对象，对象名为mpu
int16_t ax,ay,az,gx,gy,gz;

/***************angle data************************/
float Gyro_y;//y轴陀螺仪数据暂存
float Gyro_x;
float Gyro_z;
float angleAx;
float angle6;
float K1=0.05;//对加速度计取值的权重
float Angle;//一阶互补滤波算法计算出的小车最终倾斜角度
float accelz=0;

/*************kalman_filter***********************/
float P[2][2]={{1,0},
{0,1}
 };
float Pdot[4]={0,0,0,0};
float Q_angle=0.001,Q_gyro=0.005;//角度数据置信度，角速度数据置信度
float R_angle=0.5,C_0=1;
float q_bias,angle_err,PCt_0,PCt_1,E,K_0,K_1,t_0,t_1;
float timeChange=5;//滤波法采样时间间隔毫秒
float dt=timeChange*0.001;//注意：dt的取值为滤波器采样时间

void Angleget()
{
  //平衡参数
  Angle=atan2(ay,az)*57.3;//角度计算公式
  Gyro_x=(gx-128.1)/131;//角度转换
  Kalman_Filter(Angle,Gyro_x);//卡尔曼滤波
  //旋转角度z轴参数
  if(gz>32768)gz-=65536;//强制转换2g 1g
  Gyro_z=-gz/131;//z轴参数转换
  accelz=az/16.4;

  angleAx=atan2(ax,az)*180/PI;//计算与x轴夹角
  Gyro_y=-gy/131.00;//计算角速度
  //一阶互补滤波
  angle6=K1*angleAx+(1-K1)*(angle6+Gyro_y*dt);
  
  }


/**************kalman*********************************/
float angle,angle_dot;  //平衡角度值
void Kalman_Filter(double angle_m,double gyro_m)
{
  angle+=(gyro_m-q_bias)*dt;
  angle_err=angle_m-angle;
  Pdot[0]=Q_angle-P[0][1]-P[1][0];
  Pdot[1]=-P[1][1];
  Pdot[2]=-P[1][1];
  Pdot[3]=Q_gyro;
  P[0][0]+=Pdot[0]*dt;
  P[0][1]+=Pdot[1]*dt;
  P[1][0]+=Pdot[2]*dt;
  P[1][1]+=Pdot[3]*dt;
  PCt_0=C_0*P[0][0];
  PCt_1=C_0*P[1][0];
  E=R_angle+C_0*PCt_0;
  K_0=PCt_0/E;
  K_1=PCt_1/E;
  t_0=PCt_0;
  t_1=C_0*P[0][1];
  P[0][0]-=K_0*t_0;
  P[0][1]-=K_0*t_1;
  P[1][0]-=K_1*t_0;
  P[1][1]-=K_1*t_1;
  angle+=K_0*angle_err;//角度
  q_bias+=K_1*angle_err;
  angle_dot=gyro_m-q_bias;//角速度
  
  }

/************mpu6050初始化函数*********************/
void mpu_init(void){
   Wire.begin();//开启总线并以主机加入总线
   Serial.begin(9600);
   delay(1000);
   mpu.initialize();//初始化mpu6050
  }
/****************mpu6050获取数据**************************/
void mpu_get(void){
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);//iic获取6050六轴数据
  Angleget();//获取angle角度和卡尔曼滤波
//Serial.print(ax);Serial.print(",");
//Serial.print(ay);Serial.print(",");
//Serial.print(az);Serial.print("---");
//Serial.print(angle);Serial.print(",");
//Serial.print(angle_dot);Serial.print(",");
//  Serial.println(angle6);//angle6为所需要的角度数据
  }

  
/****************电机控制函数****************************/
void motion(float pct/*转速百分比*/){//-100<=pct<=100
//接线：
/*PWMA->D5*/
/*PWMB->D11*/
/*STBY->D9;高电平为工作状态，低电平为待机状态*/
/*AIN1->D8*/
/*AIN2->D7*/
/*BIN1->D10*/
/*BIN2->D4*/
/*E2A->D3(中断)*/
/*E1A->D2(中断)*/
/*E2B->D12*/
/*E1B->D13*/
  if(pct>0)//电机正转
  {
    digitalWrite(8,HIGH);//AIN1
    digitalWrite(7,LOW);//ASIN2
    digitalWrite(9,HIGH);//stby
    digitalWrite(10,HIGH);//BIN1
    digitalWrite(4,LOW);//BIN2
    }
  if(pct<0)//电机反转
  {
    digitalWrite(8,LOW);
    digitalWrite(7,HIGH);
    digitalWrite(9,HIGH);//stby
    digitalWrite(10,LOW);
    digitalWrite(4,HIGH);    
    }

  if(pct==0)
  { 
    digitalWrite(8,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(9,HIGH);//stby
    digitalWrite(10,HIGH);
    digitalWrite(4,HIGH);   
    
    }
   if(abs(pct)>=-0.4 && abs(pct)<=0.4){
  analogWrite(11,0);
  analogWrite(5,0);
  }
  else{
  analogWrite(11,abs(pct)+9);
  analogWrite(5,abs(pct)+9);
  }
}
/********************引脚初始化函数***************/
void Pin_init(void){
   pinMode(2,INPUT);
   pinMode(3,INPUT);
   pinMode(4,OUTPUT);
   pinMode(5,OUTPUT);
   pinMode(6,OUTPUT);
   pinMode(7,OUTPUT);
   pinMode(8,OUTPUT);
   pinMode(9,OUTPUT);
   pinMode(10,OUTPUT);
   pinMode(11,OUTPUT);
   pinMode(12,INPUT);
   pinMode(13,INPUT);
  }

/******************中断初始化函数**************/
void interrupt_init(void){
  /*函数原型：attachInterrupt(interrupt, ISR, mode)*/
  //interrupt:中断号
  //ISR：中断处理函数（无参数无返回值）
  //mode:中断触发模式
  //low:低电平触发
  //change:管脚状态改变
  //rising：上升沿触发
  //falling：下降沿触发
  
  attachInterrupt(0,encoderR_R/*右*/,RISING);
  attachInterrupt(0,encoderR_F/*右*/,FALLING);
  attachInterrupt(1,encoderL_R/*左*/,RISING);
  attachInterrupt(1,encoderL_F/*右*/,FALLING);
  }
/*中断处理函数*/
static float encoderA=0; 
/*E1B->D12*/
void encoderR_R(void){
  if(LOW==digitalRead(12))
  encoderA++;
  if(HIGH==digitalRead(12))
  encoderA--;
  }
void encoderR_F(void){
  if(HIGH==digitalRead(12))
  encoderA++;
  if(LOW==digitalRead(12))
  encoderA--;
  }
static float encoderB=0;
/*E2B->D13*/
void encoderL_R(void){
  if(HIGH==digitalRead(13))
  encoderB++;
  if(LOW==digitalRead(13))
  encoderB--;
  }
void encoderL_F(void){
  if(LOW==digitalRead(13))
  encoderB++;
  if(HIGH==digitalRead(13))
  encoderB--;
  }

/********速度解算函数*****************************/
static float encoderA_v=0;
static float encoderB_v=0;
 static float V=0;
void v_get(void){//速度->V
  encoderA_v=(125*encoderA)/390;
  encoderB_v=(125*encoderB)/390;
  encoderA=0;
  encoderB=0;
  V=(encoderA_v+encoderB_v)/2;
  }

  
/***********************PID控制器函数*********************/
float Pi(float target_v,float v){
  float Kp=3.55;//3.95;
  float Ki=0.04;
  float OutPut;
  static float A_pi;
  static float B_pi;
  A_pi=Kp*(target_v-v);
  B_pi=B_pi+Ki*(target_v-v);
  if((target_v-v)>=15 || (target_v-v)<=-15)
  B_pi=0;
  OutPut=A_pi+B_pi;
  return OutPut;
  }

float Pd(float target_angle,float angle){
  float OutPut;
  float Kp=6;//5.2;
  float Kd=3.3;//2.8;
  static float last_angle;
  static float A_pd;
  static float B_pd;
  A_pd=Kp*(target_angle-angle);
  B_pd=Kd*(angle-last_angle);
  OutPut=A_pd+B_pd;
  last_angle=angle;
  if(angle<=(target_a+0.3) && angle>=(target_a-0.3))
  OutPut=0;
  return OutPut;
  }



/******************************************************************************用户代码*******************************************************/
void setup() {
   Pin_init();//引脚初始化
   mpu_init();//mpu6050初始化，arduino主机加入总线
   interrupt_init();//初始化编码器外部中断
}



void loop() {
mpu_get();//获取角度数据，加速度信息
v_get();//获取速度->V
motion(Pi(Pd(target_a,angle6),V));//串级pid
/******test***********/
//motion(Pi(0.2,V));
//Serial.print(ax);Serial.print(",");
//Serial.print(ay);Serial.print(",");
//Serial.print(az);//Serial.print("---");
//Serial.print(angle);Serial.print(",");
Serial.print(angle_dot);Serial.print(",");
Serial.println(angle6);//angle6为所需要的角度数据
/********************/

delay(8);
}
