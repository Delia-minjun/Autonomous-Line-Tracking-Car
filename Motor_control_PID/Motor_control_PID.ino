#include <SSD1306.h>
#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#include <PS2X_lib.h>         //定时中断头文件库
#include <Wire.h>
// 引入驱动OLED0.96所需的库
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//显示屏
#define SCREEN_WIDTH 128 // 设置OLED宽度,单位:像素
#define SCREEN_HEIGHT 64 // 设置OLED高度,单位:像素
// 自定义重置引脚,虽然教程未使用,但却是Adafruit_SSD1306库文件所必需的
#define OLED_RESET 4
/***********电机控制板引脚定义************/
unsigned int Motor_BIN1=11;//控制电机的方向引脚 
unsigned int Motor_BIN2=5;//控制电机的方向引脚 
unsigned int Motor_AIN2=6;//控制电机的方向引脚  
unsigned int Motor_AIN1=4;//控制电机的方向引脚 
//中间
unsigned int Sensor_MID_R2=28;
unsigned int Sensor_MID_R1=26; 
unsigned int Sensor_MID_M=14; 
unsigned int Sensor_MID_L1=15; 
unsigned int Sensor_MID_L2=16;  
//右侧
unsigned int Sensor_RIGHT_R2=52;
unsigned int Sensor_RIGHT_R1=50;  
unsigned int Sensor_RIGHT_L1=48; 
unsigned int Sensor_RIGHT_L2=46; 
//左侧
unsigned int Sensor_LEFT_R2=30;
unsigned int Sensor_LEFT_R1=19;  
unsigned int Sensor_LEFT_L1=18; 
unsigned int Sensor_LEFT_L2=17; 
//中间的传感器黑1白0
int MID_val1=0;
int MID_val2=0;
int MID_val3=0;
int MID_val4=0;
int MID_val5=0; 
//右侧
int Right_val1=0;
int Right_val2=0;
int Right_val3=0;
int Right_val4=0;
//左侧
int Left_val1=0;
int Left_val2=0;
int Left_val3=0;
int Left_val4=0;

String Target_Value;//串口获取的速度字符串变量
int value_A,value_B;//用于存储通过PI控制器计算得到的用于调整电机转速的PWM值的整形变量 

//测距传感器
int inputPin = 13; // 超声波信号接收引脚 ECHO 
int outputPin = 12; // 超声波信号发射引脚 TRIG 

float distance = 0;
int average = 0;
float lastAverage=0;
float output;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 滑动平均滤波参数
const int numSamples = 20;         // 滑动窗口大小
float readings[numSamples];          // 用于存储最近的n次读数
int readIndex = 0;                 // 当前读数索引
double total = 0;                     // 平均值输出
//异常值处理
const float spikeThreshold = 100.0;  // 跳变阈值（单位：cm）
/***********编码器引脚************/
#define DIRECTION_R 7
#define ENCODER_L 3
#define ENCODER_R 2 
#define DIRECTION_L 8 
int Velocity_Left,Velocity_Right=0;
volatile long Velocity_L,Velocity_R;//Count计数变量 Velocity存储设定时间内A相上升沿和下降沿的个数
/***********PID控制器相关参数************/
float Velocity_KP =10, Velocity_KI= 10;
//Velocity_KP,Velocity_KI.PI参数 
volatile float Target=2.0;//目标值
static float Bias_L,Bias_R,PWM_L=0,PWM_R=0,Last_bias_L=0,Last_bias_R=0;   
/*********** 限幅************
*以下两个参数让输出的PWM在一个合理区间
*当输出的PWM小于50时电机不转 所以要设置一个启始PWM
*arduino mega 2560 单片机的PWM不能超过255 所以 PWM_Restrict 起到限制上限的作用
*****************************/
int startPWM=10;//初始PWM，暂时不用
int PWM_Restrict=255;//startPW+PWM_Restric=255<256

/***********初始化************/
void setup() 
{
  Serial.begin(115200);//打开串口
  // Serial.println("/*****START*****/");
  pinMode(ENCODER_R,INPUT);//设置两个相线为输入模式
  pinMode(ENCODER_L,INPUT);
  pinMode(DIRECTION_R,INPUT);//设置两个相线为输入模式
  pinMode(DIRECTION_L,INPUT);
  pinMode(Motor_AIN1,OUTPUT);//设置两个驱动引脚为输出模式
  pinMode(Motor_AIN2,OUTPUT); 
  pinMode(Motor_BIN1,OUTPUT);//设置两个驱动引脚为输出模式
  pinMode(Motor_BIN2,OUTPUT); 
  pinMode(inputPin, INPUT); // 设置输入引脚为输入模式
  pinMode(outputPin, OUTPUT); // 设置输出引脚为输出模式
  Serial.begin(115200);
  MsTimer2::set(10, control); //10毫秒定时中断函数,入口指针
  MsTimer2::start ();//中断使能 
  attachInterrupt(0, READ_ENCODER_R,CHANGE);//开启对应2号引脚的0号外部中断,Change（二分频）触发方式为FALLING 即下降沿触发(不分频),触发的中断函数为 READ_ENCODER_A 
  attachInterrupt(1, READ_ENCODER_L,CHANGE);
  // 巡线传感器
  pinMode(Sensor_MID_R2,INPUT);
  pinMode(Sensor_MID_R1,INPUT);
  pinMode(Sensor_MID_M,INPUT);
  pinMode(Sensor_MID_L1,INPUT);
  pinMode(Sensor_MID_L2,INPUT);
  
  pinMode(Sensor_RIGHT_R2,INPUT);
  pinMode(Sensor_RIGHT_R1,INPUT);
  pinMode(Sensor_RIGHT_L1,INPUT);
  pinMode(Sensor_RIGHT_L2,INPUT);
  
  pinMode(Sensor_LEFT_R2,INPUT);
  pinMode(Sensor_LEFT_R1,INPUT);
  pinMode(Sensor_LEFT_L1,INPUT);
  pinMode(Sensor_LEFT_L2,INPUT);

  // 初始化OLED并设置其IIC地址为 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // 初始化数组
  for (int i = 0; i < numSamples; i++) {
    readings[i] = 0;
  }
}//
/***********主程序************/
void loop() 
{
  while(Serial.available()>0)//检测串口是否接收到了数据
  {
    Target_Value=Serial.readString();//读取串口字符串
    Target=Target_Value.toFloat();//将字符串转换为浮点型,并将其赋给目标值
  }
  MID_val1=digitalRead(Sensor_MID_R2);
  MID_val2=digitalRead(Sensor_MID_R1);
  MID_val3=digitalRead(Sensor_MID_M);
  MID_val4=digitalRead(Sensor_MID_L1);
  MID_val5=digitalRead(Sensor_MID_L2);

  Right_val1=digitalRead(Sensor_RIGHT_R2);
  Right_val2=digitalRead(Sensor_RIGHT_R1);
  Right_val3=digitalRead(Sensor_RIGHT_L1);
  Right_val4=digitalRead(Sensor_RIGHT_L2);

  Left_val1=digitalRead(Sensor_LEFT_R2);
  Left_val2=digitalRead(Sensor_LEFT_R1);
  Left_val3=digitalRead(Sensor_LEFT_L1);
  Left_val4=digitalRead(Sensor_LEFT_L2);

  // Serial.print(MID_val1);
  // Serial.print(" ");
  // Serial.print(MID_val2);
  // Serial.print(" ");
  // Serial.print(MID_val3);
  // Serial.print(" ");
  // Serial.print(MID_val4);
  // Serial.print(" ");
  // Serial.println(MID_val5);
  
  // Serial.print(Right_val1);
  // Serial.print(" ");
  // Serial.print(Right_val2);
  // Serial.print(" ");
  // Serial.print(Right_val3);
  // Serial.print(" ");
  // Serial.println(Right_val4);

  // Serial.print(Left_val1);
  // Serial.print(" ");
  // Serial.print(Left_val2);
  // Serial.print(" ");
  // Serial.print(Left_val3);
  // Serial.print(" ");
  // Serial.println(Left_val4);

  //测距传感器
  digitalWrite(outputPin, LOW); // 将TRIG引脚置为LOW，确保超声波传感器处于准备状态
  delayMicroseconds(2); // 等待2ms
  digitalWrite(outputPin, HIGH); // 将TRIG引脚置为HIGH，发出超声波脉冲
  delayMicroseconds(10); // 持续10ms
  digitalWrite(outputPin, LOW); // 将TRIG引脚再次置为LOW，超声波脉冲发送完毕

  distance = pulseIn(inputPin, HIGH); // 读取接收到的脉冲持续时间
  distance = distance / 58; // 将脉冲时间转换为距离（cm），计算公式为时间（ms）/ 58
  // Serial.println(distance); // 输出距离值

  if (abs(distance - average) <= spikeThreshold || average == 0)
   {
      // 是正常值，参与滑动平均
      total -= readings[readIndex];
      readings[readIndex] = distance;
      total += distance;

      readIndex = (readIndex + 1) % numSamples;
      average = total / numSamples;
      output=average+1.5;
    } 
    else
    {
      //异常值：不参与平均，只打印
      Serial.print("异常值忽略: ");
      Serial.print(distance, 2);
      Serial.print(" cm\t保留滤波值: ");
      Serial.print(average, 2);
      Serial.println(" cm");
      delay(50);
      return;  // 跳过打印正常值
    }

  // 打印滤波后的距离

  Serial.print(distance);
  Serial.print(" cm\t ");
  Serial.print(average);
  Serial.println(" cm");
  
  //OLED  
  words_display();
  display.display();
}
/**********外部中断触发计数器函数************
*根据转速的方向不同我们将计数器累计为正值或者负值(计数器累计为正值为负值为计数器方向)
*只有方向累计正确了才可以实现正确的调整,否则会出现逆方向满速旋转
*
*※※※※※※超级重点※※※※※※
*
*所谓累计在正确的方向即
*(1)计数器方向
*(2)电机输出方向(控制电机转速方向的接线是正着接还是反着接)
*(3)PI 控制器 里面的误差(Basi)运算是目标值减当前值(Target-Encoder),还是当前值减目标值(Encoder-Target)
*三个方向只有对应上才会有效果否则你接上就是使劲的朝着一个方向(一般来说是反方向)满速旋转

例子里是已经对应好的,如果其他驱动单片机在自己尝试的时候出现满速旋转就是三个方向没对应上

下列函数中由于在A相上升沿触发时,B相是低电平,和A相下降沿触发时B是高电平是一个方向,在这种触发方式下,我们将count累计为正,另一种情况将count累计为负
********************************************/
void READ_ENCODER_R() //外部中断
{
  if (digitalRead(ENCODER_R) ==0) 
  {     
    if (digitalRead(DIRECTION_R) == LOW)      
      Velocity_R--;//根据另外一相电平判定方向
    else      
      Velocity_R++;
  }
  else 
  {    
    if (digitalRead(DIRECTION_R) == LOW)      
    Velocity_R++;//根据另外一相电平判定方向
    else      
    Velocity_R--;
  }
}

void READ_ENCODER_L() //外部中断
{
  if (digitalRead(ENCODER_L) ==0) 
  {     
    if (digitalRead(DIRECTION_L) == LOW) 
    {
      Velocity_L++;//根据另外一相电平判定方向
    }     
    else   
    {
      Velocity_L--;
    }   

  }
  else 
  {    
    if (digitalRead(DIRECTION_L) == LOW)   
    {
      Velocity_L--;//根据另外一相电平判定方向
    }   
    else    
    {
      Velocity_L++;
    }  

  }
}
/**********定时器中断触发函数*********/
void control()
{     
  Velocity_Left=Velocity_L;
  Velocity_Right=Velocity_R;//把采用周期(内部定时中断周期)所累计的脉冲下降沿的个数,赋值给速度
  Velocity_L=0;//将脉冲计数器清零
  Velocity_R=0;

  if(Right_val1==0&&Right_val2==0&&Right_val3==0
  &&Left_val2==0&&Left_val3==0&&Left_val4==0)//直路
  {
    if(MID_val3==1)
    {
      //直走
      value_A=Incremental_PI_A(Velocity_Right,Target*0.5);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      value_B=Incremental_PI_B(Velocity_Left,Target*0.5);
      Set_PWM_R(value_A);//将算好的值输出给电机
      Set_PWM_L(value_B);
    }
    else if(MID_val2==1||(MID_val3==0&&MID_val4==0&&MID_val5==0))
    {
      //慢左转
      value_A=Incremental_PI_A(Velocity_Right,Target);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      value_B=Incremental_PI_B(Velocity_Left,Target*0.3);
      Set_PWM_R(value_A);//将算好的值输出给电机
      Set_PWM_L(value_B);
    }
    else if((MID_val1==0&&MID_val2==0&&MID_val3==0)||MID_val4==1)
    {
      //慢右转
      value_A=Incremental_PI_A(Velocity_Right,Target*0.3);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      value_B=Incremental_PI_B(Velocity_Left,Target);
      Set_PWM_R(value_A);//将算好的值输出给电机
      Set_PWM_L(value_B);
    }
    else if(MID_val1==1&&MID_val2==0&&MID_val3==0&&MID_val4==0&&MID_val5==0)
    {
      //快左转
      value_A=Incremental_PI_A(Velocity_Right,Target);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      value_B=Incremental_PI_B(Velocity_Left,Target*0.1);
      Set_PWM_R(value_A);//将算好的值输出给电机
      Set_PWM_L(value_B);
    }
    else if(MID_val1==0&&MID_val2==0&&MID_val3==0&&MID_val4==0&&MID_val5==1)
    {
      //快右转
      value_A=Incremental_PI_A(Velocity_Right,Target*0.1);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      value_B=Incremental_PI_B(Velocity_Left,Target);
      Set_PWM_R(value_A);//将算好的值输出给电机
      Set_PWM_L(value_B);
    }
    else if(Right_val4==1&&Left_val1==1)//停止
    {
      // value_A=Incremental_PI_A(Velocity_Right,Target*0);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      // value_B=Incremental_PI_B(Velocity_Left,Target*0);
      
      Set_PWM_R(0);//将算好的值输出给电机
      Set_PWM_L(0);
    }
    else
    {
      value_A=Incremental_PI_A(Velocity_Right,Target);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      value_B=Incremental_PI_B(Velocity_Left,Target);
      Set_PWM_R(value_A);//将算好的值输出给电机
      Set_PWM_L(value_B);
    }
  }
  else if(Right_val1==1||Right_val2==1||Right_val3==1||Right_val4==1||Left_val1==1||Left_val2==1||Left_val3==1||Left_val4==1)//岔路
  {
    if((Right_val4==1&&Left_val1==1)||(MID_val1==1&&MID_val5==1))//停止
    {
      // value_A=Incremental_PI_A(Velocity_Right,Target*0);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
      // value_B=Incremental_PI_B(Velocity_Left,Target*0);
      
      Set_PWM_R(0);//将算好的值输出给电机
      Set_PWM_L(0);
    }
    else
    {
        value_A=Incremental_PI_A(Velocity_Right,Target*0.1);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
        value_B=Incremental_PI_B(Velocity_Left,Target*0.1);
        Set_PWM_R(value_A);//将算好的值输出给电机
        Set_PWM_L(value_B);
    }
  }

  // value_A=Incremental_PI_A(Velocity_Right,Target);//通过目标值和当前值在这个函数下算出我们需要调整用的PWM值
  // value_B=Incremental_PI_B(Velocity_Left,Target);


//  Serial.print(Velocity_Left);
//  Serial.print(",");
//  Serial.println(Velocity_Right);
}
/***********PI控制器****************/
int Incremental_PI_B(int Encoder,float Target1)
{  
  // static float Bias,PWM=0,Last_bias=0;//定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias_L=Target1-Encoder;//计算偏差,目标值减去当前值
   PWM_L += Velocity_KP*(Bias_L-Last_bias_L)+Velocity_KI*Bias_L;//增量式PI控制计算
   
   if(PWM_L>PWM_Restrict)
   {
      PWM_L=PWM_Restrict;//限幅
   }

   if(PWM_L<-PWM_Restrict)
   {
    PWM_L=-PWM_Restrict;//限幅  
   }
   Last_bias_L=Bias_L;//保存上一次偏差 
 
   return PWM_L;//增量输出
}

int Incremental_PI_A(int Encoder,float Target1)
{  
  // static float Bias,PWM=0,Last_bias=0;//定义全局静态浮点型变量 PWM,Bias(本次偏差),Last_bias(上次偏差)
   Bias_R=Target1-Encoder;//计算偏差,目标值减去当前值
   PWM_R += Velocity_KP*(Bias_R-Last_bias_R)+Velocity_KI*Bias_R;//增量式PI控制计算
   
   if(PWM_R>PWM_Restrict)
   {
      PWM_R=PWM_Restrict;//限幅
   }

   if(PWM_R<-PWM_Restrict)
   {
    PWM_R=-PWM_Restrict;//限幅  
   }
   Last_bias_R=Bias_R;//保存上一次偏差 
 
   return PWM_R;//增量输出
}

/**********PWM控制函数*********/
void Set_PWM_R(int motora)                        
{ 
  if (motora > 0)//如果算出的PWM为正
  {
    analogWrite(Motor_AIN1,motora);//让PWM在设定正转方向(我们认为的正转方向)正向输出调整，10是死区补偿
    digitalWrite(Motor_AIN2, 0);
  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } 
  else if (motora == 0)//如果PWM为0停车
  {
    digitalWrite(Motor_AIN1, 0);
    digitalWrite(Motor_AIN2, 0);
  } 
  else if (motora < 0)//如果算出的PWM为负
  {
    analogWrite(Motor_AIN1, motora+255);//让PWM在设定反转方向反向输出调整
    digitalWrite(Motor_AIN2,1);
  }
}

void Set_PWM_L(int motorb)                        
{ 
  if (motorb > 0)//如果算出的PWM为正
  {
    analogWrite(Motor_BIN1,motorb);//让PWM在设定正转方向(我们认为的正转方向)正向输出调整，10是死区补偿
    digitalWrite(Motor_BIN2, 0);
  //让PWM在设定正转方向(我们认为的正转方向)正向输出调整
  } 
  else if (motorb == 0)//如果PWM为0停车
  {
    digitalWrite(Motor_BIN1, 0);
    digitalWrite(Motor_BIN2, 0);
  } 
  else if (motorb < 0)//如果算出的PWM为负
  {
    analogWrite(Motor_BIN1, motorb+255);//让PWM在设定反转方向反向输出调整
    digitalWrite(Motor_BIN2,1);
  }
}
//oLED
void words_display()
{
  // 清除屏幕
  display.clearDisplay();
 
  // 设置字体颜色,白色可见
  display.setTextColor(WHITE);
 
  //设置字体大小
  display.setTextSize(1.5);
 
  //设置光标位置
  display.setCursor(0, 0);
  display.print("Before filtering:");
  display.setCursor(0, 10);
  display.print(distance);
  display.print(" ");


  display.setCursor(0, 20);
  display.print("After filtering: ");
  //打印自开发板重置以来的秒数：
  display.setCursor(0, 30);
  display.print(output);
  display.print(" ");
}