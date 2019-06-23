//#include <ArduinoHardware.h>
//#include <ros.h>
//#include <geometry_msgs/Twist.h>
#include <MsTimer2.h>        //定时中断
#include "digitalWriteFast.h"

//*****编码器*******//
#define c_RightEncoderInterruptA 5
#define c_RightEncoderInterruptB 4
#define c_RightEncoderPinA 18
#define c_RightEncoderPinB 19

#define c_LeftEncoderInterruptA 3
#define c_LeftEncoderInterruptB 2
#define c_LeftEncoderPinA 20
#define c_LeftEncoderPinB 21

volatile long RightEncoderStatus;
volatile long Last_RightEncoderStatus;

volatile long LeftEncoderStatus;
volatile long Last_LeftEncoderStatus;

volatile long right_count = 0;
volatile long left_count = 0;
float R=0.0625;   //轮子半径 单位米
float D=0.42  ;    //两轮间距  单位米
float rate_encoder = 2*PI*R/4000;  //编码器为1000线，计算一个脉冲对应的直线长度
float dL,dR,d;                       //左右两轮走过的距离
float Xw=0;
float Yw=0;
float angle=90*PI/180;                        //机器人X,Y累计距离
int count_A,count_B;                 //编码器单位时间内的计数值

//********************************************************************//////////

#define pls 3               //脉冲引脚  两个驱动器pls口共用一个脉冲
#define dir1 4              //右轮方向控制引脚
#define dir2 5              //左轮方向控制引脚

//ros::NodeHandle nh;         //创建ROS句柄

const String Auto = "A";          //手机端发送字符A，底盘实现自动点到点
const String Telecontrol = "T";   //手机端发送字符T，底盘进入遥控模式
const String Input = "I";         //手机端发送字符I，底盘进入读写模式，读入目标点X,Y的值 
const String GO = "G";            //手机端发送字符G，底盘在遥控模式下向前
const String BACK = "B";          //手机端发送字符B，底盘在遥控模式下后退
const String STOP = "S";          //手机端发送字符S，底盘在遥控模式下停止
const String LEFT = "L";          //手机端发送字符L，底盘在遥控模式下左转
const String RIGHT = "R";         //手机端发送字符R，底盘在遥控模式下右转



float subdivide=1600;         //驱动器的细分，每subdivide 个脉冲电机转一圈
float Hz=800;                 //HZ 脉冲频率  每秒发送HZ个脉冲
//float R=0.0625;               // 轮子半径  单位米
float L=0.42;                 //两轮间距
float V=2*PI*R*(Hz/subdivide);  //轮子速度  m/s
float W=2*V/L;                  //自转角速度 rad/s
float point0[2] = {0,0};        //初始坐标  单位米
float point1[2] = {0,0};        //终点坐标
//float xita;                   //转动的角度
float xita_t;                  //转动的时间
float go_t;                    //直走的时间
int control_flag;             //判定遥控操作还是自动点对点操作
float X, Y;                   //需要到达的世界坐标位置

float  dir=0;
float ang=0;

//******建立ROS的订阅节点********//
//void motorControl(const geometry_msgs::Twist& msg)
//{  
//dir=msg.linear.x;
//ang=msg.angular.z; 
//}
//ros::Subscriber<geometry_msgs::Twist> sub("myrobot",&motorControl);
void SetupEncoders()
{  
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterruptA, HandleLeftMotorInterruptA, CHANGE); //20--A
  attachInterrupt(c_LeftEncoderInterruptB, HandleLeftMotorInterruptB, CHANGE); //21--B

  pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_RightEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_RightEncoderInterruptA, HandleRightMotorInterruptA, CHANGE); //18--A
  attachInterrupt(c_RightEncoderInterruptB, HandleRightMotorInterruptB, CHANGE); //19--B
}
void setup() {
  //nh.initNode();
  //nh.advertise(pub);
  Serial.begin(9600);         //通信频率57600
  SetupEncoders();
  pinMode(pls,OUTPUT);
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(13,OUTPUT);
 // nh.subscribe(sub);
  MsTimer2::set(5, control);  //使用Timer2设置20ms定时中断
  MsTimer2::start();          //使用中断使能
  }
  
void loop() {
//     nh.spinOnce();
     String s = readTtl();
     if(s== Input)
     control_flag=1;
     if(s== Auto) 
     control_flag=2;
     if(s== Telecontrol) 
     control_flag=3; 

     
     if(control_flag==1)
     { 
        get_xy(s);
        point1[0]=X;
        point1[1]=Y;
      }
     if(control_flag==2)
     {
       point_to_point(point0,point1);
       control_flag=0 ;
     }  
     if(control_flag==3)
     {
      if(s== GO)
      forward();
      if(s== BACK)
      back();
      if(s== STOP)
      Stop();
      if(s== LEFT)
      turnleft();
      if(s== RIGHT)
      turnright();
      }

//      if(dir==2.0)
//      forward();
//      if(dir==-2.0)
//      back();
//      if(dir==0.0)
//      Stop();
//      if(ang==2.0)
//      turnleft();
//      if(ang==-2.0)
//      turnright();
////      if(ang==1.0)
////      point_to_point();
//
//      nh.spinOnce();
//      delay(100);
}

void control()
{
  static int flag;   // 延时控制标志
  
  if( ++flag >= 8)    //40ms计算一次
  {
      count_A = left_count;    left_count = 0;  //读左轮编码器数据，并清零
      count_B = right_count;    right_count = 0; 
      dL=count_A*rate_encoder;     //左轮走过的距离
      dR=count_B*rate_encoder;     //右轮走过的距离
//      d=(dR+dL)/2;                 //如果两轮正转编码器输出均为正
//      angle=angle+(dR-dL)/D;
      d=(dR-dL)/2;                 //如果两轮正转编码器输出为一正一负
      angle=angle+(dR+dL)/D;      
      Xw=Xw+d*cos(angle);           //得到X方向累计值 
      Yw=Yw+d*sin(angle);           //得到Y方向累计值
      flag=0;
    }
  }
void HandleLeftMotorInterruptB()
{
  bool tempA = digitalReadFast(c_LeftEncoderPinA);
  bool tempB = digitalReadFast(c_LeftEncoderPinB);
  if(tempA && ~tempB)       LeftEncoderStatus = 1;     //AB=10
  else if(tempA && tempB)    LeftEncoderStatus = 2;     //AB=11
  else if(~tempA && tempB)    LeftEncoderStatus = 3;     //AB=01
  else if(~tempA && ~tempB)   LeftEncoderStatus = 4;     //AB=00
  switch(LeftEncoderStatus - Last_LeftEncoderStatus)
  {
    case 1:   left_count++;
          break;
    case 2:   left_count = left_count + 2;
          break;
    case -3:  left_count++;
          break;
    case -1:  left_count--;
          break;
    case -2:  left_count = left_count - 2;
          break;
    case 3:   left_count--;
          break;
    default:
          break;
  }
  Last_LeftEncoderStatus = LeftEncoderStatus;
}

void HandleLeftMotorInterruptA()
{
  bool tempA = digitalReadFast(c_LeftEncoderPinA);
  bool tempB = digitalReadFast(c_LeftEncoderPinB);
  if(tempA && ~tempB)       LeftEncoderStatus = 1;     //AB=10
  else if(tempA && tempB)     LeftEncoderStatus = 2;     //AB=11
  else if(~tempA && tempB)    LeftEncoderStatus = 3;     //AB=01
  else if(~tempA && ~tempB)     LeftEncoderStatus = 4;     //AB=00
  switch(LeftEncoderStatus - Last_LeftEncoderStatus)
  {
    case 1:   left_count++;
          break;
    case 2:   left_count = left_count + 2;
          break;
    case -3:  left_count++;
          break;
    case -1:  left_count--;
          break;
    case -2:  left_count = left_count - 2;
          break;
    case 3:   left_count--;
          break;
    default:
          break;
  }
  Last_LeftEncoderStatus = LeftEncoderStatus;
}

void HandleRightMotorInterruptB()
{
  bool tempA = digitalReadFast(c_RightEncoderPinA);
  bool tempB = digitalReadFast(c_RightEncoderPinB);
  if(tempA && ~tempB)       RightEncoderStatus = 1;     //AB=10
  else if(tempA && tempB)     RightEncoderStatus = 2;     //AB=11
  else if(~tempA && tempB)    RightEncoderStatus = 3;     //AB=01
  else if(~tempA && ~tempB)   RightEncoderStatus = 4;     //AB=00
  switch(RightEncoderStatus - Last_RightEncoderStatus)
  {
    case 1:   right_count++;
          break;
    case 2:   right_count = right_count + 2;
          break;
    case -3:  right_count++;
          break;
    case -1:  right_count--;
          break;
    case -2:  right_count = right_count - 2;
          break;
    case 3:   right_count--;
          break;
    default:
          break;
  }
  Last_RightEncoderStatus = RightEncoderStatus;
}


void HandleRightMotorInterruptA()
{
  bool tempA = digitalReadFast(c_RightEncoderPinA);
  bool tempB = digitalReadFast(c_RightEncoderPinB);
  if(tempA && ~tempB)       RightEncoderStatus = 1;     //AB=10
  else if(tempA && tempB)     RightEncoderStatus = 2;     //AB=11
  else if(~tempA && tempB)    RightEncoderStatus = 3;     //AB=01
  else if(~tempA && ~tempB)     RightEncoderStatus = 4;     //AB=00
  switch(RightEncoderStatus - Last_RightEncoderStatus)
  {
    case 1:   right_count++;
          break;
    case 2:   right_count = right_count + 2;
          break;
    case -3:  right_count++;
          break;
    case -1:  right_count--;
          break;
    case -2:  right_count = right_count - 2;
          break;
    case 3:   right_count--;
          break;
    default:
          break;
  }
  Last_RightEncoderStatus = RightEncoderStatus;
}


//******从通信串口信息得到目标点X,Y的值******//
void get_xy(String str )
{
  int str_len = str.length() + 1; 
  char buf[str_len];
  str.toCharArray(buf, str_len);
  char x[10];
  char y[10];
  sscanf(buf, "X%s Y%s", &x, &y);
  String tmp = x;
  X = tmp.toFloat();
  String tmp1 = y;
  Y = tmp1.toFloat();  
}


//******自动点到点运动******//
void point_to_point(float p0[2],float p1[2])
{
  static float x,y,r,xita,xita1,xita2,xita3;
  static int rotate;
  x=p1[0]-p0[0];
  y=p1[1]-p0[1];
  r= sqrt((x*x)+(y*y));
  go_t = r/V;
  xita1 = (180*atan(abs(y/x)))/PI;
  if (x>=0)
  {
    if(y>=0) xita2=xita1;
    else     xita2=0-xita1;
    }
    else
    {
      if(y>=0) xita2=180-xita1;
      else     xita2=180+xita1;
    }
    if (xita2>=90)
    {
    rotate=1;
    xita=xita2-90;
    }
    else
    {
      rotate=-1;
      xita=90-xita2;
    }
    xita_t= ((xita*PI)/180)/W;
    autorotation(xita_t,rotate);  
    go_straight(go_t);
    autorotation_back(xita_t,rotate);
//    point0[0]=p1[0];
//    point0[1]=p1[1];
    point0[0]=Xw;
    point0[1]=Yw;
  }
//******点到点运动中的自转过程*****//
void autorotation(float t,int rotate1)
{
  if(rotate1 == 1)
  { 
    turnleft();
    delay(t*1000);
    noTone(pls);
    delay(50);
  }
  else
  {
    turnright();
    delay(t*1000);
    noTone(pls);
    delay(50);    
    }
  }

  //******点到点运动中的回转过程*****//
void autorotation_back(float t,int rotate1)
{
  if(rotate1 == -1)
  {
    turnleft();
    delay(t*1000);
    noTone(pls);
    delay(50);
    }
  else
  {
    turnright();
    delay(t*1000);
    noTone(pls);
    delay(50);
  }
  }
//******点到点运动中的直走过程*****//
void go_straight(float t1)
{
  forward(); 
  delay(t1*1000);
  noTone(pls);
  delay(50);
  
  }

//********方向控制命令*********//  
void forward()
{
 digitalWrite(dir1,HIGH);
 digitalWrite(dir2,LOW);
 digitalWrite(13,HIGH);
 delay(2);
 tone(pls,Hz);

}
 
void back()
{
 digitalWrite(dir1,LOW);
 digitalWrite(dir2,HIGH);
 digitalWrite(13,LOW);
 delay(2);
 tone(pls,Hz);
}
 
void turnleft()
{
 digitalWrite(dir1,HIGH);
 digitalWrite(dir2,HIGH);
 delay(2);
 tone(pls,Hz);
}

void turnright()
{
 digitalWrite(dir1,LOW);
 digitalWrite(dir2,LOW);
 delay(2);
 tone(pls,Hz);
} 

void Stop()
{
  noTone(pls);         
} 
//********方向控制命令*********//  


//********通信接受处理*********//  
String readTtl() 
{
  String comdata = "";
  while (Serial.available())
  {
    comdata += char(Serial.read());
    delay(2);
  }
  return comdata;
}
