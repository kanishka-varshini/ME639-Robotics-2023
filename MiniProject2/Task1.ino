#include <ESP32Encoder.h>

//________________________________________ Link Lengths____________________________________________________
const double l1= 11;
const double l2= 11.5;

//________________________________________ PINS____________________________________________________
const int speedPin2= 14;
const int dirPin2= 27;
const int speedPin1= 13;
const int dirPin1= 12;

const int DT1=33;
const int CLK1=32;
const int DT2=26;
const int CLK2=25;

//________________________________________ Omega and Force Calc Vars____________________________________________________
long p1=0;
long p2=0;

long p1_=0;
long p2_=0;

double q1dot;
double q2dot;

unsigned long t=0;
unsigned long t_=0;
long dt=0;
int tch=20;


//________________________________________ Angular Displacements____________________________________________________
double q1;
double q2;
double q1d;
double q2d;

//________________________________________ Motor Inputs____________________________________________________
double in1;
double in2;

//________________________________________ Desired Values____________________________________________________
double pos[2]={0,0};
double q[2]={0,0};
long progress=0;

//________________________________________ Setup____________________________________________________
ESP32Encoder e1;
ESP32Encoder e2;

void setup() {
  Serial.begin(9600);
  pinMode(speedPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(speedPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(forcePin, INPUT);

  e1.attachHalfQuad ( DT1, CLK1);
  e1.setCount ( -2048 );

  e2.attachHalfQuad ( DT2, CLK2 );
  e2.setCount ( 2048 );

  delay(2000);

}

//________________________________________ Trajectory Function____________________________________________________
double* trajectory(int t){
  double x,y;
  
  //desired trajectory
  x=0;
  y=l1+l2-0.1


  pos[0]=x;
  pos[1]=y;
  return pos;
}

//________________________________________ Inverse Kinematics Function____________________________________________________
double* inv(double x, double y) {
    double theta = acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2)); // in radians
    double q1 = atan2(y, x) - atan2(l2 * sin(theta), l1 + l2 * cos(theta)); // in radians
    double q2 = q1+theta; //absolute angle
    q[0]=q1;
    q[1]=q2;
    return q;
}

//________________________________________ Total Error Function____________________________________________________
float querrt(){
    long en1= e1.getCount()%8192;
    long en2= e2.getCount()%8192;

    q1= 360*en1/8192;
    q2= 360*en2/8192;

    float qerrs=abs(q1-180*q1d/PI)+abs(q2-180*q2d/PI);
    Serial.println(qerrs);
    return qerrs;

}


//________________________________________ Motor Input Function--1--____________________________________________________
double getSpd1(double q, double qdot, double qd){
    Serial.print(q);
    Serial.print("\t");
    double qerr= (qd-q);
    Serial.print("angle error 1: ");
    Serial.print(180*qerr/PI);
    Serial.print("\t");

    double spd= 90*(qerr)-1.55*qdot;

    if(spd >=110)
      spd=110;
    else if(spd <=-110)
      spd=-110;
    Serial.println(spd);
    if(abs(180*qerr/PI)>5 && abs(spd)<75 && abs(qdot)<3){
      int dir=qerr/abs(qerr);
      spd=dir*80;
    }
    if(abs(180*qerr/PI)>5)
      return spd;
    else
      return 0;
}
//________________________________________ Motor Input Function--2--____________________________________________________
double getSpd2(double q, double qdot, double qd){
    Serial.print(q);
    Serial.print("\t");
    double qerr= (qd-q);
    Serial.print("angle error 2: ");
    Serial.print(180*qerr/PI);
    Serial.print("\t");

    double spd= 85*(qerr)-1.55*qdot;

    if(spd >=90)
      spd=90;
    else if(spd <=-90)
      spd=-90;
    Serial.println(spd);
    if(abs(180*qerr/PI)>5 && abs(spd)<90 && abs(qdot)<3){
      int dir=qerr/abs(qerr);
      spd=dir*90;
    }
    if(abs(180*qerr/PI)>5)
      return spd;
    else
      return 0;
}
//__________________________________________________________________________________ Main Loop_____________________________________________________________________
void loop(){
_______________________ Check for progression ____________________________________________________
    if(querrt()<10){
        progress++;
    }
    Serial.println("Progress = "+String(progress));
    
//________________________________________ Omegas ____________________________________________________
    double dp1,dp2;
    t_=millis();
    
    if(t_-t>tch){
        p1_ = e1.getCount();  
        p2_ = e2.getCount();
        dt=t_-t;
        t=t_;

        dp1= p1_-p1;
        dp2= p2_-p2;

        q1dot=dp1*1000*2*PI/(8192*dt);
        q2dot=dp2*1000*2*PI/(8192*dt);

        p1=p1_;
        p2=p2_;
    }
//________________________________________ Destinations ____________________________________________________
    double* posd= trajectory(progress+50);

    double* qd= inv(posd[0], posd[1]);
    q1d= qd[0];
    q2d= qd[1];

    long en1= e1.getCount()%8192;
    long en2= e2.getCount()%8192;


//________________________________________ Motor Power and Actuation ____________________________________________________
    in2= getSpd2(2*PI*en2/8192, q2dot, q2d);
    in1= getSpd1(2*PI*en1/8192, q1dot, q1d);

    if(in1>0){
    digitalWrite(dirPin1, 0);
    analogWrite(speedPin1, in1);
    } 
    else{
        digitalWrite(dirPin1, 1);
        analogWrite(speedPin1, -in1);
    }
    if(in2>0){
        digitalWrite(dirPin2, 0);
        analogWrite(speedPin2, in2);
        } 
    else{
        digitalWrite(dirPin2, 1);
        analogWrite(speedPin2, -in2);
    }

    delay(50);
}