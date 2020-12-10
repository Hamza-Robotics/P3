#include "src/libs/IntervalTimer/IntervalTimer.h"
#include "src/libs/Dynamixel_Lib/Dynamixel.h"
#include "src/libs/Control_System/Control.h"

#define JOINT_1 0x01
#define JOINT_2 0x02
#define JOINT_3 0x03
#define GRIPPER_LEFT 0x04
#define GRIPPER_RIGHT 0x05
#define GRIPPER_BOTH 0x0A // Shadow ID 10

#define WRITE 0x03
#define REQ_WRITE 0x04

#define DIRECTION_PIN 13


Dynamixelclass Dynamix;

Controller Crust;

double theta1_0=0;
double theta1_f=0;
double theta2_0=0;
double theta2_f=0;
double theta3_0=0;
double theta3_f=0;

double timef=10;




void setup() {
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  Serial.begin(9600);
  Dynamix.begin(Serial2, 57600, DIRECTION_PIN);
  delay(1000);
  while (!Serial2) {}
  Dynamix.setStatusReturnLevel(JOINT_1, 01, WRITE);
  Dynamix.setStatusReturnLevel(JOINT_2, 01, WRITE);
  Dynamix.setStatusReturnLevel(JOINT_3, 01, WRITE);
  Dynamix.setStatusReturnLevel(GRIPPER_LEFT, 01, WRITE);
  Dynamix.setStatusReturnLevel(GRIPPER_RIGHT, 01, WRITE);


  Dynamix.setOperationMode(JOINT_1, 16, WRITE);
  Dynamix.setOperationMode(JOINT_2, 16, WRITE);
  Dynamix.setOperationMode(JOINT_3, 16, WRITE);
  Dynamix.setOperationMode(GRIPPER_LEFT, 16, WRITE);
  Dynamix.setOperationMode(GRIPPER_RIGHT, 16, WRITE);


  Dynamix.setMaxPosition(JOINT_1, 4095, WRITE);
  Dynamix.setMinPosition(JOINT_1, 0, WRITE);

  Dynamix.setMaxPosition(JOINT_2, 3280, WRITE);
  Dynamix.setMinPosition(JOINT_2, 754, WRITE);

  Dynamix.setMaxPosition(JOINT_3, 3280, WRITE);
  Dynamix.setMinPosition(JOINT_3, 742, WRITE);

  Dynamix.setMaxPosition(GRIPPER_LEFT, 3200, WRITE);
  Dynamix.setMinPosition(GRIPPER_LEFT, 2000, WRITE);

  Dynamix.setMaxPosition(GRIPPER_RIGHT, 3200, WRITE);
  Dynamix.setMinPosition(GRIPPER_RIGHT, 2000, WRITE);


  Dynamix.setEnableTorque(JOINT_1, 01, WRITE);
  Dynamix.setEnableTorque(JOINT_2, 01, WRITE);
  Dynamix.setEnableTorque(JOINT_3, 01, WRITE);
  Dynamix.setEnableTorque(GRIPPER_LEFT, 01, WRITE);
  Dynamix.setEnableTorque(GRIPPER_RIGHT, 01, WRITE);


  Dynamix.setGain(JOINT_1, 600, WRITE, 'p');
  Dynamix.setGain(JOINT_1, 20, WRITE, 'i');
  Dynamix.setGain(JOINT_1, 20, WRITE, 'd');

  Dynamix.setGain(JOINT_2, 600, WRITE, 'p');
  Dynamix.setGain(JOINT_2, 20, WRITE, 'i');
  Dynamix.setGain(JOINT_2, 20, WRITE, 'd');

  Dynamix.setGain(JOINT_3, 600, WRITE, 'p');
  Dynamix.setGain(JOINT_3, 20, WRITE, 'i');
  Dynamix.setGain(JOINT_3, 20, WRITE, 'd');

  Dynamix.setGain(GRIPPER_LEFT, 600, WRITE, 'p');
  Dynamix.setGain(GRIPPER_LEFT, 20, WRITE, 'i');
  Dynamix.setGain(GRIPPER_LEFT, 20, WRITE, 'd');

  Dynamix.setGain(GRIPPER_RIGHT, 600, WRITE, 'p');
  Dynamix.setGain(GRIPPER_RIGHT, 20, WRITE, 'i');
  Dynamix.setGain(GRIPPER_RIGHT, 20, WRITE, 'd');


  Dynamix.setAccelerationProfile(JOINT_1, 100, WRITE);
  Dynamix.setVelocityProfile(JOINT_1, 200, WRITE);

  Dynamix.setAccelerationProfile(JOINT_2, 50, WRITE);
  Dynamix.setVelocityProfile(JOINT_2, 100, WRITE);

  Dynamix.setAccelerationProfile(JOINT_3, 20, WRITE);
  Dynamix.setVelocityProfile(JOINT_3, 100, WRITE);

  Dynamix.setAccelerationProfile(GRIPPER_LEFT, 20, WRITE);
  Dynamix.setVelocityProfile(GRIPPER_LEFT, 100, WRITE);

  Dynamix.setAccelerationProfile(GRIPPER_RIGHT, 20, WRITE);
  Dynamix.setVelocityProfile(GRIPPER_RIGHT, 100, WRITE);

  Dynamix.setPWM(JOINT_1, 0, REQ_WRITE);
  delay(2);
  Dynamix.setPWM(JOINT_2, -180, REQ_WRITE);
  delay(2);
  Dynamix.setPWM(JOINT_3, -100, REQ_WRITE);
  delay(2);

  delay(2);
  Dynamix.setAction(0xFE);
  delay(2000);
}

void startup()
 {
  for (int i = 0; i < 10; i++) {
    Dynamix.getPosition(JOINT_1);
    delay(6);
    Dynamix.getPosition(JOINT_2);
    delay(6);
    Dynamix.getPosition(JOINT_3);
    delay(6);
    Dynamix.getPosition(GRIPPER_LEFT);
    delay(6);
    Dynamix.getPosition(GRIPPER_RIGHT);
    delay(6);
  }

}


void PrimeMover(int a){
  const char Limb[]={JOINT_1,JOINT_2, JOINT_3};
  int32_t joint = Dynamix.getPosition(Limb[a]);
    int32_t sendjointN = joint - 40;
    int32_t sendjointP = joint + 40;
    delay(6);
    if (digitalRead(10)) {
      Dynamix.setPosition(Limb[a], sendjointN, WRITE);
    while((sendjointN-10)>Dynamix.getPosition(Limb[a])){
      Serial.println("Stuck in while loop");
        delay(6);
        }    
    }
    else if (digitalRead(11)) {
      Dynamix.setPosition(Limb[a], sendjointP, WRITE);
    while((sendjointP+10)<Dynamix.getPosition(Limb[a])){
             delay(6);
        }
    }
    else{
      Dynamix.clearSerialBuffer(); 
       }

}
 void loop() {
    double time_start = millis()/1000;

  while (!Serial2) {}
  startup();
  float hertz = 1000;
  long old_time;

  while (true) {
    old_time = millis();
 double time =millis();
double t=((time/1000)-time_start);
//Serial.println(t);

    int32_t starttid = millis();
double theta[3]={Crust.PosTrac(theta1_0, theta1_f,t,timef),Crust.PosTrac(theta2_0, theta2_f,t,timef),Crust.PosTrac(theta3_0, theta3_f,t,timef)};
double dtheta[3]={Crust.VelTrac(theta1_0, theta1_f,t,timef),Crust.VelTrac(theta2_0, theta2_f,t,timef),Crust.VelTrac(theta3_0, theta3_f,t,timef)};
double ddtheta[3]={Crust.AccTrac(theta1_0, theta1_f,t,timef),Crust.AccTrac(theta2_0, theta2_f,t,timef),Crust.AccTrac(theta3_0, theta3_f,t,timef)};

double Perror1=theta[0]-Dynamix.getPositionRadians(JOINT_1);
double Perror2=theta[1]-Dynamix.getPositionRadians(JOINT_2);
double Perror3=theta[2]-Dynamix.getPositionRadians(JOINT_3);

double Verror1=dtheta[0]-Dynamix.getVelocityRadians(JOINT_1);
double Verror2=dtheta[1]-Dynamix.getVelocityRadians(JOINT_2);
double Verror3=dtheta[2]-Dynamix.getVelocityRadians(JOINT_3);



double t1=Crust.ServoLaw(ddtheta[0],Perror1,Verror1);
double t2=Crust.ServoLaw(ddtheta[1],Perror2,Verror2);
double t3=Crust.ServoLaw(ddtheta[2],Perror3,Verror3);


delay(1);
 double control1=Crust.Controlsystem(Dynamix.getPositionRadians(JOINT_1),Dynamix.getVelocityRadians(JOINT_1),t1,t2,Dynamix.getVelocityRadians(JOINT_2),Dynamix.getPositionRadians(JOINT_2),t3,Dynamix.getVelocityRadians(JOINT_3),Dynamix.getPositionRadians(JOINT_2),1);
 delay(1);
 double control2=Crust.Controlsystem(Dynamix.getPositionRadians(JOINT_1),Dynamix.getVelocityRadians(JOINT_1),t1,t2,Dynamix.getVelocityRadians(JOINT_2),Dynamix.getPositionRadians(JOINT_2),t3,Dynamix.getVelocityRadians(JOINT_3),Dynamix.getPositionRadians(JOINT_2),2);
 delay(1);
 double control3=Crust.Controlsystem(Dynamix.getPositionRadians(JOINT_1),Dynamix.getVelocityRadians(JOINT_1),t1,t2,Dynamix.getVelocityRadians(JOINT_2),Dynamix.getPositionRadians(JOINT_2),t3,Dynamix.getVelocityRadians(JOINT_3),Dynamix.getPositionRadians(JOINT_2),3);

double Pwm1=Crust.Torque2Pwm(control1, dtheta[0], 1);
double Pwm2=Crust.Torque2Pwm(control2, dtheta[1], 2);
double Pwm3=Crust.Torque2Pwm(control3, dtheta[2], 3);

//Serial.println(Pwm2);

//Serial.println(t1);

Dynamix.setPWM(JOINT_1,Pwm1,0x04);
Dynamix.setPWM(JOINT_2,Pwm2,0x04);
Dynamix.setPWM(JOINT_3,Pwm3,0x04);
Dynamix.setAction(0xfe);
delay(1);


//Serial.println(control2);
//Serial.println(Dynamix.getPWM(JOINT_2));


//Serial.println(Dynamix.getVelocityRadians(JOINT_2));
//Serial.println(Dynamix.getPositionRadians(JOINT_2));
    

    //xbee.updateData();

   int32_t sluttid=millis();
  int32_t tid;
  tid=sluttid-starttid;
  Serial.println(tid);
    }
    
      
    
   

   
    //moving();
//    if(Dynamix.getMoving(01)==0){
//    while(Serial2.available()){
//                             Serial2.read();
//}}


//Serial.println(XbeeMeter(xbee.getAccY()));

   while (millis() - old_time < 200){}

 

 }
