#include "Control.h"
#include <math.h>

double Controller::Mass(double theta1, double ddtheta1, double theta2, double ddtheta2, double theta3,double ddtheta3, int joint){

if (joint==1){
double Mass1=ddtheta2*(0.000032669184954733717964837201853562*cos(theta2 + theta3 + 1.569333479860983395711863672254) - 0.000017807919999999997375493554496728*sin(theta2)) + 0.000032669184954733717964837201853562*ddtheta3*cos(theta2 + theta3 + 1.569333479860983395711863672254) + ddtheta1*(0.00028211582749044971299237401724541*cos(2.0*theta2 + 2.0*theta3 - 0.00025333566500056721070216563328695) - 0.0066126117710137419890825584183635*cos(2.0*theta2 - 0.001752443109112493633835037904297) - 0.0088297863675000001076886846362868*cos(2.0*theta2 + theta3) + 0.0088297863675000001076886846362868*cos(theta3) + 0.020477058988725500149330471799658);
return Mass1;
}

if(joint==2){
double Mass2=ddtheta2*(0.0026084163692907500094186890038372*cos(2.0*theta2) - 0.0033106716146813750835387057946946*cos(0.069813170079773183076947630739545*theta1) + 0.0022074465918750000269221711590717*cos(theta3) + 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3) - 0.0026084163692907500094186890038372*cos(2.0*theta2)*cos(0.069813170079773183076947630739545*theta1) - 0.00070225524539062496569979954230689*sin(2.0*theta2)*sin(2.0*theta3) + 0.0022074465918750000269221711590717*cos(2.0*theta2)*cos(theta3) - 0.0022074465918750000269221711590717*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) - 0.0022074465918750000269221711590717*sin(2.0*theta2)*sin(theta3) - 0.0022074465918750000269221711590717*cos(2.0*theta2)*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) + 0.0022074465918750000269221711590717*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(theta3) - 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3)*cos(0.069813170079773183076947630739545*theta1) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(2.0*theta3) + 0.017238061144681376957166207830596) + ddtheta3*(0.0011037232959375000134610855795358*cos(theta3) - 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1) + 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3) - 0.00070225524539062496569979954230689*sin(2.0*theta2)*sin(2.0*theta3) + 0.0011037232959375000134610855795358*cos(2.0*theta2)*cos(theta3) - 0.0011037232959375000134610855795358*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) - 0.0011037232959375000134610855795358*sin(2.0*theta2)*sin(theta3) - 0.0011037232959375000134610855795358*cos(2.0*theta2)*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) + 0.0011037232959375000134610855795358*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(theta3) - 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3)*cos(0.069813170079773183076947630739545*theta1) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(2.0*theta3) + 0.0069535131453906254619945492834177) + ddtheta1*(0.000032669184954733717964837201853562*cos(theta2 + theta3 + 1.569333479860983395711863672254) - 0.000017807919999999997375493554496728*sin(theta2));
return Mass2;
}

if(joint==3){
double Mass3=0.000032669184954733717964837201853562*ddtheta1*cos(theta2 + theta3 + 1.569333479860983395711863672254) - ddtheta2*(0.00035112762269531248284989977115345*cos(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + 2.0*theta3) + 0.00035112762269531248284989977115345*cos(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + 2.0*theta3) - 0.0011037232959375000134610855795358*cos(2.0*theta2 + theta3) + 0.00055186164796875000673054278976792*cos(theta3 - 0.069813170079773183076947630739545*theta1) + 0.00055186164796875000673054278976792*cos(0.069813170079773183076947630739545*theta1 + theta3) + 0.00055186164796875000673054278976792*cos(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + theta3) + 0.00055186164796875000673054278976792*cos(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + theta3) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1) - 0.0011037232959375000134610855795358*cos(theta3) - 0.00070225524539062496569979954230689*cos(2.0*theta2 + 2.0*theta3) - 0.0069535131453906254619945492834177) - ddtheta3*(0.00035112762269531248284989977115345*cos(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.00035112762269531248284989977115345*cos(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) - 0.00070225524539062496569979954230689*cos(2*theta2 + 2*theta3) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739544*theta1) - 0.0069535131453906254619945492834177);
return Mass3;
}

return 0;
}

double Controller::G2(double theta2, double theta3){
  double g=-9.82; 

    double Gravity2=0.040171912499999996859489925782327*g*cos(theta2 + theta3) + 0.10170991059999999472918119636233*g*cos(theta2);

    return Gravity2;
}
double Controller::G3(double theta2, double theta3){
   double g=-9.82; 

    double Gravity3=0.040171912499999996859489925782327*g*cos(theta2 + theta3);
    return Gravity3;
}

double Controller::V1(double theta1, double dtheta1, double theta2, double dtheta2, double theta3, double dtheta3)
{
  double V1=0.000045525453907039893077998704562503*(dtheta2*dtheta2)*sin(2.0*theta2 - 0.069813170079773183076947630739545*theta1) - 0.000045525453907039893077998704562503*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739545*theta1 + 2.0*theta2) + 0.000012256666221467139594319961659608*(dtheta2*dtheta2)*sin(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + 2.0*theta3) - 0.000012256666221467139594319961659608*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + 2.0*theta3) + 0.000012256666221467139594319961659608*(dtheta3*dtheta3)*sin(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + 2.0*theta3) - 0.000012256666221467139594319961659608*(dtheta3*dtheta3)*sin(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + 2.0*theta3) + 0.000038527211090146256733997981447359*(dtheta2*dtheta2)*sin(theta3 - 0.069813170079773183076947630739545*theta1) - 0.000038527211090146256733997981447359*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739545*theta1 + theta3) + 0.000038527211090146256733997981447359*(dtheta2*dtheta2)*sin(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + theta3) - 0.000038527211090146256733997981447359*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + theta3) - 0.00011556424025701406534463733244422*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739545*theta1) - 0.000024513332442934279188639923319215*(dtheta3*dtheta3)*sin(0.069813170079773183076947630739545*theta1) - 0.000032669149999999998743634849196482*(dtheta2*dtheta2)*cos(theta2 + theta3) - 0.000032669149999999998743634849196482*(dtheta3*dtheta3)*cos(theta2 + theta3) - 0.000000047789999999999999785588042843995*(dtheta2*dtheta2)*sin(theta2 + theta3) - 0.000000047789999999999999785588042843995*(dtheta3*dtheta3)*sin(theta2 + theta3) - 0.000017807919999999997375493554496728*(dtheta2*dtheta2)*cos(theta2) - 0.0088297863675000001076886846362868*dtheta1*dtheta3*sin(theta3) + 0.00000014294000000000000807948597348479*dtheta1*dtheta2*cos(2.0*theta2 + 2.0*theta3) + 0.00000014294000000000000807948597348479*dtheta1*dtheta3*cos(2.0*theta2 + 2.0*theta3) - 0.00056423163687499935155938990405389*dtheta1*dtheta2*sin(2.0*theta2 + 2.0*theta3) - 0.00056423163687499935155938990405389*dtheta1*dtheta3*sin(2.0*theta2 + 2.0*theta3) + 0.000024513332442934279188639923319215*dtheta2*dtheta3*sin(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + 2.0*theta3) - 0.000024513332442934279188639923319215*dtheta2*dtheta3*sin(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + 2.0*theta3) + 0.017659572735000000215377369272574*dtheta1*dtheta2*sin(2.0*theta2 + theta3) + 0.0088297863675000001076886846362868*dtheta1*dtheta3*sin(2.0*theta2 + theta3) + 0.000038527211090146256733997981447359*dtheta2*dtheta3*sin(theta3 - 0.069813170079773183076947630739545*theta1) - 0.000038527211090146256733997981447359*dtheta2*dtheta3*sin(0.069813170079773183076947630739545*theta1 + theta3) + 0.000038527211090146256733997981447359*dtheta2*dtheta3*sin(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + theta3) - 0.000038527211090146256733997981447359*dtheta2*dtheta3*sin(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + theta3) - 0.000023176439999999999975327513723755*dtheta1*dtheta2*cos(2.0*theta2) + 0.013225203234326000922838062479059*dtheta1*dtheta2*sin(2.0*theta2) - 0.00004902666488586855837727984663843*dtheta2*dtheta3*sin(0.069813170079773183076947630739545*theta1) - 0.000065338299999999997487269698392964*dtheta2*dtheta3*cos(theta2 + theta3) - 0.00000009557999999999999957117608568799*dtheta2*dtheta3*sin(theta2 + theta3);
return V1;
}
double Controller::V23(double theta1, double dtheta1, double theta2, double dtheta2, double theta3, double dtheta3)
{
  double V2=0.00028211581843749967577969495202694*(dtheta1*dtheta1)*sin(2.0*theta2 + 2.0*theta3) - 0.000000071470000000000004039742986742395*(dtheta1*dtheta1)*cos(2.0*theta2 + 2.0*theta3) - 0.0011037232959375000134610855795358*(dtheta3*dtheta3)*sin(theta3) - 0.00070225524539062496569979954230689*(dtheta2*dtheta2)*sin(2.0*theta2 + 2.0*theta3) - 0.00070225524539062496569979954230689*(dtheta3*dtheta3)*sin(2.0*theta2 + 2.0*theta3) - 0.0013042081846453750047093445019186*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739544*theta1 - 2.0*theta2) + 0.0013042081846453750047093445019186*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2) + 0.00035112762269531248284989977115345*(dtheta2*dtheta2)*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.00035112762269531248284989977115345*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) + 0.00035112762269531248284989977115345*(dtheta3*dtheta3)*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.00035112762269531248284989977115345*(dtheta3*dtheta3)*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) - 0.0088297863675000001076886846362868*(dtheta1*dtheta1)*sin(2.0*theta2 + theta3) - 0.0022074465918750000269221711590717*(dtheta2*dtheta2)*sin(2.0*theta2 + theta3) - 0.0011037232959375000134610855795358*(dtheta3*dtheta3)*sin(2.0*theta2 + theta3) + 0.00055186164796875000673054278976792*(dtheta3*dtheta3)*sin(0.069813170079773183076947630739544*theta1 + theta3) + 0.0011037232959375000134610855795358*(dtheta2*dtheta2)*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + theta3) + 0.0011037232959375000134610855795358*(dtheta2*dtheta2)*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + theta3) + 0.00055186164796875000673054278976792*(dtheta3*dtheta3)*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + theta3) + 0.00055186164796875000673054278976792*(dtheta3*dtheta3)*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + theta3) + 0.000011588219999999999987663756861878*(dtheta1*dtheta1)*cos(2.0*theta2) - 0.0066126016171630004614190312395294*(dtheta1*dtheta1)*sin(2.0*theta2) - 0.0026084163692907500094186890038372*(dtheta2*dtheta2)*sin(2.0*theta2) - 0.00055186164796875000673054278976792*(dtheta3*dtheta3)*sin(0.069813170079773183076947630739544*theta1 - theta3) - 0.0022074465918750000269221711590717*dtheta2*dtheta3*sin(theta3) - 0.0014045104907812499313995990846138*dtheta2*dtheta3*sin(2.0*theta2 + 2.0*theta3) + 0.000091050907814079786155997409125007*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1 - 2.0*theta2) + 0.000091050907814079786155997409125007*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2) - 0.000024513332442934279188639923319215*dtheta1*dtheta2*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.000024513332442934279188639923319215*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) - 0.000024513332442934279188639923319215*dtheta1*dtheta3*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.000024513332442934279188639923319215*dtheta1*dtheta3*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) + 0.00070225524539062496569979954230689*dtheta2*dtheta3*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.00070225524539062496569979954230689*dtheta2*dtheta3*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) - 0.0022074465918750000269221711590717*dtheta2*dtheta3*sin(2.0*theta2 + theta3) + 0.000077054422180292513467995962894719*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1 + theta3) + 0.000038527211090146256733997981447359*dtheta1*dtheta3*sin(0.069813170079773183076947630739544*theta1 + theta3) + 0.0011037232959375000134610855795358*dtheta2*dtheta3*sin(0.069813170079773183076947630739544*theta1 + theta3) - 0.000077054422180292513467995962894719*dtheta1*dtheta2*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + theta3) + 0.000077054422180292513467995962894719*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + theta3) - 0.000038527211090146256733997981447359*dtheta1*dtheta3*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + theta3) + 0.000038527211090146256733997981447359*dtheta1*dtheta3*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + theta3) + 0.0011037232959375000134610855795358*dtheta2*dtheta3*sin(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + theta3) + 0.0011037232959375000134610855795358*dtheta2*dtheta3*sin(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + theta3) + 0.00023112848051402813068927466488844*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1) + 0.00004902666488586855837727984663843*dtheta1*dtheta3*sin(0.069813170079773183076947630739544*theta1) + 0.000077054422180292513467995962894719*dtheta1*dtheta2*sin(0.069813170079773183076947630739544*theta1 - theta3) + 0.000038527211090146256733997981447359*dtheta1*dtheta3*sin(0.069813170079773183076947630739544*theta1 - theta3) - 0.0011037232959375000134610855795358*dtheta2*dtheta3*sin(0.069813170079773183076947630739544*theta1 - theta3);
  return V2;

}

double Controller::PosTrac(double th0, double thf, double t, double tf){
  double a2=((3/pow(tf,2))*(thf-th0));  
  double a3=-((2/pow(tf,3))*(thf-th0)); 
  double Pos=th0+a2*pow(t,2)+a3*pow(t,3);

  while(t>tf){
      return(th0+a2*pow(tf,2)+a3*pow(tf,3));
  }
  return Pos;  
}
double Controller::VelTrac(double th0, double thf, double t, double tf){
  double a2=((3/pow(tf,2))*(thf-th0));  
  double a3=-((2/pow(tf,3))*(thf-th0)); 
  double Vel=2*a2*t+3*a3*pow(t,2);
  while(t>tf){
      return(2*a2*tf+3*a3*pow(tf,2));
  }

  return Vel;  
}
double Controller::AccTrac(double th0, double thf, double t, double tf){
  double a2=((3/pow(tf,2))*(thf-th0));  
  double a3=-((2/pow(tf,3))*(thf-th0)); 
  double Acc=2*a2+6*a3*t;
  while(t>tf){
      return(0);
  }

  return Acc;  
}

double Controller::ServoLaw(double ddtheta, double pError, double vError){

    double kp=16;
    double kv=8;
    double t=ddtheta+kv*vError+kp*pError;
    return t;
}

double Controller::Controlsystem(double Atheta1,double dAtheta1,double ServoLaw1, double ServoLaw2,double dAtheta2,double Atheta2,double ServoLaw3,double dAtheta3,double Atheta3, int joint){
  double Mass1=Mass(Atheta1,ServoLaw1, Atheta2,ServoLaw2,Atheta3,ServoLaw3,joint);
  double Mass2=Mass(Atheta1,ServoLaw1, Atheta2,ServoLaw2,Atheta3,ServoLaw3,joint);
  double Mass3=Mass(Atheta1,ServoLaw1, Atheta2,ServoLaw2,Atheta3,ServoLaw3,joint);
  double Marray[3]={Mass1,Mass2,Mass3};

  double Vel1=V1(Atheta1,dAtheta1,Atheta2,dAtheta2, Atheta3, dAtheta3);
  double Vel2=V23(Atheta1,dAtheta1,Atheta2,dAtheta2, Atheta3, dAtheta3);

  double Velarray[3]={Vel1,Vel2,Vel2};

  double g2=G2(Atheta2,Atheta3);
  double g3=G3(Atheta2,Atheta3);

  double Garray[3]{0,g2,g3};

  double t=Marray[joint-1]+Garray[joint-1]+Velarray[joint-1];


return t;


    }

double Controller::Torque2Pwm(double torque, double velocity, int joint){
  double C1[3]={203.0874,115,242.857};
  double C2[3]={12.142,18.57,12.142};
  double PWM=torque*C1[joint-1]+velocity*C2[joint-1];

  return PWM;
}
