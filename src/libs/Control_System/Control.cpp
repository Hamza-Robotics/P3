#include "Control.h"
#include <math.h>


int kp;
int kv;

double th1_0;
double th1_f;
double th2_0;
double th2_f;
double th3_0;
double th3_f;
double t;
double tf;

double g=-9.80665; 

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
    double Gravity2=0.040171912499999996859489925782327*g*cos(theta2 + theta3) + 0.10170991059999999472918119636233*g*cos(theta2);

    return Gravity2;
}
double Controller::G3(double theta2, double theta3){
    double Gravity3=0.040171912499999996859489925782327*g*cos(theta2 + theta3);
    return Gravity3;
}

double Controller::V1(double theta1, double dtheta1, double theta2, double dtheta2, double theta3, double dtheta3)
{
  double V1_tmp;
  double V1_tmp_tmp;
  double b_V1_tmp;
  double b_V1_tmp_tmp;
  double c_V1_tmp;
  double c_V1_tmp_tmp;
  double d_V1_tmp;
  double d_V1_tmp_tmp;
  double e_V1_tmp;
  double f_V1_tmp;
  double g_V1_tmp;
  double h_V1_tmp;
  double i_V1_tmp;
  double j_V1_tmp;
  double k_V1_tmp;
  double l_V1_tmp;
  double m_V1_tmp;
  double n_V1_tmp;
  double o_V1_tmp;
  double p_V1_tmp;
  double q_V1_tmp;
  double r_V1_tmp;
  double s_V1_tmp;

  V1_tmp_tmp = 2.0 * theta2 - 0.069813170079773182 * theta1;
  V1_tmp = std::sin(V1_tmp_tmp + 2.0 * theta3);
  b_V1_tmp_tmp = 0.069813170079773182 * theta1 + 2.0 * theta2;
  b_V1_tmp = std::sin(b_V1_tmp_tmp + 2.0 * theta3);
  c_V1_tmp = std::sin(0.069813170079773182 * theta1);
  c_V1_tmp_tmp = theta2 + theta3;
  d_V1_tmp = std::cos(c_V1_tmp_tmp);
  e_V1_tmp = std::sin(c_V1_tmp_tmp);
  c_V1_tmp_tmp = 2.0 * theta2 + 2.0 * theta3;
  f_V1_tmp = std::cos(c_V1_tmp_tmp);
  g_V1_tmp = std::sin(c_V1_tmp_tmp);
  h_V1_tmp = std::sin(2.0 * theta2 + theta3);
  i_V1_tmp = std::sin(theta3 - 0.069813170079773182 * theta1);
  j_V1_tmp = std::sin(0.069813170079773182 * theta1 + theta3);
  k_V1_tmp = std::sin(V1_tmp_tmp + theta3);
  l_V1_tmp = std::sin(b_V1_tmp_tmp + theta3);
  c_V1_tmp_tmp = dtheta2 * dtheta2;
  m_V1_tmp = 4.5525453907039893E-5 * c_V1_tmp_tmp;
  n_V1_tmp = 1.225666622146714E-5 * c_V1_tmp_tmp;
  d_V1_tmp_tmp = dtheta3 * dtheta3;
  o_V1_tmp = 1.225666622146714E-5 * d_V1_tmp_tmp;
  p_V1_tmp = 3.8527211090146257E-5 * c_V1_tmp_tmp;
  q_V1_tmp = 2.4513332442934279E-5 * dtheta2 * dtheta3;
  r_V1_tmp = 0.0088297863675 * dtheta1 * dtheta3;
  s_V1_tmp = 3.8527211090146257E-5 * dtheta2 * dtheta3;
  return (((((((((((((((((((((((((((((((((m_V1_tmp * std::sin(V1_tmp_tmp) -
    m_V1_tmp * std::sin(b_V1_tmp_tmp)) + n_V1_tmp * V1_tmp) - n_V1_tmp *
    b_V1_tmp) + o_V1_tmp * V1_tmp) - o_V1_tmp * b_V1_tmp) + p_V1_tmp * i_V1_tmp)
    - p_V1_tmp * j_V1_tmp) + p_V1_tmp * k_V1_tmp) - p_V1_tmp * l_V1_tmp) -
    0.00011556424025701407 * c_V1_tmp_tmp * c_V1_tmp) - 2.4513332442934279E-5 *
    d_V1_tmp_tmp * c_V1_tmp) - 3.266915E-5 * c_V1_tmp_tmp * d_V1_tmp) -
    3.266915E-5 * d_V1_tmp_tmp * d_V1_tmp) - 4.779E-8 * c_V1_tmp_tmp * e_V1_tmp)
    - 4.779E-8 * d_V1_tmp_tmp * e_V1_tmp) - 1.7807919999999997E-5 * c_V1_tmp_tmp
    * std::cos(theta2)) - r_V1_tmp * std::sin(theta3)) + 1.4294E-7 * dtheta1 *
    dtheta2 * f_V1_tmp) + 1.4294E-7 * dtheta1 * dtheta3 * f_V1_tmp) -
                       0.00056423163687499935 * dtheta1 * dtheta2 * g_V1_tmp) -
                      0.00056423163687499935 * dtheta1 * dtheta3 * g_V1_tmp) +
                     q_V1_tmp * V1_tmp) - q_V1_tmp * b_V1_tmp) + 0.017659572735 *
                   dtheta1 * dtheta2 * h_V1_tmp) + r_V1_tmp * h_V1_tmp) +
                 s_V1_tmp * i_V1_tmp) - s_V1_tmp * j_V1_tmp) + s_V1_tmp *
               k_V1_tmp) - s_V1_tmp * l_V1_tmp) - 2.317644E-5 * dtheta1 *
             dtheta2 * std::cos(2.0 * theta2)) + 0.013225203234326001 * dtheta1 *
            dtheta2 * std::sin(2.0 * theta2)) - 4.9026664885868558E-5 * dtheta2 *
           dtheta3 * c_V1_tmp) - 6.53383E-5 * dtheta2 * dtheta3 * d_V1_tmp) -
    9.558E-8 * dtheta2 * dtheta3 * e_V1_tmp;

}
double Controller::V23(double theta1, double dtheta1, double theta2, double dtheta2, double theta3, double dtheta3)
{
  double V2_tmp;
  double V2_tmp_tmp;
  double b_V2_tmp;
  double b_V2_tmp_tmp;
  double c_V2_tmp;
  double c_V2_tmp_tmp;
  double d;
  double d1;
  double d2;
  double d_V2_tmp;
  double e_V2_tmp;
  double f_V2_tmp;
  double g_V2_tmp;
  double h_V2_tmp;
  double i_V2_tmp;
  double j_V2_tmp;
  double k_V2_tmp;
  double l_V2_tmp;
  double m_V2_tmp;
  double n_V2_tmp;
  double o_V2_tmp;
  double p_V2_tmp;
  double q_V2_tmp;
  double r_V2_tmp;
  double s_V2_tmp;
  double t_V2_tmp;
  double u_V2_tmp;
  double v_V2_tmp;
  double w_V2_tmp;
  double x_V2_tmp;

  V2_tmp_tmp = 2.0 * theta2 + 2.0 * theta3;
  V2_tmp = std::sin(V2_tmp_tmp);
  b_V2_tmp_tmp = 2.0 * theta2 - 0.069813170079773182 * theta1;
  b_V2_tmp = std::sin(b_V2_tmp_tmp + 2.0 * theta3);
  c_V2_tmp_tmp = 0.069813170079773182 * theta1 + 2.0 * theta2;
  c_V2_tmp = std::sin(c_V2_tmp_tmp + 2.0 * theta3);
  d_V2_tmp = std::sin(2.0 * theta2 + theta3);
  e_V2_tmp = std::sin(b_V2_tmp_tmp + theta3);
  f_V2_tmp = std::sin(c_V2_tmp_tmp + theta3);
  g_V2_tmp = std::sin(2.0 * theta2);
  h_V2_tmp = std::sin(theta3);
  i_V2_tmp = std::sin(0.069813170079773182 * theta1 - 2.0 * theta2);
  b_V2_tmp_tmp = std::sin(c_V2_tmp_tmp);
  c_V2_tmp_tmp = std::sin(0.069813170079773182 * theta1 + theta3);
  j_V2_tmp = std::sin(0.069813170079773182 * theta1);
  k_V2_tmp = std::sin(0.069813170079773182 * theta1 - theta3);
  d = dtheta1 * dtheta1;
  d1 = dtheta3 * dtheta3;
  d2 = dtheta2 * dtheta2;
  l_V2_tmp = 0.001304208184645375 * d2;
  m_V2_tmp = 0.00035112762269531248 * d2;
  n_V2_tmp = 0.00035112762269531248 * d1;
  o_V2_tmp = 0.0011037232959375 * d2;
  p_V2_tmp = 0.00055186164796875 * d1;
  q_V2_tmp = 9.1050907814079786E-5 * dtheta1 * dtheta2;
  r_V2_tmp = 2.4513332442934279E-5 * dtheta1 * dtheta2;
  s_V2_tmp = 2.4513332442934279E-5 * dtheta1 * dtheta3;
  t_V2_tmp = 0.000702255245390625 * dtheta2 * dtheta3;
  u_V2_tmp = 0.002207446591875 * dtheta2 * dtheta3;
  v_V2_tmp = 7.7054422180292513E-5 * dtheta1 * dtheta2;
  w_V2_tmp = 3.8527211090146257E-5 * dtheta1 * dtheta3;
  x_V2_tmp = 0.0011037232959375 * dtheta2 * dtheta3;
  return ((((((((((((((((((((((((((((((((((((((((((((((0.00028211581843749968 *
    d * V2_tmp - 7.147E-8 * d * std::cos(V2_tmp_tmp)) - 0.0011037232959375 * d1 *
    h_V2_tmp) - 0.000702255245390625 * d2 * V2_tmp) - 0.000702255245390625 * d1 *
    V2_tmp) - l_V2_tmp * i_V2_tmp) + l_V2_tmp * b_V2_tmp_tmp) + m_V2_tmp *
    b_V2_tmp) + m_V2_tmp * c_V2_tmp) + n_V2_tmp * b_V2_tmp) + n_V2_tmp *
    c_V2_tmp) - 0.0088297863675 * d * d_V2_tmp) - 0.002207446591875 * d2 *
    d_V2_tmp) - 0.0011037232959375 * (dtheta3 * dtheta3) * d_V2_tmp) + p_V2_tmp *
    c_V2_tmp_tmp) + o_V2_tmp * e_V2_tmp) + o_V2_tmp * f_V2_tmp) + p_V2_tmp *
    e_V2_tmp) + p_V2_tmp * f_V2_tmp) + 1.158822E-5 * d * std::cos(2.0 * theta2))
    - 0.0066126016171630005 * d * g_V2_tmp) - 0.00260841636929075 * d2 *
    g_V2_tmp) - p_V2_tmp * k_V2_tmp) - u_V2_tmp * h_V2_tmp) -
    0.00140451049078125 * dtheta2 * dtheta3 * V2_tmp) + q_V2_tmp * i_V2_tmp) +
    q_V2_tmp * b_V2_tmp_tmp) - r_V2_tmp * b_V2_tmp) + r_V2_tmp * c_V2_tmp) -
    s_V2_tmp * b_V2_tmp) + s_V2_tmp * c_V2_tmp) + t_V2_tmp * b_V2_tmp) +
                        t_V2_tmp * c_V2_tmp) - u_V2_tmp * d_V2_tmp) + v_V2_tmp *
                      c_V2_tmp_tmp) + w_V2_tmp * c_V2_tmp_tmp) + x_V2_tmp *
                    c_V2_tmp_tmp) - v_V2_tmp * e_V2_tmp) + v_V2_tmp * f_V2_tmp)
                 - w_V2_tmp * e_V2_tmp) + w_V2_tmp * f_V2_tmp) + x_V2_tmp *
               e_V2_tmp) + x_V2_tmp * f_V2_tmp) + 0.00023112848051402813 *
             dtheta1 * dtheta2 * j_V2_tmp) + 4.9026664885868558E-5 * dtheta1 *
            dtheta3 * j_V2_tmp) + v_V2_tmp * k_V2_tmp) + w_V2_tmp * k_V2_tmp) -
    x_V2_tmp * k_V2_tmp;

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
      return(2*a2+6*a3*tf);
  }

  return Acc;  
}


double Controller::ServoLaw(double ddtheta, double pError, double vError){

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

double Controller::Torque2Pwm(double torque, double velocity, double joint){
  double C1[3]={203.0974,146.2385,203.0974};
  double C2[3]={131.4431,160.1894,131.4431};
  double PWM=torque*C1[joint-1]+velocity*C2[joint-1];
}

/*
double Pos[3]{{PosTrac(th1_0, th1_f,t,tf),PosTrac(th2_0, th2_f,t,tf),PosTrac(th3_0, th3_f,t,tf)}};
double Vel[3]={VelTrac(th1_0, th1_f,t,tf),VelTrac(th2_0, th2_f,t,tf),VelTrac(th3_0, th3_f,t,tf)};
double Acc[3]={AccTrac(th1_0, th1_f,t,tf),AccTrac(th2_0, th2_f,t,tf),AccTrac(th3_0, th3_f,t,tf)};
*/
/*


double M1(double theta1, double ddtheta1, double theta2, double ddtheta2, double theta3,double ddtheta3)
{

double Mass1=ddtheta2*(0.000032669184954733717964837201853562*cos(theta2 + theta3 + 1.569333479860983395711863672254) - 0.000017807919999999997375493554496728*sin(theta2)) + 0.000032669184954733717964837201853562*ddtheta3*cos(theta2 + theta3 + 1.569333479860983395711863672254) + ddtheta1*(0.00028211582749044971299237401724541*cos(2.0*theta2 + 2.0*theta3 - 0.00025333566500056721070216563328695) - 0.0066126117710137419890825584183635*cos(2.0*theta2 - 0.001752443109112493633835037904297) - 0.0088297863675000001076886846362868*cos(2.0*theta2 + theta3) + 0.0088297863675000001076886846362868*cos(theta3) + 0.020477058988725500149330471799658);

return Mass1;
}
double M2(double theta1, double ddtheta1, double theta2, double ddtheta2, double theta3, double ddtheta3)
{

double Mass2=ddtheta2*(0.0026084163692907500094186890038372*cos(2.0*theta2) - 0.0033106716146813750835387057946946*cos(0.069813170079773183076947630739545*theta1) + 0.0022074465918750000269221711590717*cos(theta3) + 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3) - 0.0026084163692907500094186890038372*cos(2.0*theta2)*cos(0.069813170079773183076947630739545*theta1) - 0.00070225524539062496569979954230689*sin(2.0*theta2)*sin(2.0*theta3) + 0.0022074465918750000269221711590717*cos(2.0*theta2)*cos(theta3) - 0.0022074465918750000269221711590717*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) - 0.0022074465918750000269221711590717*sin(2.0*theta2)*sin(theta3) - 0.0022074465918750000269221711590717*cos(2.0*theta2)*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) + 0.0022074465918750000269221711590717*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(theta3) - 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3)*cos(0.069813170079773183076947630739545*theta1) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(2.0*theta3) + 0.017238061144681376957166207830596) + ddtheta3*(0.0011037232959375000134610855795358*cos(theta3) - 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1) + 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3) - 0.00070225524539062496569979954230689*sin(2.0*theta2)*sin(2.0*theta3) + 0.0011037232959375000134610855795358*cos(2.0*theta2)*cos(theta3) - 0.0011037232959375000134610855795358*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) - 0.0011037232959375000134610855795358*sin(2.0*theta2)*sin(theta3) - 0.0011037232959375000134610855795358*cos(2.0*theta2)*cos(0.069813170079773183076947630739545*theta1)*cos(theta3) + 0.0011037232959375000134610855795358*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(theta3) - 0.00070225524539062496569979954230689*cos(2.0*theta2)*cos(2.0*theta3)*cos(0.069813170079773183076947630739545*theta1) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1)*sin(2.0*theta2)*sin(2.0*theta3) + 0.0069535131453906254619945492834177) + ddtheta1*(0.000032669184954733717964837201853562*cos(theta2 + theta3 + 1.569333479860983395711863672254) - 0.000017807919999999997375493554496728*sin(theta2));

return Mass2;
}
double M3(double theta1, double ddtheta1, double theta2, double ddtheta2, double theta3, double ddtheta3)
{
double Mass3=0.000032669184954733717964837201853562*ddtheta1*cos(theta2 + theta3 + 1.569333479860983395711863672254) - ddtheta2*(0.00035112762269531248284989977115345*cos(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + 2.0*theta3) + 0.00035112762269531248284989977115345*cos(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + 2.0*theta3) - 0.0011037232959375000134610855795358*cos(2.0*theta2 + theta3) + 0.00055186164796875000673054278976792*cos(theta3 - 0.069813170079773183076947630739545*theta1) + 0.00055186164796875000673054278976792*cos(0.069813170079773183076947630739545*theta1 + theta3) + 0.00055186164796875000673054278976792*cos(2.0*theta2 - 0.069813170079773183076947630739545*theta1 + theta3) + 0.00055186164796875000673054278976792*cos(0.069813170079773183076947630739545*theta1 + 2.0*theta2 + theta3) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739545*theta1) - 0.0011037232959375000134610855795358*cos(theta3) - 0.00070225524539062496569979954230689*cos(2.0*theta2 + 2.0*theta3) - 0.0069535131453906254619945492834177) - ddtheta3*(0.00035112762269531248284989977115345*cos(2.0*theta2 - 0.069813170079773183076947630739544*theta1 + 2.0*theta3) + 0.00035112762269531248284989977115345*cos(0.069813170079773183076947630739544*theta1 + 2.0*theta2 + 2.0*theta3) - 0.00070225524539062496569979954230689*cos(2*theta2 + 2*theta3) + 0.00070225524539062496569979954230689*cos(0.069813170079773183076947630739544*theta1) - 0.0069535131453906254619945492834177);
return Mass3;
}
*/