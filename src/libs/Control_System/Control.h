#pragma once 
#include <arduino.h>
#include <math.h>


class Controller
{
private:
    double Trajectory(int tf);
    double position[3];
    double velocity[3];
    double Acceleration[3];
    double g;

    double Mass(double theta1, double ddtheta1, double theta2, double ddtheta2, double theta3,double ddtheta3, int joint);
    double G2(double theta2, double theta3);
    double G3(double theta2, double theta3);

    
    double V1(double theta1, double dtheta1, double theta2, double dtheta2, double theta3, double dtheta3);
    double V23(double theta1, double dtheta1, double theta2, double dtheta2, double theta3, double dtheta3);

 





public:
double PosTrac(double th0, double thf, double t, double tf);
double VelTrac(double th0, double thf, double t, double tf);
double AccTrac(double th0, double thf, double t, double tf);

double Controlsystem(double Atheta1,double dAtheta1,double ServoLaw1, double ServoLaw2,double dAtheta2,double Atheta2,double ServoLaw3,double dAtheta3,double Atheta3, int joint);


double ServoLaw(double ddtheta, double pError, double vError);



};

