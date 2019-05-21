
#ifndef VREP_FUNCT
#define VREP_FUNCT


#include <eigen3/Eigen/Dense>
#include <iostream>

extern "C" {
#include "extApi.h"
    /*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}


using namespace Eigen;

namespace VREP{



class Simulador{
private:
    VectorXd resp{24};
    int portNb,clientID,wheel_rl, wheel_rr,wheel_fl,wheel_fr,joint1, joint2, joint3, joint4, joint5,
    wheel_rl2, wheel_rr2,wheel_fl2,wheel_fr2,joint1_2, joint2_2, joint3_2, joint4_2, joint5_2,
     baseX,Ox,Ox2, baseX2,youBot_Tip,youBot_Tip2, leftTarget, rightTarget, WorldFrame2,  worldx,worldy, baseangle,Force_sensor,
    worldx2,worldy2, baseangle2;
    float prismaticos[3],baseRot[3],tip[3],desired[3],desired2[3],prismaticos2[3],baseRot2[3],tip2[3],force[3],torque[3];
    double Kwh,Krot;
    unsigned char Fsensor_state[1];
public:
    Simulador(VectorXd Q, VectorXd Q2, bool wheelsON);
    VectorXd connect(VectorXd, VectorXd, VectorXd, VectorXd, bool wheelsON);

    ~Simulador (){
        simxStopSimulation(clientID,simx_opmode_oneshot);
        simxFinish(clientID);
        }
};


}


#endif // VREP_FUNCT



