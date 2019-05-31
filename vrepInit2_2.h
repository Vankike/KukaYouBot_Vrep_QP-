
#ifndef VREP_FUNCT
#define VREP_FUNCT

//! Funciona con la escena force_control_experiment.tt
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
    VectorXd resp{29};
    int portNb,clientID,wheel_rl, wheel_rr,wheel_fl,wheel_fr,joint1, joint2, joint3, joint4, joint5,
    wheel_rl2, wheel_rr2,wheel_fl2,wheel_fr2,joint1_2, joint2_2, joint3_2, joint4_2, joint5_2,
     baseX,Ox,Ofs,Ox2, baseX2,youBot_Tip,youBot_Tip2, Pc, Pk,Pj, WorldFrame2,  worldx,worldy, baseangle,Force_sensor,
    worldx2,worldy2, baseangle2;
    int valido[16];
    float prismaticos[3],baseRot[3],tip[3],desired[3],PuntoK[3],PuntoJ[3],prismaticos2[3],baseRot2[3],
          J1[1],J2[1],J3[1],J4[1],J5[1],tip2[3],force[3],torque[3],OriFsensor[3] ;
    double Kwh,Krot;
    unsigned char Fsensor_state[1];
public:
    Simulador(VectorXd Q, bool wheelsON, double KWH, double KROT);
    VectorXd connect(VectorXd, VectorXd, bool wheelsON);

    ~Simulador (){
        simxStopSimulation(clientID,simx_opmode_oneshot);
        simxFinish(clientID);
        }
};


}


#endif // VREP_FUNCT



